"""
Vector search service using Qdrant for semantic similarity search.
"""
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
from typing import List, Dict
import logging
import asyncio

logger = logging.getLogger(__name__)


class VectorSearchService:
    """Service for vector similarity search using Qdrant."""

    def __init__(self, url: str, api_key: str, collection_name: str):
        """
        Initialize Qdrant client.
        
        Args:
            url: Qdrant Cloud URL
            api_key: Qdrant API key
            collection_name: Collection name
        """
        self.client = AsyncQdrantClient(
            url=url,
            api_key=api_key
        )
        self.collection_name = collection_name

    async def collection_exists(self) -> bool:
        """Check if collection exists."""
        try:
            collections = await asyncio.wait_for(
                self.client.get_collections(),
                timeout=30.0
            )
            return any(c.name == self.collection_name for c in collections.collections)
        except asyncio.TimeoutError:
            logger.error("Timeout checking collection existence (30s)")
            raise
        except Exception as e:
            logger.error(f"Failed to check collection existence: {e}")
            return False

    async def create_collection(self, vector_size: int = 1536):
        """Create collection for text-embedding-3-small (1536 dimensions)."""
        try:
            exists = await self.collection_exists()
            if not exists:
                await asyncio.wait_for(
                    self.client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=VectorParams(
                            size=vector_size,
                            distance=Distance.COSINE
                        )
                    ),
                    timeout=30.0
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection already exists: {self.collection_name}")
        except asyncio.TimeoutError:
            logger.error("Timeout creating collection (30s)")
            raise ConnectionError("Qdrant connection timeout. Check your network and Qdrant cluster status.")
        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            raise

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5
    ) -> List[Dict]:
        """Search for similar chunks using vector similarity."""
        try:
            results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True
            )

            chunks = []
            for result in results:
                chunks.append({
                    'score': result.score,
                    'chapter': result.payload.get('chapter', ''),
                    'lesson': result.payload.get('lesson', ''),
                    'heading': result.payload.get('heading', ''),
                    'content': result.payload.get('content', ''),
                    'url': result.payload.get('url', ''),
                    'file_path': result.payload.get('file_path', '')
                })

            return chunks
        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            return []

    async def upsert_chunks(
        self,
        chunks: List[Dict],
        embeddings: List[List[float]],
        batch_size: int = 50,
        max_retries: int = 3
    ):
        """
        Upsert chunks with embeddings to Qdrant in batches.
        
        Args:
            chunks: List of chunk dictionaries
            embeddings: List of embedding vectors
            batch_size: Number of points to upload per batch (default: 50)
            max_retries: Maximum retry attempts per batch (default: 3)
        """
        if len(chunks) != len(embeddings):
            raise ValueError(f"Chunks ({len(chunks)}) and embeddings ({len(embeddings)}) count mismatch")
        
        total_chunks = len(chunks)
        total_batches = (total_chunks + batch_size - 1) // batch_size
        
        logger.info(f"Uploading {total_chunks} chunks in {total_batches} batches (batch size: {batch_size})")
        
        for batch_idx in range(total_batches):
            start_idx = batch_idx * batch_size
            end_idx = min(start_idx + batch_size, total_chunks)
            batch_chunks = chunks[start_idx:end_idx]
            batch_embeddings = embeddings[start_idx:end_idx]
            
            # Build points for this batch
            points = []
            for idx, (chunk, embedding) in enumerate(zip(batch_chunks, batch_embeddings)):
                global_idx = start_idx + idx
                points.append(PointStruct(
                    id=chunk.get('id', global_idx),
                    vector=embedding,
                    payload={
                        'chapter': chunk['chapter'],
                        'lesson': chunk['lesson'],
                        'heading': chunk['heading'],
                        'content': chunk['content'],
                        'file_path': chunk['file_path'],
                        'url': chunk['url'],
                        'token_count': chunk.get('token_count', 0)
                    }
                ))
            
            # Retry logic for each batch
            last_exception = None
            for attempt in range(max_retries):
                try:
                    await asyncio.wait_for(
                        self.client.upsert(
                            collection_name=self.collection_name,
                            points=points,
                            wait=True
                        ),
                        timeout=120.0  # 2 minutes per batch
                    )
                    logger.info(f"✓ Batch {batch_idx + 1}/{total_batches}: Uploaded {len(points)} chunks ({end_idx}/{total_chunks} total)")
                    break  # Success, move to next batch
                except asyncio.TimeoutError:
                    last_exception = TimeoutError(f"Batch {batch_idx + 1} upload timeout (120s)")
                    wait_time = 2 ** attempt  # Exponential backoff: 1s, 2s, 4s
                    logger.warning(f"Batch {batch_idx + 1} timeout (attempt {attempt + 1}/{max_retries}). Retrying in {wait_time}s...")
                    if attempt < max_retries - 1:
                        await asyncio.sleep(wait_time)
                except Exception as e:
                    last_exception = e
                    wait_time = 2 ** attempt
                    logger.warning(f"Batch {batch_idx + 1} failed (attempt {attempt + 1}/{max_retries}): {e}. Retrying in {wait_time}s...")
                    if attempt < max_retries - 1:
                        await asyncio.sleep(wait_time)
            else:
                # All retries exhausted
                error_msg = f"Failed to upload batch {batch_idx + 1} after {max_retries} attempts"
                logger.error(f"{error_msg}: {last_exception}")
                raise RuntimeError(f"{error_msg}: {last_exception}") from last_exception
        
        logger.info(f"✓ Successfully uploaded all {total_chunks} chunks to Qdrant")
