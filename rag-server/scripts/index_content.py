"""
Content indexing script for processing markdown files and creating embeddings.
Parses YAML frontmatter, chunks content, generates embeddings, and uploads to Qdrant.
"""
import asyncio
import re
import yaml
import tiktoken
from pathlib import Path
from typing import Dict, List
import sys
import argparse

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import settings
from services.vector_search import VectorSearchService
from services.llm_service import LLMService
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MarkdownParser:
    """Parser for markdown files with YAML frontmatter."""

    def __init__(self, docs_dir: str, base_url: str):
        self.docs_dir = Path(docs_dir)
        self.base_url = base_url.rstrip('/')

    def parse_file(self, file_path: Path) -> Dict:
        """Parse markdown file with YAML frontmatter."""
        content = file_path.read_text(encoding='utf-8')

        # Extract YAML frontmatter
        frontmatter = {}
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                try:
                    frontmatter = yaml.safe_load(parts[1]) or {}
                except yaml.YAMLError:
                    frontmatter = {}
                content = parts[2]

        # Extract chapter/lesson from path
        relative_path = file_path.relative_to(self.docs_dir)
        parts = list(relative_path.parts)
        chapter = parts[0] if len(parts) > 0 else "unknown"
        lesson = file_path.stem

        # Generate Docusaurus URL
        url_path = '/'.join(parts[:-1] + [lesson])
        url = f"{self.base_url}/{url_path}"

        # Parse content sections
        sections = self._parse_sections(content)

        return {
            'file_path': str(file_path),
            'chapter': chapter,
            'lesson': lesson,
            'title': frontmatter.get('title', lesson),
            'sections': sections,
            'url': url
        }

    def _parse_sections(self, content: str) -> List[Dict]:
        """Split content by headings (## H2 level)."""
        sections = []
        current_heading = "Introduction"
        current_content = []

        for line in content.split('\n'):
            if line.startswith('##') and not line.startswith('###'):
                # Save previous section
                if current_content:
                    sections.append({
                        'heading': current_heading,
                        'content': '\n'.join(current_content).strip()
                    })
                current_heading = line.lstrip('#').strip()
                current_content = []
            else:
                current_content.append(line)

        # Add last section
        if current_content:
            sections.append({
                'heading': current_heading,
                'content': '\n'.join(current_content).strip()
            })

        return [s for s in sections if s['content']]  # Filter empty sections


class ContentChunker:
    """Chunks content with token limits and overlap."""

    def __init__(self, max_tokens: int = 800, overlap_tokens: int = 100):
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        # Use cl100k_base encoding (compatible with gpt-5-nano, gpt-4, gpt-3.5-turbo)
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def chunk_sections(self, sections: List[Dict], metadata: Dict) -> List[Dict]:
        """Create chunks from sections with metadata."""
        chunks = []

        for section in sections:
            content = section['content']
            heading = section['heading']
            tokens = self.encoding.encode(content)

            if len(tokens) <= self.max_tokens:
                # Section fits in one chunk
                chunks.append({
                    'heading': heading,
                    'content': content,
                    'token_count': len(tokens),
                    **metadata
                })
            else:
                # Split section into multiple chunks with overlap
                chunk_num = 1
                for i in range(0, len(tokens), self.max_tokens - self.overlap_tokens):
                    chunk_tokens = tokens[i:i + self.max_tokens]
                    chunk_text = self.encoding.decode(chunk_tokens)
                    chunks.append({
                        'heading': f"{heading} (part {chunk_num})",
                        'content': chunk_text,
                        'token_count': len(chunk_tokens),
                        **metadata
                    })
                    chunk_num += 1

        return chunks

    def count_tokens(self, text: str) -> int:
        """Count tokens in text."""
        return len(self.encoding.encode(text))


async def index_textbook_content(
    docs_dir: str,
    collection_name: str,
    base_url: str = "https://devhammad0.github.io/physical-ai-robotics-textbook"
):
    """Main indexing function."""
    logger.info(f"Starting content indexing from {docs_dir}")

    # Initialize services
    vector_service = VectorSearchService(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        collection_name=collection_name
    )
    llm_service = LLMService(api_key=settings.openai_api_key)

    # Create collection if it doesn't exist
    await vector_service.create_collection()

    # Parse all markdown files
    parser = MarkdownParser(docs_dir, base_url)
    chunker = ContentChunker()
    all_chunks = []
    chunk_id = 0

    markdown_files = list(Path(docs_dir).rglob("*.md"))
    logger.info(f"Found {len(markdown_files)} markdown files")

    for md_file in markdown_files:
        try:
            parsed = parser.parse_file(md_file)
            file_chunks = chunker.chunk_sections(
                parsed['sections'],
                metadata={
                    'chapter': parsed['chapter'],
                    'lesson': parsed['lesson'],
                    'file_path': parsed['file_path'],
                    'url': parsed['url']
                }
            )

            # Assign unique IDs
            for chunk in file_chunks:
                chunk['id'] = chunk_id
                chunk_id += 1

            all_chunks.extend(file_chunks)
            logger.info(f"Processed {md_file.name}: {len(file_chunks)} chunks")

        except Exception as e:
            logger.error(f"Error processing {md_file}: {e}")

    logger.info(f"Total chunks created: {len(all_chunks)}")

    # Generate embeddings in batches
    texts = [chunk['content'] for chunk in all_chunks]
    logger.info("Generating embeddings...")
    embeddings = await llm_service.generate_embeddings_batch(texts, batch_size=100)

    # Upload to Qdrant
    logger.info("Uploading to Qdrant...")
    await vector_service.upsert_chunks(all_chunks, embeddings)

    logger.info(f"✓ Indexing complete! Indexed {len(all_chunks)} chunks.")


async def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Index textbook content for RAG chatbot")
    parser.add_argument(
        '--docs-dir',
        default='book-source/docs',
        help='Directory containing markdown files'
    )
    parser.add_argument(
        '--collection',
        default=settings.qdrant_collection_name,
        help='Qdrant collection name'
    )
    parser.add_argument(
        '--base-url',
        default='https://devhammad0.github.io/physical-ai-robotics-textbook',
        help='Base URL for Docusaurus site'
    )

    args = parser.parse_args()

    await index_textbook_content(
        docs_dir=args.docs_dir,
        collection_name=args.collection,
        base_url=args.base_url
    )


if __name__ == "__main__":
    asyncio.run(main())
