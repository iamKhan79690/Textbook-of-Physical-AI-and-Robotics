"""
LLM service using OpenAI for embeddings and chat completions.
"""
from openai import AsyncOpenAI
from typing import List, Dict, Optional
import logging

logger = logging.getLogger(__name__)


class LLMService:
    """Service for LLM operations using OpenAI API."""

    def __init__(
        self,
        api_key: str,
        model: str = "gpt-3.5-turbo",
        embedding_model: str = "text-embedding-3-small"
    ):
        self.client = AsyncOpenAI(api_key=api_key)
        self.model = model
        self.embedding_model = embedding_model

    def build_system_prompt(self) -> str:
        """Build system prompt for chatbot behavior."""
        return """You are an AI teaching assistant for a Physical AI & Robotics textbook. Your role is to:

1. Answer questions accurately based ONLY on the provided textbook content
2. Cite specific chapters/lessons when referencing information
3. Use clear, educational language appropriate for students
4. If a question is outside the textbook scope, politely redirect: "This question is outside the textbook scope. Try asking about: ROS 2, Gazebo, Navigation, or AI Integration."
5. For unclear questions, ask for clarification
6. Keep responses concise (under 300 words)

Always base your answers on the retrieved context provided below."""

    def build_context(
        self,
        chunks: List[Dict],
        max_tokens: int = 3000
    ) -> str:
        """Build context string from retrieved chunks."""
        context_parts = []
        token_count = 0

        for chunk in chunks:
            chunk_text = (
                f"\n---\n"
                f"Chapter: {chunk.get('chapter', 'Unknown')}\n"
                f"Lesson: {chunk.get('lesson', 'Unknown')}\n"
                f"Section: {chunk.get('heading', 'Unknown')}\n\n"
                f"{chunk.get('content', '')}\n"
            )
            # Approximate token count (rough estimate: 1 token ≈ 4 characters)
            chunk_tokens = len(chunk_text) // 4

            if token_count + chunk_tokens > max_tokens:
                break

            context_parts.append(chunk_text)
            token_count += chunk_tokens

        return "\n".join(context_parts)

    def build_conversation_context(
        self,
        conversation_history: List[Dict],
        max_messages: int = 5
    ) -> List[Dict]:
        """Build message history for LLM (last N messages)."""
        return [
            {"role": msg['role'], "content": msg['content']}
            for msg in conversation_history[-max_messages:]
        ]

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[Dict],
        conversation_history: Optional[List[Dict]] = None,
        selected_text: Optional[str] = None,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> Optional[str]:
        """Generate LLM response with error handling."""
        try:
            system_prompt = self.build_system_prompt()
            context = self.build_context(retrieved_chunks)

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "system", "content": f"Retrieved Context:\n{context}"}
            ]

            # Add conversation history
            if conversation_history:
                messages.extend(self.build_conversation_context(conversation_history))

            # Build user query with selected text if provided
            user_content = query
            if selected_text:
                user_content = f"Context: \"{selected_text}\"\n\nQuestion: {query}"

            messages.append({"role": "user", "content": user_content})

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature,
                n=1
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"LLM generation failed: {e}")
            return None

    async def generate_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for query text."""
        try:
            response = await self.client.embeddings.create(
                input=text,
                model=self.embedding_model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            return None

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        batch_size: int = 100
    ) -> List[List[float]]:
        """Generate embeddings in batches for indexing."""
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                response = await self.client.embeddings.create(
                    input=batch,
                    model=self.embedding_model
                )
                embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(embeddings)
                logger.info(f"Generated embeddings for batch {i // batch_size + 1}")
            except Exception as e:
                logger.error(f"Batch embedding generation failed: {e}")
                raise

        return all_embeddings


def extract_sources(chunks: List[Dict], top_n: int = 3) -> List[Dict]:
    """Extract top-N unique sources from chunks."""
    seen = set()
    sources = []

    for chunk in chunks:
        key = (chunk.get('chapter', ''), chunk.get('lesson', ''))
        if key not in seen and key != ('', ''):
            sources.append({
                'chapter_name': chunk.get('chapter', 'Unknown'),
                'lesson_title': chunk.get('lesson', 'Unknown'),
                'section_heading': chunk.get('heading', 'Unknown'),
                'url': chunk.get('url', ''),
                'relevance_score': chunk.get('score', 0.0)
            })
            seen.add(key)

        if len(sources) >= top_n:
            break

    return sources
