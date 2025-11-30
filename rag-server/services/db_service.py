"""
Database service module with AsyncPG connection pooling and CRUD operations.
"""
import asyncpg
from typing import Optional, Dict, List
import logging
import json
from datetime import datetime
from uuid import UUID, uuid4

logger = logging.getLogger(__name__)

# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def init_db_pool(database_url: str, min_size: int = 2, max_size: int = 10):
    """Initialize database connection pool on application startup."""
    global _pool

    try:
        _pool = await asyncpg.create_pool(
            dsn=database_url,
            min_size=min_size,
            max_size=max_size,
            command_timeout=60,
            max_inactive_connection_lifetime=300.0
        )
        logger.info("Database connection pool initialized")
    except Exception as e:
        logger.error(f"Failed to initialize database pool: {e}")
        raise


async def close_db_pool():
    """Close database connection pool on application shutdown."""
    global _pool
    if _pool:
        await _pool.close()
        logger.info("Database connection pool closed")


def get_pool() -> asyncpg.Pool:
    """Get database pool for dependency injection."""
    if _pool is None:
        raise RuntimeError("Database pool not initialized")
    return _pool


class ConversationService:
    """Service for managing conversation persistence."""

    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool

    async def create_or_get_conversation(
        self,
        session_id: str,
        conversation_id: Optional[str] = None
    ) -> str:
        """Create new conversation or get existing one."""
        async with self.pool.acquire() as conn:
            if conversation_id:
                # Check if conversation exists
                result = await conn.fetchrow(
                    "SELECT id FROM conversations WHERE id = $1",
                    UUID(conversation_id)
                )
                if result:
                    return conversation_id

            # Create new conversation
            new_id = await conn.fetchval(
                """
                INSERT INTO conversations (session_id)
                VALUES ($1)
                RETURNING id
                """,
                session_id
            )
            return str(new_id)

    async def add_message(
        self,
        conversation_id: str,
        role: str,
        content: str,
        selected_text: Optional[str] = None,
        sources: Optional[List[Dict]] = None
    ) -> str:
        """Add message to conversation."""
        async with self.pool.acquire() as conn:
            message_id = await conn.fetchval(
                """
                INSERT INTO messages
                (conversation_id, role, content, selected_text, sources)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id
                """,
                UUID(conversation_id),
                role,
                content,
                selected_text,
                json.dumps(sources or [])
            )

            # Update conversation timestamp
            await conn.execute(
                """
                UPDATE conversations
                SET updated_at = NOW()
                WHERE id = $1
                """,
                UUID(conversation_id)
            )

            return str(message_id)

    async def get_conversation_history(
        self,
        conversation_id: str,
        limit: int = 10
    ) -> List[Dict]:
        """Get recent messages from conversation."""
        async with self.pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content, sources, created_at
                FROM messages
                WHERE conversation_id = $1
                ORDER BY created_at DESC
                LIMIT $2
                """,
                UUID(conversation_id),
                limit
            )

            # Reverse to chronological order
            messages = []
            for row in reversed(rows):
                messages.append({
                    'role': row['role'],
                    'content': row['content'],
                    'sources': json.loads(row['sources']) if row['sources'] else [],
                    'created_at': row['created_at'].isoformat()
                })

            return messages


class AnalyticsService:
    """Service for logging query analytics."""

    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool

    async def log_query(
        self,
        query: str,
        response_time_ms: int,
        success: bool,
        session_id: str,
        conversation_id: Optional[str] = None,
        error_message: Optional[str] = None
    ):
        """Log query for analytics."""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO chat_analytics
                (query, response_time_ms, success, error_message, session_id, conversation_id)
                VALUES ($1, $2, $3, $4, $5, $6)
                """,
                query,
                response_time_ms,
                success,
                error_message,
                session_id,
                UUID(conversation_id) if conversation_id else None
            )
