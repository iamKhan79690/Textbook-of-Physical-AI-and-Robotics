"""
PostgresSession - Custom Session implementation for OpenAI Agents SDK
Integrates conversation history with Postgres database
"""

from typing import List, Dict, Any, Optional
import uuid
import logging
from uuid import UUID
from services.db_service import get_pool

logger = logging.getLogger(__name__)


class PostgresSession:
    """Custom Session implementation using Postgres for conversation persistence."""

    def __init__(self, conversation_id: Optional[str] = None, session_id: str = None):
        """
        Initialize PostgresSession.

        Args:
            conversation_id: Optional existing conversation ID (must be valid UUID string)
            session_id: Session ID for grouping conversations
        """
        # Validate conversation_id if provided
        if conversation_id:
            try:
                # Validate it's a proper UUID
                UUID(conversation_id)
                self.conversation_id = conversation_id
            except (ValueError, TypeError):
                # Invalid UUID format, generate new one
                logger.warning(f"Invalid conversation_id format: {conversation_id}. Generating new UUID.")
                self.conversation_id = str(uuid.uuid4())
        else:
            # No conversation_id provided, generate new one
            self.conversation_id = str(uuid.uuid4())
        
        self.session_id = session_id
        self._messages: List[Dict[str, Any]] = []

    async def load_history(self) -> List[Dict[str, Any]]:
        """
        Load conversation history from Postgres.

        Returns:
            List of message dictionaries with role and content
        """
        # Validate conversation_id is a valid UUID before querying
        try:
            conversation_uuid = UUID(self.conversation_id)
        except (ValueError, TypeError) as e:
            logger.warning(f"Cannot load history: invalid conversation_id format: {self.conversation_id}")
            self._messages = []
            return self._messages
        
        pool = get_pool()
        async with pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT role, content, created_at
                FROM messages
                WHERE conversation_id = $1
                ORDER BY created_at ASC
                """,
                conversation_uuid
            )
            self._messages = [
                {"role": row["role"], "content": row["content"]}
                for row in rows
            ]
        return self._messages

    async def save_message(self, role: str, content: str):
        """
        Save a message to Postgres.

        Args:
            role: Message role (user, assistant, system)
            content: Message content
        """
        # Validate conversation_id is a valid UUID
        try:
            conversation_uuid = UUID(self.conversation_id)
        except (ValueError, TypeError) as e:
            logger.error(f"Cannot save message: invalid conversation_id format: {self.conversation_id}")
            raise ValueError(f"Invalid conversation_id format: {self.conversation_id}") from e
        
        pool = get_pool()
        async with pool.acquire() as conn:
            # Ensure conversation exists
            await conn.execute(
                """
                INSERT INTO conversations (id, session_id, created_at)
                VALUES ($1, $2, NOW())
                ON CONFLICT (id) DO NOTHING
                """,
                conversation_uuid,
                self.session_id
            )

            # Save message
            await conn.execute(
                """
                INSERT INTO messages (conversation_id, role, content, created_at)
                VALUES ($1, $2, $3, NOW())
                """,
                conversation_uuid,
                role,
                content
            )

    def get_messages(self) -> List[Dict[str, Any]]:
        """
        Return message history.

        Returns:
            List of messages in memory
        """
        return self._messages

    def add_message(self, role: str, content: str):
        """
        Add message to in-memory history.

        Args:
            role: Message role
            content: Message content
        """
        self._messages.append({"role": role, "content": content})
