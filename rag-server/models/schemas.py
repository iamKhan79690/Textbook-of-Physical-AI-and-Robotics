"""
Pydantic models for request/response validation.
"""
from pydantic import BaseModel, Field, field_validator
from typing import Optional, List
from datetime import datetime


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str = Field(..., min_length=1, max_length=1000, description="User query")
    conversation_id: Optional[str] = Field(None, description="Existing conversation ID")
    session_id: str = Field(..., description="Session identifier")
    selected_text: Optional[str] = Field(None, max_length=2000, description="Highlighted text context")

    @field_validator('query')
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """Validate query is not empty or whitespace only."""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()

    @field_validator('conversation_id')
    @classmethod
    def validate_conversation_id(cls, v: Optional[str]) -> Optional[str]:
        """Validate conversation_id is a valid UUID format if provided."""
        if v is None:
            return v
        from uuid import UUID
        try:
            UUID(v)
            return v
        except (ValueError, TypeError):
            # Return None for invalid UUIDs - will generate new one in session
            return None


class Source(BaseModel):
    """Source citation model."""
    chapter_name: str = Field(..., description="Chapter name")
    lesson_title: str = Field(..., description="Lesson title")
    section_heading: str = Field(..., description="Section heading")
    url: str = Field(..., description="Docusaurus page URL")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    message: str = Field(..., description="AI-generated response")
    sources: List[Source] = Field(default_factory=list, description="Source citations")
    conversation_id: str = Field(..., description="Conversation identifier")
    response_time_ms: int = Field(..., ge=0, description="Response time in milliseconds")


class HealthResponse(BaseModel):
    """Response model for health check endpoint."""
    status: str = Field(..., description="Overall health status")
    database: str = Field(..., description="Database connection status")
    vector_db: str = Field(..., description="Vector database connection status")
    llm_api: str = Field(..., description="LLM API connection status")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Timestamp")
