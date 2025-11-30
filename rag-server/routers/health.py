"""
Health check endpoint for monitoring backend services.
"""
from fastapi import APIRouter, Depends
from datetime import datetime
import logging
import asyncpg
from qdrant_client import AsyncQdrantClient

from models.schemas import HealthResponse
from services.db_service import get_pool
from config import settings

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("/health", response_model=HealthResponse)
async def health_check(pool: asyncpg.Pool = Depends(get_pool)):
    """
    Check health status of all backend services.

    Returns status for:
    - PostgreSQL database
    - Qdrant vector database
    - OpenAI LLM API
    """
    status_results = {
        "database": "unknown",
        "vector_db": "unknown",
        "llm_api": "unknown"
    }

    # Check PostgreSQL
    try:
        async with pool.acquire() as conn:
            await conn.fetchval("SELECT 1")
        status_results["database"] = "connected"
    except Exception as e:
        status_results["database"] = f"error: {str(e)[:50]}"
        logger.error(f"Database health check failed: {e}")

    # Check Qdrant
    try:
        qdrant = AsyncQdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
        await qdrant.get_collections()
        status_results["vector_db"] = "connected"
    except Exception as e:
        status_results["vector_db"] = f"error: {str(e)[:50]}"
        logger.error(f"Qdrant health check failed: {e}")

    # Check OpenAI (lightweight check - just API key format)
    try:
        if settings.openai_api_key and settings.openai_api_key.startswith("sk-"):
            status_results["llm_api"] = "configured"
        else:
            status_results["llm_api"] = "not configured"
    except Exception as e:
        status_results["llm_api"] = f"error: {str(e)[:50]}"

    # Overall status
    overall_status = "healthy" if all(
        s in ["connected", "configured"] for s in status_results.values()
    ) else "unhealthy"

    return HealthResponse(
        status=overall_status,
        database=status_results["database"],
        vector_db=status_results["vector_db"],
        llm_api=status_results["llm_api"],
        timestamp=datetime.utcnow()
    )
