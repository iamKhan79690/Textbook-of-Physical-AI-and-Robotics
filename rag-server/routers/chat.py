"""
Chat router implementing agent-based RAG pipeline for question answering.
"""
from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
import time
import logging
import asyncpg
import json

from models.schemas import ChatRequest, ChatResponse
from services.db_service import get_pool, AnalyticsService
from services.agent_service import AgentService
from services.session_service import PostgresSession
from config import settings

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    pool: asyncpg.Pool = Depends(get_pool)
):
    """
    Process chat query using OpenAI Agents SDK with autonomous tool usage.

    The agent autonomously decides when to search the textbook based on the query.

    Steps:
    1. Create PostgresSession for conversation persistence
    2. Run agent with query (agent decides tool usage)
    3. Extract structured sources from tool calls
    4. Log analytics
    5. Return response with sources
    """
    start_time = time.time()

    try:
        # Initialize services
        analytics_service = AnalyticsService(pool)

        # Create or load PostgresSession
        session = PostgresSession(
            conversation_id=request.conversation_id,
            session_id=request.session_id
        )

        # Run agent with session
        agent_result = await AgentService.run_agent(
            query=request.query,
            session=session,
            selected_text=request.selected_text
        )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Log analytics
        await analytics_service.log_query(
            query=request.query,
            response_time_ms=response_time_ms,
            success=True,
            session_id=request.session_id,
            conversation_id=session.conversation_id
        )

        # Return structured response with sources
        return ChatResponse(
            message=agent_result["message"],
            sources=agent_result["sources"],  # Structured sources extracted from tool calls
            conversation_id=session.conversation_id,
            response_time_ms=response_time_ms
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat endpoint error: {e}", exc_info=True)

        # Log failed query
        response_time_ms = int((time.time() - start_time) * 1000)
        try:
            analytics_service = AnalyticsService(pool)
            await analytics_service.log_query(
                query=request.query,
                response_time_ms=response_time_ms,
                success=False,
                session_id=request.session_id,
                error_message=str(e)
            )
        except:
            pass

        raise HTTPException(
            status_code=500,
            detail="An error occurred processing your request. Please try again."
        )


@router.post("/chat/stream")
async def chat_stream(
    request: ChatRequest,
    pool: asyncpg.Pool = Depends(get_pool)
):
    """
    Process chat query with streaming support using OpenAI Agents SDK.
    
    Returns Server-Sent Events (SSE) stream with:
    - text events: {"delta": "..."} for incremental text
    - done event: {"message": "...", "sources": [...], "conversation_id": "..."} when complete
    """
    start_time = time.time()
    
    async def generate_stream():
        try:
            # Create or load PostgresSession
            session = PostgresSession(
                conversation_id=request.conversation_id,
                session_id=request.session_id
            )
            
            # Stream agent responses
            async for event in AgentService.run_agent_stream(
                query=request.query,
                session=session,
                selected_text=request.selected_text
            ):
                # Format as Server-Sent Event
                event_type = event.get("type", "text")
                event_data = event.get("data", {})
                
                # Send SSE formatted event
                yield f"event: {event_type}\n"
                yield f"data: {json.dumps(event_data)}\n\n"
            
            # Calculate response time
            response_time_ms = int((time.time() - start_time) * 1000)
            
            # Log analytics (non-blocking)
            try:
                analytics_service = AnalyticsService(pool)
                await analytics_service.log_query(
                    query=request.query,
                    response_time_ms=response_time_ms,
                    success=True,
                    session_id=request.session_id,
                    conversation_id=session.conversation_id
                )
            except Exception as e:
                logger.warning(f"Failed to log analytics: {e}")
                
        except Exception as e:
            # Enhanced error logging with diagnostic information
            logger.error("=== Streaming Endpoint Error Diagnostics ===")
            logger.error(f"Error type: {type(e).__name__}")
            logger.error(f"Error message: {str(e)}")
            logger.error(f"Request query: {request.query[:100] if len(request.query) > 100 else request.query}")
            logger.error(f"Session ID: {request.session_id}")
            logger.error(f"Conversation ID: {request.conversation_id}")
            
            # Check for connection/DNS errors
            error_str = str(e)
            if "getaddrinfo" in error_str or "DNS" in error_str or "connection" in error_str.lower():
                logger.error("DNS/Connection error detected in router")
                logger.error("This suggests the OpenAI API endpoint cannot be resolved")
                logger.error("Check network connectivity and DNS settings")
            
            # Log full exception details
            logger.error(f"Full exception details:", exc_info=True)
            logger.error("============================================")
            
            # Send error event
            error_data = {"error": str(e)}
            yield f"event: error\n"
            yield f"data: {json.dumps(error_data)}\n\n"
            
            # Log failed query
            response_time_ms = int((time.time() - start_time) * 1000)
            try:
                analytics_service = AnalyticsService(pool)
                await analytics_service.log_query(
                    query=request.query,
                    response_time_ms=response_time_ms,
                    success=False,
                    session_id=request.session_id,
                    error_message=str(e)
                )
            except:
                pass
    
    return StreamingResponse(
        generate_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        }
    )
