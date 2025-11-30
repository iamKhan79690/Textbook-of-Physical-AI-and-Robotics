"""
ChatKit token generation endpoint for frontend authentication.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import jwt
import time
from config import settings
import logging

router = APIRouter()
logger = logging.getLogger(__name__)


class TokenRequest(BaseModel):
    user_id: str


class TokenResponse(BaseModel):
    token: str
    expires_in: int


@router.post("/chatkit/token", response_model=TokenResponse)
async def create_chatkit_token(request: TokenRequest):
    """
    Generate ChatKit client token for frontend authentication.
    
    The token is used by the frontend ChatKit component to authenticate
    with the chat backend. It includes the user ID and domain key.
    
    Args:
        request: TokenRequest with user_id
        
    Returns:
        TokenResponse with JWT token and expiry time
    """
    try:
        logger.info(f"Generating ChatKit token for user: {request.user_id}")
        
        # Create JWT payload
        payload = {
            "sub": request.user_id,
            "domain_key": settings.chatkit_domain_key,
            "iat": int(time.time()),
            "exp": int(time.time()) + 3600  # 1 hour expiry
        }
        
        # Generate JWT token
        token = jwt.encode(payload, settings.openai_api_key, algorithm="HS256")
        
        logger.info(f"✓ Token generated for user: {request.user_id}")
        
        return TokenResponse(
            token=token,
            expires_in=3600
        )
        
    except Exception as e:
        logger.error(f"Failed to generate ChatKit token: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to generate token: {str(e)}"
        )


@router.get("/chatkit/status")
async def chatkit_status():
    """Health check for ChatKit integration."""
    return {
        "status": "ok",
        "chatkit_enabled": True,
        "domain_key": settings.chatkit_domain_key
    }
