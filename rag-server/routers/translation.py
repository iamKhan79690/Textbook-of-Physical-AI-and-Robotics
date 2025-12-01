"""
Translation router for text translation to Urdu.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from openai import AsyncOpenAI
from config import settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

client = AsyncOpenAI(api_key=settings.openai_api_key)


class TranslationRequest(BaseModel):
    text: str
    target_language: str = "urdu"
    title: str | None = None


class TranslationResponse(BaseModel):
    translated_text: str
    source_language: str = "english"
    target_language: str


@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Translate text to Urdu using OpenAI.
    """
    try:
        logger.info(f"Translating text to {request.target_language}")
        
        # Create translation prompt
        system_prompt = f"""You are a professional translator specializing in technical and educational content.
Translate the following text from English to Urdu ({request.target_language}). 
Maintain technical accuracy, preserve formatting, and ensure the translation is natural and readable.
For technical terms that don't have direct Urdu equivalents, provide the English term in parentheses after the Urdu translation."""

        user_prompt = f"""Title: {request.title or 'Content'}

Text to translate:
{request.text}

Please provide a high-quality Urdu translation."""

        # Call OpenAI API
        response = await client.chat.completions.create(
            model= "gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=4000
        )

        translated_text = response.choices[0].message.content.strip()

        logger.info(f"Translation completed successfully")

        return TranslationResponse(
            translated_text=translated_text,
            source_language="english",
            target_language=request.target_language
        )

    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(
        status_code=500,
            detail=f"Translation failed: {str(e)}"
        )
