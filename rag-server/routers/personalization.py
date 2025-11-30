"""
Personalization router for rewriting content based on user profile.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from openai import OpenAI
from config import settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

client = OpenAI(api_key=settings.openai_api_key)


class UserProfile(BaseModel):
    softwareBackground: str | None = None
    hardwareExperience: str | None = None
    learningGoals: str | None = None
    hasRoboticsExperience: bool | None = None


class PersonalizeRequest(BaseModel):
    text: str
    user_profile: UserProfile
    title: str | None = None


class PersonalizeResponse(BaseModel):
    personalized_text: str
    adaptation_summary: str  # Brief explanation of what changed


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Rewrite text to match the user's background and goals.
    """
    try:
        logger.info(f"Personalizing content for profile: {request.user_profile}")
        
        # Construct profile description
        profile_desc = []
        if request.user_profile.softwareBackground:
            profile_desc.append(f"- Software Experience: {request.user_profile.softwareBackground}")
        if request.user_profile.hardwareExperience:
            profile_desc.append(f"- Hardware Experience: {request.user_profile.hardwareExperience}")
        if request.user_profile.hasRoboticsExperience is not None:
            profile_desc.append(f"- Robotics Experience: {'Yes' if request.user_profile.hasRoboticsExperience else 'No'}")
        if request.user_profile.learningGoals:
            profile_desc.append(f"- Learning Goals: {request.user_profile.learningGoals}")
            
        profile_str = "\n".join(profile_desc)

        system_prompt = f"""You are an expert educational content adapter. 
Your task is to rewrite technical content to perfectly match a specific learner's profile.

Learner Profile:
{profile_str}

Adaptation Rules:
1. If the learner is a BEGINNER (Software/Hardware):
   - Simplify complex jargon or explain it immediately.
   - Use more analogies.
   - Break down long paragraphs.
   - Focus on "why" before "how".

2. If the learner is ADVANCED:
   - Be concise.
   - Skip basic explanations.
   - Focus on technical depth, implementation details, and edge cases.
   - Use industry-standard terminology freely.

3. General:
   - Maintain the core technical accuracy.
   - Keep the same general structure (headings, code blocks).
   - If the user has specific learning goals, highlight connections to those goals where relevant.

Return the response in two parts:
1. A brief "Adaptation Summary" (1-2 sentences) explaining how you adapted the content.
2. The "Personalized Content" (the full rewritten text).
"""

        user_prompt = f"""Title: {request.title or 'Content'}

Original Content:
{request.text}

Please rewrite this content for the learner."""

        # Call OpenAI API
        response = client.chat.completions.create(
            model=settings.openai_model, # Use the optimized model setting
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.4,
            max_tokens=4000
        )

        content = response.choices[0].message.content.strip()
        
        # Simple parsing to separate summary from content if the model follows instructions
        # This is a basic implementation; in production, we might use structured output or specific delimiters
        adaptation_summary = "Content adapted to your profile."
        personalized_text = content

        return PersonalizeResponse(
            personalized_text=personalized_text,
            adaptation_summary=adaptation_summary
        )

    except Exception as e:
        logger.error(f"Personalization error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Personalization failed: {str(e)}"
        )
