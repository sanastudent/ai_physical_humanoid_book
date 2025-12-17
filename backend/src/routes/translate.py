"""
Translation API Route

Provides endpoint for translating chapter content to Urdu
Migrated from Anthropic Claude to OpenAI GPT-4
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
import openai
import os

router = APIRouter(prefix="/translate", tags=["Translation"])


class TranslateRequest(BaseModel):
    """Request model for translation"""
    content: str
    target_language: str = "urdu"
    chapter_id: Optional[str] = None


class TranslateResponse(BaseModel):
    """Response model for translation"""
    original_chapter_id: Optional[str]
    translated_content: str
    target_language: str
    processing_time_ms: Optional[int]


@router.post("", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate chapter content to target language (primarily Urdu)

    Args:
        request: TranslateRequest with content and target language

    Returns:
        TranslateResponse with translated content
    """
    try:
        import time
        start_time = time.time()

        # Initialize OpenAI client (migrated from Anthropic)
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise HTTPException(
                status_code=500,
                detail="OPENAI_API_KEY environment variable not set"
            )

        openai.api_key = api_key

        # Create translation prompt
        language_map = {
            "urdu": "Urdu (اردو)",
            "arabic": "Arabic (العربية)",
            "spanish": "Spanish (Español)",
            "french": "French (Français)",
            "german": "German (Deutsch)",
        }

        target_lang = language_map.get(request.target_language.lower(), request.target_language)

        system_prompt = f"""You are a professional translator specializing in technical documentation.
Your task is to translate the provided technical content into {target_lang} while:
1. Preserving all technical terms and code examples exactly as they appear
2. Maintaining markdown formatting
3. Keeping the same structure and organization
4. Ensuring natural, fluent translation that reads well in the target language
5. Preserving any URLs, links, and references

IMPORTANT:
- Do NOT translate code blocks, variable names, function names, or technical keywords
- DO translate explanatory text, comments, and documentation
- Maintain all markdown formatting (headers, lists, code blocks, etc.)
"""

        # Call OpenAI API for translation (migrated from Claude)
        response = openai.ChatCompletion.create(
            model="gpt-4",  # Using GPT-4 for high-quality translation
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Translate the following content to {target_lang}:\n\n{request.content}"}
            ],
            max_tokens=8000,
            temperature=0.3  # Lower temperature for more consistent translation
        )

        # Extract translated content
        translated_content = response.choices[0].message.content

        processing_time = int((time.time() - start_time) * 1000)

        response = TranslateResponse(
            original_chapter_id=request.chapter_id,
            translated_content=translated_content,
            target_language=request.target_language,
            processing_time_ms=processing_time
        )

        return response

    except openai.error.OpenAIError as e:
        print(f"OpenAI API error in translation: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation API error: {str(e)}"
        )
    except Exception as e:
        print(f"Error in translation endpoint: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )
