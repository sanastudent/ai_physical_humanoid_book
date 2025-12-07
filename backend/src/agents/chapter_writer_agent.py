"""
Chapter Writer Agent for generating full Markdown chapters with exercises, image placeholders, and citation placeholders

Implements FR-002: The system MUST generate chapters based on the outline
"""
import asyncio
import os
import json
import tiktoken
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
import google.generativeai as genai
from dotenv import load_dotenv

from src.config.ai_config import ai_config
from src.skills.content_processing import ContentProcessingSkill
from src.agents.book_outline_agent import ChapterOutline

load_dotenv()

class GeneratedChapter(BaseModel):
    """Represents a generated chapter with full content"""
    title: str
    content: str
    exercises: List[str]
    image_placeholders: List[Dict[str, str]]  # {"id": "img1", "description": "description"}
    citation_placeholders: List[Dict[str, str]]  # {"id": "cite1", "context": "context"}


class ChapterWriterAgent:
    """
    Agent responsible for generating full chapters based on chapter outlines.

    Implements FR-002: The system MUST generate chapters based on the outline
    with full Markdown content, code blocks, exercises, image placeholders, citation placeholders.
    """

    def __init__(self):
        self.gemini_key = ai_config.primary_api_key or os.getenv("GOOGLE_API_KEY")
        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel(ai_config.primary_model_name or 'gemini-pro')
        self.content_skill = ContentProcessingSkill()
        # Initialize tokenizer for token counting
        self.encoder = tiktoken.get_encoding("cl100k_base")

    def _estimate_token_count(self, text: str) -> int:
        """
        Estimate the token count for a given text using tiktoken.

        Args:
            text: The text to count tokens for

        Returns:
            Estimated token count
        """
        return len(self.encoder.encode(text))

    def _truncate_context(self, context: str, max_tokens: int) -> str:
        """
        Truncate the context to fit within token limits.

        Args:
            context: The context to truncate
            max_tokens: Maximum number of tokens allowed

        Returns:
            Truncated context string
        """
        tokens = self.encoder.encode(context)

        if len(tokens) <= max_tokens:
            return context

        # Truncate tokens to the maximum allowed
        truncated_tokens = tokens[:max_tokens]

        # Decode back to text (this might result in incomplete words/sentences)
        truncated_text = self.encoder.decode(truncated_tokens)

        return truncated_text

    async def _summarize_context(self, context: str, max_tokens: int) -> str:
        """
        Summarize the context to fit within token limits.

        Args:
            context: The context to summarize
            max_tokens: Maximum number of tokens allowed

        Returns:
            Summarized context string
        """
        if not context or len(context) == 0:
            return context

        # First try to truncate without summarization if possible
        current_tokens = self._estimate_token_count(context)
        if current_tokens <= max_tokens:
            return context

        # If context is too large, try to summarize
        try:
            # Create a summarization prompt
            prompt = f"""
            Please provide a concise summary of the following book context that maintains the essential information
            but reduces the length to fit within token limits. Focus on the most relevant information for the
            chapter that is being generated.

            Book Context:
            {context}

            Summary:
            """

            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.3,
                    max_output_tokens=max_tokens
                )
            )

            summary = response.text.strip()
            summary_tokens = self._estimate_token_count(summary)

            # If summary is still too long, truncate it
            if summary_tokens > max_tokens:
                summary = self._truncate_context(summary, max_tokens)

            return summary

        except Exception as e:
            print(f"Error summarizing context: {e}")
            # Fallback to truncation if summarization fails
            return self._truncate_context(context, max_tokens)

    async def generate_chapter(
        self,
        chapter_outline: ChapterOutline,
        book_context: str = "",
        include_exercises: bool = True,
        include_images: bool = True,
        include_citations: bool = True
    ) -> GeneratedChapter:
        """
        Generate a full chapter based on the chapter outline.

        Args:
            chapter_outline: The outline for the chapter to generate
            book_context: Context from the rest of the book
            include_exercises: Whether to include exercises in the chapter
            include_images: Whether to include image placeholders
            include_citations: Whether to include citation placeholders

        Returns:
            GeneratedChapter: Complete chapter with all required elements
        """
        # Handle token limit for large books (Edge Case #105)
        max_context_tokens = 2000  # Allow for context and response
        adjusted_book_context = ""

        if book_context:
            context_tokens = self._estimate_token_count(book_context)
            if context_tokens > max_context_tokens:
                print(f"Book context is too large ({context_tokens} tokens). Summarizing to fit within limits...")
                adjusted_book_context = await self._summarize_context(book_context, max_context_tokens)
            else:
                adjusted_book_context = book_context

        # Create a prompt for generating the chapter
        prompt_parts = [
            f"Generate a comprehensive chapter titled '{chapter_outline.title}'.",
            f"The chapter should cover these subtopics: {', '.join(chapter_outline.subtopics)}.",
            f"Learning outcomes for this chapter: {', '.join(chapter_outline.learning_outcomes)}."
        ]

        if adjusted_book_context:
            prompt_parts.append(f"Here is the context from the rest of the book: {adjusted_book_context}")

        prompt_parts.extend([
            "Generate the chapter in Markdown format with:",
            "- Proper headings and subheadings",
            "- Code blocks where appropriate",
            "- Detailed explanations of concepts",
            "- Examples and use cases",
            f"- {'Exercises' if include_exercises else 'No exercises'} (to be placed at the end of the chapter)",
            f"- {'Image placeholders' if include_images else 'No image placeholders'} (using the format ![Image Description](images/image_id.jpg) with descriptive alt text)",
            f"- {'Citation placeholders' if include_citations else 'No citation placeholders'} (using the format [CITATION_ID] where appropriate)",
            "",
            "The content should be appropriate for students with basic AI knowledge.",
            "Include practical examples and hands-on activities.",
            "Make sure the chapter is self-contained but can reference other chapters.",
            "",
            "Return the content in JSON format with the following structure:",
            "{",
            '  "content": "Full Markdown content of the chapter",',
            '  "exercises": ["exercise1", "exercise2", ...],',
            '  "image_placeholders": [{"id": "img1", "description": "description"}, ...],',
            '  "citation_placeholders": [{"id": "cite1", "context": "context"}, ...]',
            "}"
        ])

        prompt = "\n".join(prompt_parts)

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=ai_config.temperature,
                    max_output_tokens=ai_config.max_tokens
                )
            )

            # Parse the response
            chapter_data = json.loads(response.text)

            # Create and return the GeneratedChapter
            generated_chapter = GeneratedChapter(
                title=chapter_outline.title,
                content=chapter_data.get("content", ""),
                exercises=chapter_data.get("exercises", []),
                image_placeholders=chapter_data.get("image_placeholders", []),
                citation_placeholders=chapter_data.get("citation_placeholders", [])
            )

            return generated_chapter

        except Exception as e:
            print(f"Error generating chapter: {e}")
            # Return a default chapter in case of error
            return await self._generate_default_chapter(chapter_outline)

    async def _generate_default_chapter(self, chapter_outline: ChapterOutline) -> GeneratedChapter:
        """Generate a default chapter in case of API failure"""
        print("Generating default chapter due to API error...")

        # Create basic Markdown content
        content_parts = [
            f"# {chapter_outline.title}",
            "",
            "## Introduction",
            f"This chapter covers {', '.join(chapter_outline.subtopics)}.",
            "",
            "## Main Content",
            "Here is the main content of the chapter...",
            "",
            "### Code Example",
            "```python",
            "# Example code",
            "def example_function():",
            "    return 'Hello, World!'",
            "```",
            "",
            "## Summary",
            f"In this chapter, we learned about {', '.join(chapter_outline.subtopics)}.",
        ]

        # Add exercises if specified in the outline
        if chapter_outline.exercises:
            content_parts.extend([
                "",
                "## Exercises",
            ])
            for i, exercise in enumerate(chapter_outline.exercises, 1):
                content_parts.append(f"{i}. {exercise}")

        # Add image placeholders
        image_placeholders = [
            {"id": f"img{i}", "description": f"Image for {chapter_outline.title}"}
            for i in range(1, 3)
        ]

        # Add citation placeholders
        citation_placeholders = [
            {"id": f"cite{i}", "context": f"Citation in {chapter_outline.title}"}
            for i in range(1, 3)
        ]

        content = "\n".join(content_parts)

        return GeneratedChapter(
            title=chapter_outline.title,
            content=content,
            exercises=chapter_outline.exercises,
            image_placeholders=image_placeholders,
            citation_placeholders=citation_placeholders
        )

    async def generate_exercises_from_outcomes(
        self,
        learning_outcomes: List[str],
        subtopics: List[str]
    ) -> List[str]:
        """
        Generate exercises based on learning outcomes and subtopics.

        Args:
            learning_outcomes: List of learning outcomes for the chapter
            subtopics: List of subtopics in the chapter

        Returns:
            List of exercises
        """
        prompt = f"""
        Generate 3-5 practical exercises based on these learning outcomes:
        {', '.join(learning_outcomes)}

        The exercises should relate to these subtopics:
        {', '.join(subtopics)}

        Each exercise should:
        - Reinforce the learning outcomes
        - Be practical and hands-on
        - Include both theoretical and application-based activities
        - Be appropriate for students with basic AI knowledge

        Return only the exercises as a JSON array.
        """

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.5,
                    max_output_tokens=400
                )
            )

            result = json.loads(response.text)
            return result if isinstance(result, list) else []

        except Exception as e:
            print(f"Error generating exercises: {e}")
            return [f"Exercise related to: {', '.join(learning_outcomes[:2])}"]

    async def generate_image_placeholders(
        self,
        chapter_title: str,
        subtopics: List[str],
        count: int = 2
    ) -> List[Dict[str, str]]:
        """
        Generate image placeholders for the chapter.

        Args:
            chapter_title: Title of the chapter
            subtopics: List of subtopics in the chapter
            count: Number of image placeholders to generate

        Returns:
            List of image placeholder dictionaries
        """
        prompt = f"""
        Generate {count} descriptive image placeholders for the chapter titled "{chapter_title}".
        The chapter covers these subtopics: {', '.join(subtopics)}.

        For each image, provide:
        - An ID for the image
        - A description of what the image should depict

        Return the results as a JSON array of objects with "id" and "description" properties.
        """

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.4,
                    max_output_tokens=300
                )
            )

            result = json.loads(response.text)
            return result if isinstance(result, list) else []

        except Exception as e:
            print(f"Error generating image placeholders: {e}")
            return [{"id": f"img{i}", "description": f"Placeholder for {chapter_title}"} for i in range(1, count + 1)]

    async def generate_citation_placeholders(
        self,
        chapter_title: str,
        subtopics: List[str],
        count: int = 2
    ) -> List[Dict[str, str]]:
        """
        Generate citation placeholders for the chapter.

        Args:
            chapter_title: Title of the chapter
            subtopics: List of subtopics in the chapter
            count: Number of citation placeholders to generate

        Returns:
            List of citation placeholder dictionaries
        """
        prompt = f"""
        Generate {count} citation placeholder contexts for the chapter titled "{chapter_title}".
        The chapter covers these subtopics: {', '.join(subtopics)}.

        For each citation, provide:
        - An ID for the citation
        - A description of where in the chapter context it would be used

        Return the results as a JSON array of objects with "id" and "context" properties.
        """

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.3,
                    max_output_tokens=300
                )
            )

            result = json.loads(response.text)
            return result if isinstance(result, list) else []

        except Exception as e:
            print(f"Error generating citation placeholders: {e}")
            return [{"id": f"cite{i}", "context": f"Citation needed in {chapter_title}"} for i in range(1, count + 1)]

    async def add_markdown_formatting(self, content: str) -> str:
        """
        Ensure the content has proper Markdown formatting.

        Args:
            content: Raw content to format

        Returns:
            Properly formatted Markdown content
        """
        # This is a basic implementation - in a full implementation,
        # we would have more sophisticated formatting rules
        if not content.strip().startswith('#'):
            # If there's no heading, add one based on the context
            content = f"# Chapter Content\n\n{content}"

        return content

    async def validate_chapter(self, chapter: GeneratedChapter) -> Dict[str, Any]:
        """
        Validate the generated chapter for completeness and quality.

        Args:
            chapter: The generated chapter to validate

        Returns:
            Dictionary with validation results
        """
        content_lines = chapter.content.split('\n') if chapter.content else []
        has_headings = any(line.strip().startswith('#') for line in content_lines)
        has_code_blocks = '```' in chapter.content if chapter.content else False
        has_exercises = len(chapter.exercises) > 0
        has_images = len(chapter.image_placeholders) > 0
        has_citations = len(chapter.citation_placeholders) > 0

        validation_results = {
            "has_content": bool(chapter.content and chapter.content.strip()),
            "has_headings": has_headings,
            "has_code_blocks": has_code_blocks,
            "has_exercises": has_exercises,
            "has_image_placeholders": has_images,
            "has_citation_placeholders": has_citations,
            "content_length": len(chapter.content) if chapter.content else 0,
            "exercise_count": len(chapter.exercises),
            "image_placeholder_count": len(chapter.image_placeholders),
            "citation_placeholder_count": len(chapter.citation_placeholders),
        }

        validation_results["is_valid"] = validation_results["has_content"]

        return validation_results

    async def generate_complete_chapter_from_outline(
        self,
        chapter_outline: ChapterOutline,
        book_context: str = "",
        validate: bool = True
    ) -> tuple[GeneratedChapter, Dict[str, Any]]:
        """
        Generate a complete chapter and optionally validate it.

        Args:
            chapter_outline: The outline for the chapter to generate
            book_context: Context from the rest of the book
            validate: Whether to validate the generated chapter

        Returns:
            Tuple of (generated_chapter, validation_results)
        """
        # Generate the chapter
        chapter = await self.generate_chapter(
            chapter_outline=chapter_outline,
            book_context=book_context,
            include_exercises=True,
            include_images=True,
            include_citations=True
        )

        # Validate if requested
        validation_results = {}
        if validate:
            validation_results = await self.validate_chapter(chapter)

        return chapter, validation_results