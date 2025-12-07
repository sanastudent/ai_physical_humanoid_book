"""
Book Outline Agent for generating book outlines with chapters, subtopics, and learning outcomes

Implements FR-001: The system MUST generate a book outline
"""
import asyncio
import os
import json
import time
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
import google.generativeai as genai
from dotenv import load_dotenv

from src.config.ai_config import ai_config
from src.skills.content_processing import ContentProcessingSkill

load_dotenv()

class ChapterOutline(BaseModel):
    """Represents a chapter in the book outline"""
    title: str
    subtopics: List[str]
    learning_outcomes: List[str]
    exercises: List[str]


class BookOutline(BaseModel):
    """Represents the complete book outline"""
    title: str
    chapters: List[ChapterOutline]
    glossary_terms: List[str]
    references: List[str]


class BookOutlineAgent:
    """
    Agent responsible for generating book outlines from course syllabi.

    Implements FR-001: The system MUST generate a book outline
    with chapters array with titles, subtopics, learning outcomes as specified in the input.
    """

    def __init__(self):
        self.gemini_key = ai_config.primary_api_key or os.getenv("GOOGLE_API_KEY")
        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel(ai_config.primary_model_name or 'gemini-pro')
        self.content_skill = ContentProcessingSkill()

    async def generate_outline(self, course_syllabus: str, book_title: str = "AI-Driven Book") -> BookOutline:
        """
        Generate a complete book outline from a course syllabus.

        Args:
            course_syllabus: The course syllabus to use as input
            book_title: The title for the generated book

        Returns:
            BookOutline: Complete book outline with chapters, learning outcomes, etc.
        """
        # Try to generate the outline with retry logic
        max_retries = 3
        retry_delay = 1  # seconds

        for attempt in range(max_retries):
            try:
                # Create a prompt for generating the book outline
                prompt = f"""
                Generate a comprehensive book outline based on the following course syllabus.
                The outline should include:
                1. Book title
                2. Multiple chapters with detailed titles
                3. For each chapter:
                   - Subtopics to cover
                   - Learning outcomes (what students should know after reading)
                   - Exercises (practical activities for students)
                4. A glossary section with key terms
                5. A references section

                Course Syllabus:
                {course_syllabus}

                Please provide the outline in JSON format with the following structure:
                {{
                    "title": "Book Title",
                    "chapters": [
                        {{
                            "title": "Chapter Title",
                            "subtopics": ["subtopic1", "subtopic2", ...],
                            "learning_outcomes": ["outcome1", "outcome2", ...],
                            "exercises": ["exercise1", "exercise2", ...]
                        }}
                    ],
                    "glossary_terms": ["term1", "term2", ...],
                    "references": ["reference1", "reference2", ...]
                }}

                Make sure the content is appropriate for students with basic AI knowledge.
                Include practical exercises and examples as specified in the requirements.
                """

                response = await self.model.generate_content_async(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=ai_config.temperature,
                        max_output_tokens=ai_config.max_tokens
                    )
                )

                # Parse the response
                outline_json = json.loads(response.text)

                # Validate and create BookOutline object
                book_outline = BookOutline(**outline_json)
                book_outline.title = book_title  # Override with provided title

                # Ensure glossary and references are properly populated
                if not book_outline.glossary_terms or not book_outline.references:
                    # Generate additional glossary and references if missing
                    additional_glossary, additional_references = await self.generate_glossary_and_references(course_syllabus)
                    if not book_outline.glossary_terms:
                        book_outline.glossary_terms = additional_glossary
                    if not book_outline.references:
                        book_outline.references = additional_references

                return book_outline

            except Exception as e:
                print(f"Attempt {attempt + 1} failed: Error generating book outline: {e}")
                if attempt < max_retries - 1:
                    # Wait before retrying
                    await asyncio.sleep(retry_delay * (2 ** attempt))  # Exponential backoff
                else:
                    # All retries failed, return default outline
                    print("All retry attempts failed. Returning default outline.")
                    return await self._generate_default_outline(course_syllabus, book_title)

    async def _generate_default_outline(self, course_syllabus: str, book_title: str) -> BookOutline:
        """Generate a default outline in case of API failure"""
        print("Generating default outline due to API error...")

        # This is a fallback implementation
        default_chapters = [
            ChapterOutline(
                title="Introduction to the Topic",
                subtopics=["Overview", "Key Concepts", "Historical Context"],
                learning_outcomes=["Understand basic concepts", "Identify key principles"],
                exercises=["Exercise 1", "Exercise 2"]
            ),
            ChapterOutline(
                title="Core Principles",
                subtopics=["Principle 1", "Principle 2", "Application Examples"],
                learning_outcomes=["Apply core principles", "Analyze use cases"],
                exercises=["Exercise 3", "Exercise 4"]
            )
        ]

        return BookOutline(
            title=book_title,
            chapters=default_chapters,
            glossary_terms=["Key term 1", "Key term 2"],
            references=["Reference 1", "Reference 2"]
        )

    async def generate_learning_outcomes(self, chapter_title: str, subtopics: List[str]) -> List[str]:
        """
        Generate specific learning outcomes for a chapter.

        Args:
            chapter_title: Title of the chapter
            subtopics: List of subtopics in the chapter

        Returns:
            List of learning outcomes
        """
        prompt = f"""
        Generate 3-5 specific, measurable learning outcomes for the chapter titled "{chapter_title}".
        The chapter covers these subtopics: {', '.join(subtopics)}.

        Each learning outcome should:
        - Start with an action verb (e.g., "Explain", "Apply", "Analyze")
        - Be specific and measurable
        - Focus on what students should be able to do after reading the chapter

        Return only the learning outcomes as a JSON array.
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
            return result if isinstance(result, list) else [chapter_title]  # fallback

        except Exception as e:
            print(f"Error generating learning outcomes: {e}")
            return [f"Understand the basics of {chapter_title}"]

    async def generate_exercises(self, chapter_title: str, subtopics: List[str]) -> List[str]:
        """
        Generate practical exercises for a chapter.

        Args:
            chapter_title: Title of the chapter
            subtopics: List of subtopics in the chapter

        Returns:
            List of exercises
        """
        prompt = f"""
        Generate 2-4 practical exercises for the chapter titled "{chapter_title}".
        The chapter covers these subtopics: {', '.join(subtopics)}.

        Exercises should be:
        - Practical and hands-on
        - Related to the chapter content
        - Appropriate for students with basic AI knowledge
        - Include both theoretical and application-based activities

        Return only the exercises as a JSON array.
        """

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.5,
                    max_output_tokens=300
                )
            )

            result = json.loads(response.text)
            return result if isinstance(result, list) else [f"Exercise for {chapter_title}"]  # fallback

        except Exception as e:
            print(f"Error generating exercises: {e}")
            return [f"Basic exercise for {chapter_title}"]

    async def validate_outline(self, outline: BookOutline) -> Dict[str, Any]:
        """
        Validate the generated outline for completeness and quality.

        Args:
            outline: The book outline to validate

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "title_present": bool(outline.title and outline.title.strip()),
            "has_chapters": len(outline.chapters) > 0,
            "chapters_have_content": all([
                chap.title and chap.subtopics and chap.learning_outcomes
                for chap in outline.chapters
            ]),
            "glossary_present": len(outline.glossary_terms) > 0,
            "references_present": len(outline.references) > 0,
            "total_chapters": len(outline.chapters),
            "total_glossary_terms": len(outline.glossary_terms),
            "total_references": len(outline.references)
        }

        validation_results["is_valid"] = all([
            validation_results["title_present"],
            validation_results["has_chapters"],
            validation_results["chapters_have_content"]
        ])

        return validation_results

    async def generate_glossary_and_references(self, course_syllabus: str) -> tuple[List[str], List[str]]:
        """
        Generate glossary terms and references for the book.

        Args:
            course_syllabus: The course syllabus to use for context

        Returns:
            Tuple of (glossary_terms, references)
        """
        # Try to generate glossary and references with retry logic
        max_retries = 3
        retry_delay = 1  # seconds

        for attempt in range(max_retries):
            try:
                prompt = f"""
                Based on this course syllabus, generate:
                1. A list of 8-12 key glossary terms that students should know
                2. A list of 5-8 references/citations that would be appropriate for the book

                Course Syllabus:
                {course_syllabus}

                Return the results as a JSON object with keys "glossary_terms" and "references".
                """

                response = await self.model.generate_content_async(
                    prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.3,
                        max_output_tokens=500
                    )
                )

                result = json.loads(response.text)
                glossary_terms = result.get("glossary_terms", [])
                references = result.get("references", [])

                return glossary_terms, references

            except Exception as e:
                print(f"Attempt {attempt + 1} failed: Error generating glossary and references: {e}")
                if attempt < max_retries - 1:
                    # Wait before retrying
                    await asyncio.sleep(retry_delay * (2 ** attempt))  # Exponential backoff
                else:
                    # All retries failed, return fallback values
                    print("All retry attempts failed. Returning fallback values.")
                    return ["Term 1", "Term 2"], ["Reference 1", "Reference 2"]  # fallback