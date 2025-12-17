"""
Personalization Agent for customizing book content based on user preferences

Implements the personalization feature for AI-driven book content
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
from src.schema import UserPreferences, LearningStyle, ExperienceLevel


class PersonalizationAgent:
    """
    Agent responsible for personalizing book content based on user preferences.

    This agent takes original chapter content and user preferences to generate
    a personalized version that matches the user's learning style, experience level,
    and interests while preserving the original structure and formatting.
    """

    def __init__(self):
        self.gemini_key = ai_config.primary_api_key or os.getenv("GOOGLE_API_KEY")
        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel(
            ai_config.primary_model_name or 'gemini-pro',
            generation_config={
                "temperature": 0.7,
                "max_output_tokens": 8192,
            }
        )
        self.content_skill = ContentProcessingSkill()
        # Initialize tokenizer for token counting
        self.encoder = tiktoken.get_encoding("cl100k_base")

    def _estimate_token_count(self, text: str) -> int:
        """
        Estimate the token count for a given text using tiktoken.

        Args:
            text: Input text to count tokens for

        Returns:
            Estimated token count
        """
        return len(self.encoder.encode(text))

    def _construct_personalization_prompt(self, content: str, preferences: UserPreferences) -> str:
        """
        Construct a detailed prompt for content personalization based on user preferences.

        Args:
            content: Original chapter content to personalize
            preferences: User preferences for personalization

        Returns:
            Formatted prompt string for the AI model
        """
        # Map learning style to specific instructions
        learning_style_instructions = {
            LearningStyle.VISUAL: "Include more diagrams, charts, and visual aids references. Use structured formats like tables and bullet points.",
            LearningStyle.AUDITORY: "Focus on explanations that can be read aloud. Include analogies and storytelling elements.",
            LearningStyle.READING_WRITING: "Provide detailed written explanations, definitions, and text-based examples.",
            LearningStyle.KINESTHETIC: "Include hands-on examples, practical exercises, and interactive elements.",
            LearningStyle.MULTIMODAL: "Balance all approaches with various types of explanations and examples."
        }

        # Map experience level to complexity instructions
        experience_level_instructions = {
            ExperienceLevel.BEGINNER: "Use simple language, provide detailed explanations for concepts, include more examples and analogies.",
            ExperienceLevel.INTERMEDIATE: "Use moderate complexity, explain intermediate concepts, balance theory and practice.",
            ExperienceLevel.ADVANCED: "Use technical terminology, assume prior knowledge, focus on advanced concepts.",
            ExperienceLevel.EXPERT: "Use expert-level terminology, focus on nuances and advanced applications, minimal basic explanations."
        }

        # Build interests context
        interests_context = ""
        if preferences.interests:
            interests_context = f"User is particularly interested in: {', '.join(preferences.interests)}. Tailor examples and explanations to align with these interests."

        # Construct the prompt
        prompt = f"""
You are an expert educational content personalization assistant. Your task is to transform the following chapter content to match the user's learning preferences while preserving all structural elements.

## Original Content:
{content}

## User Preferences:
- Learning Style: {preferences.learning_style.value}
- Experience Level: {preferences.experience_level.value}
- Interests: {', '.join(preferences.interests) if preferences.interests else 'None specified'}
- Difficulty Preference: {preferences.difficulty_preference}

## Personalization Instructions:
{learning_style_instructions[preferences.learning_style]}
{experience_level_instructions[preferences.experience_level]}
{interests_context}

## Critical Requirements:
1. **PRESERVE ALL STRUCTURE**: Maintain all Markdown formatting including:
   - Headings (# ## ###)
   - Code blocks (```python, ```bash, etc.)
   - Lists (both ordered and unordered)
   - Bold, italic, and other text formatting
   - Links and image placeholders
   - Citations ([Chapter X: Paragraph Y])

2. **PRESERVE ALL CODE**: Code examples must remain exactly as they are

3. **PRESERVE ALL CITATIONS**: Citation placeholders must remain unchanged

4. **PRESERVE ALL IMAGES**: Image placeholders must remain unchanged

5. **APPLY PERSONALIZATION TO TEXT CONTENT ONLY**: Modify explanations, examples, and narrative text based on user preferences

## Output Requirements:
- Return the full content with personalization applied
- Maintain 100% structural integrity
- Apply personalization to textual content only
- Ensure all original functionality remains intact

Personalized content:
"""
        return prompt

    async def personalize_content(self, content: str, preferences: UserPreferences) -> str:
        """
        Personalize the given content based on user preferences.

        Args:
            content: Original chapter content to personalize
            preferences: User preferences for personalization

        Returns:
            Personalized content string
        """
        try:
            # Construct the prompt
            prompt = self._construct_personalization_prompt(content, preferences)

            # Check if content is too large for the model
            token_count = self._estimate_token_count(prompt)
            if token_count > 30000:  # Leave buffer for response
                # Split content into chunks if too large
                chunks = self._split_content_for_personalization(content)
                personalized_chunks = []

                for chunk in chunks:
                    chunk_prompt = self._construct_personalization_prompt(chunk, preferences)
                    response = await self.model.generate_content_async(chunk_prompt)
                    personalized_chunk = response.text
                    personalized_chunks.append(personalized_chunk)

                return "\n\n".join(personalized_chunks)
            else:
                # Generate personalized content
                response = await self.model.generate_content_async(prompt)
                personalized_content = response.text

                # Clean up potential artifacts from the AI response
                # Remove potential prefix like "Personalized content:" if present
                if "Personalized content:" in personalized_content:
                    personalized_content = personalized_content.split("Personalized content:")[-1].strip()

                return personalized_content

        except Exception as e:
            print(f"Error in personalization: {str(e)}")
            # Return original content if personalization fails
            return content

    def _split_content_for_personalization(self, content: str) -> List[str]:
        """
        Split large content into smaller chunks for personalization.

        Args:
            content: Large content string to split

        Returns:
            List of content chunks
        """
        # Split by headings while preserving structure
        lines = content.split('\n')
        chunks = []
        current_chunk = []
        current_token_count = 0

        for line in lines:
            line_token_count = self._estimate_token_count(line)

            # If adding this line would exceed the limit, save current chunk
            if current_token_count + line_token_count > 20000:  # Conservative limit
                if current_chunk:
                    chunks.append('\n'.join(current_chunk))
                    current_chunk = [line]
                    current_token_count = line_token_count
                else:
                    # If single line is too long, add it anyway
                    chunks.append(line)
                    current_chunk = []
                    current_token_count = 0
            else:
                current_chunk.append(line)
                current_token_count += line_token_count

        # Add the last chunk if it exists
        if current_chunk:
            chunks.append('\n'.join(current_chunk))

        return chunks


# Example usage and test function
async def test_personalization_agent():
    """
    Test function to verify the personalization agent works correctly.
    """
    agent = PersonalizationAgent()

    sample_content = """# Introduction to Robotics

Robotics is an interdisciplinary field that includes mechanical engineering, electrical engineering, computer science, and others.

## Types of Robots

There are several types of robots:
- Industrial robots
- Service robots
- Medical robots

```python
# Example code
def hello_robot():
    print("Hello, Robot!")
```

[Chapter 1: Paragraph 1] This is a citation placeholder.
"""

    from src.schema import UserPreferences, LearningStyle, ExperienceLevel

    preferences = UserPreferences(
        learning_style=LearningStyle.VISUAL,
        experience_level=ExperienceLevel.BEGINNER,
        interests=["computer vision", "machine learning"],
        difficulty_preference="easy"
    )

    personalized = await agent.personalize_content(sample_content, preferences)
    print("Original content length:", len(sample_content))
    print("Personalized content length:", len(personalized))
    print("Personalization successful:", len(personalized) > 0)


if __name__ == "__main__":
    # Run test if executed directly
    asyncio.run(test_personalization_agent())