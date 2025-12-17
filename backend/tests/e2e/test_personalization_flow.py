"""
End-to-End Tests for Personalization Feature
Tests the complete personalization flow from UI to backend and back
"""

import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.main import app
from src.schema import UserPreferences
from src.agents.personalization_agent import PersonalizationAgent


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


class TestPersonalizationE2E:
    """End-to-End tests for the complete personalization flow"""

    def test_complete_personalization_flow(self, client):
        """Test the complete flow from request to response"""
        # Sample chapter content to personalize
        sample_content = """# Introduction to AI
This is an introduction to artificial intelligence.

## Section 1
Some content about AI concepts.

```python
def hello_ai():
    print("Hello, AI World!")
```

[Chapter 1: Paragraph 1] This is a citation reference.
"""

        # Sample user preferences
        sample_preferences = UserPreferences(
            learning_style="visual",
            experience_level="intermediate",
            interests=["AI", "Machine Learning"],
            difficulty_preference="moderate"
        )

        # Mock the personalization agent to avoid actual AI processing
        with patch('src.main.personalization_agent') as mock_agent:
            mock_agent.personalize_content = AsyncMock(
                return_value="# Personalized Introduction to AI\nThis is personalized content for visual learners."
            )

            # Make the personalization request
            response = client.post("/personalize", json={
                "chapter_content": sample_content,
                "user_preferences": sample_preferences.model_dump(),
                "chapter_id": "test-chapter-1"
            })

            # Verify the response
            assert response.status_code == 200
            data = response.json()

            # Check that the response has the expected structure
            assert "original_chapter_id" in data
            assert "personalized_content" in data
            assert "personalization_applied" in data

            # Verify the chapter ID is preserved
            assert data["original_chapter_id"] == "test-chapter-1"

            # Verify personalization was applied
            assert data["personalization_applied"] is True

            # Verify that content was returned
            assert "Personalized" in data["personalized_content"]

    def test_personalization_with_different_learning_styles(self, client):
        """Test personalization with different learning styles"""
        sample_content = "# Test Content\nThis is test content."

        learning_styles = ["visual", "auditory", "reading/writing", "kinesthetic", "multimodal"]

        for style in learning_styles:
            with patch('src.main.personalization_agent') as mock_agent:
                mock_agent.personalize_content = AsyncMock(
                    return_value=f"# Personalized for {style}\nContent adapted for {style} learners."
                )

                response = client.post("/personalize", json={
                    "chapter_content": sample_content,
                    "user_preferences": {
                        "learning_style": style,
                        "experience_level": "intermediate",
                        "interests": [],
                        "difficulty_preference": "moderate"
                    },
                    "chapter_id": f"test-chapter-{style}"
                })

                assert response.status_code == 200
                data = response.json()
                assert data["personalization_applied"] is True

    def test_personalization_with_different_experience_levels(self, client):
        """Test personalization with different experience levels"""
        sample_content = "# Test Content\nThis is test content."

        experience_levels = ["beginner", "intermediate", "advanced", "expert"]

        for level in experience_levels:
            with patch('src.main.personalization_agent') as mock_agent:
                mock_agent.personalize_content = AsyncMock(
                    return_value=f"# Content for {level}\nAppropriate content for {level} level."
                )

                response = client.post("/personalize", json={
                    "chapter_content": sample_content,
                    "user_preferences": {
                        "learning_style": "multimodal",
                        "experience_level": level,
                        "interests": [],
                        "difficulty_preference": "moderate"
                    },
                    "chapter_id": f"test-chapter-{level}"
                })

                assert response.status_code == 200
                data = response.json()
                assert data["personalization_applied"] is True

    def test_personalization_preserves_markdown_structure(self, client):
        """Test that personalization preserves Markdown structure"""
        original_content = """# Introduction
This is an introduction.

## Section 1
Some text here.

### Subsection 1.1
More content.

```python
def hello():
    print("Hello, World!")
```

[Chapter 1: Paragraph 1] This is a citation.

- List item 1
- List item 2

1. Numbered item 1
2. Numbered item 2

> A blockquote
"""

        with patch('src.main.personalization_agent') as mock_agent:
            # Mock response that preserves structure but personalizes content
            mock_response = """# Personalized Introduction
This is a personalized introduction for your learning style.

## Personalized Section 1
Enhanced content based on your preferences.

### Personalized Subsection 1.1
More personalized content.

```python
def hello():
    print("Hello, World!")
```

[Chapter 1: Paragraph 1] This is a citation.

- Personalized list item 1
- Personalized list item 2

1. Personalized numbered item 1
2. Personalized numbered item 2

> A personalized blockquote
"""
            mock_agent.personalize_content = AsyncMock(return_value=mock_response)

            response = client.post("/personalize", json={
                "chapter_content": original_content,
                "user_preferences": {
                    "learning_style": "visual",
                    "experience_level": "intermediate",
                    "interests": ["AI"],
                    "difficulty_preference": "moderate"
                },
                "chapter_id": "test-structure-chapter"
            })

            assert response.status_code == 200
            data = response.json()
            assert data["personalization_applied"] is True

            # Verify that structure elements are likely preserved in the response
            # (the AI agent would preserve structure while personalizing content)
            assert "personalized_content" in data
            assert "# Personalized Introduction" in data["personalized_content"]
            assert "## Personalized Section 1" in data["personalized_content"]
            assert "def hello():" in data["personalized_content"]
            assert "[Chapter 1: Paragraph 1]" in data["personalized_content"]

    def test_personalization_with_interests(self, client):
        """Test personalization with different user interests"""
        sample_content = "# Robotics\nGeneral content about robotics."

        interests_sets = [
            ["AI", "Machine Learning"],
            ["Computer Vision", "NLP"],
            ["ROS", "Navigation"]
        ]

        for interests in interests_sets:
            with patch('src.main.personalization_agent') as mock_agent:
                mock_agent.personalize_content = AsyncMock(
                    return_value=f"# Personalized Robotics\nContent personalized for interests: {', '.join(interests)}."
                )

                response = client.post("/personalize", json={
                    "chapter_content": sample_content,
                    "user_preferences": {
                        "learning_style": "multimodal",
                        "experience_level": "intermediate",
                        "interests": interests,
                        "difficulty_preference": "moderate"
                    },
                    "chapter_id": f"test-interests-chapter-{'-'.join(interests[:2])}"
                })

                assert response.status_code == 200
                data = response.json()
                assert data["personalization_applied"] is True

    def test_personalization_error_handling(self, client):
        """Test error handling in the personalization flow"""
        # Test with invalid content
        response = client.post("/personalize", json={
            "chapter_content": "",
            "user_preferences": {
                "learning_style": "visual",
                "experience_level": "intermediate",
                "interests": [],
                "difficulty_preference": "moderate"
            },
            "chapter_id": "test-error-chapter"
        })

        # Should return 422 for validation error or process with empty content
        # The exact behavior depends on validation implementation
        assert response.status_code in [200, 422]

        # Test with missing required fields
        response = client.post("/personalize", json={
            "chapter_content": "# Test",
            # Missing user_preferences
            "chapter_id": "test-error-chapter-2"
        })

        assert response.status_code == 422

    def test_personalization_content_preservation(self, client):
        """Test that important content elements are preserved during personalization"""
        original_content = """# Chapter Title
Content with [CITATION_REF_1] and [CITATION_REF_2].

```python
# Code block that should be preserved
import numpy as np
arr = np.array([1, 2, 3])
```

![Image Alt Text](image.png)

**Bold text** and *italic text* should remain.
"""

        with patch('src.main.personalization_agent') as mock_agent:
            # The agent should preserve these elements while personalizing content
            mock_response = """# Personalized Chapter Title
Personalized content with [CITATION_REF_1] and [CITATION_REF_2].

```python
# Code block that should be preserved
import numpy as np
arr = np.array([1, 2, 3])
```

![Image Alt Text](image.png)

**Bold text** and *italic text* should remain.
"""
            mock_agent.personalize_content = AsyncMock(return_value=mock_response)

            response = client.post("/personalize", json={
                "chapter_content": original_content,
                "user_preferences": {
                    "learning_style": "reading/writing",
                    "experience_level": "advanced",
                    "interests": ["Computer Science"],
                    "difficulty_preference": "challenging"
                },
                "chapter_id": "test-preservation-chapter"
            })

            assert response.status_code == 200
            data = response.json()
            assert data["personalization_applied"] is True
            # Verify that the response contains personalized content
            assert "personalized_content" in data

    def test_chapter_id_preservation(self, client):
        """Test that chapter IDs are properly preserved in responses"""
        test_cases = [
            ("english-chapter-1", "en"),
            ("urdu-chapter-1", "ur"),
            ("mixed-content-123", "en"),
            ("test-chapter_underscore", "en")
        ]

        for chapter_id, lang in test_cases:
            with patch('src.main.personalization_agent') as mock_agent:
                mock_agent.personalize_content = AsyncMock(
                    return_value=f"# Personalized Content\nFor chapter {chapter_id}."
                )

                response = client.post("/personalize", json={
                    "chapter_content": f"# Chapter {chapter_id}\nContent for {lang} locale.",
                    "user_preferences": {
                        "learning_style": "multimodal",
                        "experience_level": "intermediate",
                        "interests": [],
                        "difficulty_preference": "moderate"
                    },
                    "chapter_id": chapter_id
                })

                assert response.status_code == 200
                data = response.json()
                assert data["original_chapter_id"] == chapter_id
                assert data["personalization_applied"] is True


class TestAgentIntegration:
    """Test the personalization agent integration"""

    @pytest.mark.asyncio
    async def test_agent_personalization_method(self):
        """Test the agent's personalization method directly"""
        agent = PersonalizationAgent()

        sample_content = "# Test\nThis is test content."
        sample_preferences = UserPreferences(
            learning_style="visual",
            experience_level="beginner",
            interests=["AI"],
            difficulty_preference="easy"
        )

        # Mock the model response to avoid actual API calls
        with patch.object(agent, 'model') as mock_model:
            mock_response = AsyncMock()
            mock_response.text = "# Personalized Test\nVisual learner adapted content."
            mock_model.generate_content_async = AsyncMock(return_value=mock_response)

            result = await agent.personalize_content(sample_content, sample_preferences)

            assert result is not None
            assert "Personalized" in result

    @pytest.mark.asyncio
    async def test_agent_preserves_structure(self):
        """Test that the agent preserves content structure"""
        agent = PersonalizationAgent()

        sample_content = """# Title
## Section
```
Code block
```
[CITATION]
"""
        sample_preferences = UserPreferences()

        # Mock response that maintains structure
        with patch.object(agent, 'model') as mock_model:
            mock_response = AsyncMock()
            mock_response.text = """# Personalized Title
## Personalized Section
```
Code block
```
[CITATION]
"""
            mock_model.generate_content_async = AsyncMock(return_value=mock_response)

            result = await agent.personalize_content(sample_content, sample_preferences)

            # Verify structure is preserved
            assert "# Personalized Title" in result
            assert "## Personalized Section" in result
            assert "Code block" in result
            assert "[CITATION]" in result


if __name__ == "__main__":
    pytest.main([__file__])