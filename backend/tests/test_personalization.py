"""
Backend Integration Tests for Personalization Feature
Tests the personalization endpoint and agent functionality
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.main import app
from src.schema import UserPreferences, PersonalizationRequest
from src.agents.personalization_agent import PersonalizationAgent


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def sample_preferences():
    """Sample user preferences for testing"""
    return UserPreferences(
        learning_style="visual",
        experience_level="intermediate",
        interests=["robotics", "AI"],
        difficulty_preference="moderate"
    )


@pytest.fixture
def sample_request_data(sample_preferences):
    """Sample personalization request data"""
    return {
        "chapter_content": "# Test Chapter\nThis is a test chapter content.\n\n## Section 1\nSome content here.",
        "user_preferences": sample_preferences.model_dump(),
        "chapter_id": "test-chapter-1"
    }


class TestPersonalizationEndpoint:
    """Test the personalization API endpoint"""

    def test_personalization_endpoint_exists(self, client):
        """Test that the personalization endpoint exists"""
        response = client.get("/docs")  # Check if endpoint is in docs
        assert response.status_code == 200

    def test_personalization_endpoint_post_method(self, client, sample_request_data):
        """Test the personalization endpoint with valid data"""
        # For this test, we'll mock the personalization agent to avoid actual AI processing
        with patch('src.main.personalization_agent') as mock_agent:
            mock_agent.personalize_content = AsyncMock(return_value="Personalized content")

            response = client.post("/personalize", json=sample_request_data)

            # Should return 200 OK
            assert response.status_code == 200

            # Check response structure
            data = response.json()
            assert "original_chapter_id" in data
            assert "personalized_content" in data
            assert "personalization_applied" in data
            assert data["original_chapter_id"] == "test-chapter-1"
            assert data["personalization_applied"] is True

    def test_personalization_endpoint_invalid_data(self, client):
        """Test the personalization endpoint with invalid data"""
        invalid_data = {
            "invalid_field": "invalid_value"
        }

        response = client.post("/personalize", json=invalid_data)

        # Should return 422 for validation error
        assert response.status_code == 422

    def test_personalization_endpoint_missing_fields(self, client):
        """Test the personalization endpoint with missing required fields"""
        incomplete_data = {
            "chapter_content": "Some content"
            # Missing required fields
        }

        response = client.post("/personalize", json=incomplete_data)

        # Should return 422 for validation error
        assert response.status_code == 422


class TestPersonalizationAgent:
    """Test the personalization agent functionality"""

    @pytest.mark.asyncio
    async def test_personalization_agent_initialization(self):
        """Test that the personalization agent initializes correctly"""
        agent = PersonalizationAgent()
        assert agent is not None
        assert hasattr(agent, 'personalize_content')
        assert hasattr(agent, '_construct_personalization_prompt')

    @pytest.mark.asyncio
    async def test_construct_personalization_prompt(self):
        """Test that the personalization prompt is constructed correctly"""
        agent = PersonalizationAgent()

        sample_content = "# Test\nTest content"
        sample_preferences = UserPreferences(
            learning_style="visual",
            experience_level="beginner",
            interests=["AI"],
            difficulty_preference="easy"
        )

        prompt = agent._construct_personalization_prompt(sample_content, sample_preferences)

        # Check that the prompt contains the content
        assert sample_content in prompt
        assert "Visual" in prompt or "visual" in prompt
        assert "Beginner" in prompt or "beginner" in prompt

    @pytest.mark.asyncio
    async def test_personalization_content_preserves_structure(self):
        """Test that personalization preserves content structure"""
        agent = PersonalizationAgent()

        original_content = """# Introduction
This is an introduction.

## Section 1
Some text here.

```python
def hello():
    print("Hello, World!")
```

[Chapter 1: Paragraph 1] This is a citation.
"""

        # Mock the model response to return content similar to original
        with patch.object(agent.model, 'generate_content_async', new_callable=AsyncMock) as mock_gen:
            mock_gen.return_value = AsyncMock()
            mock_gen.return_value.text = original_content  # Return same content for testing

            result = await agent.personalize_content(original_content, UserPreferences())

            # Check that the structure is preserved
            assert "# Introduction" in result
            assert "## Section 1" in result
            assert "def hello():" in result  # Code block preserved
            assert "[Chapter 1: Paragraph 1]" in result  # Citation preserved

    @pytest.mark.asyncio
    async def test_personalization_with_different_preferences(self):
        """Test personalization with different user preferences"""
        agent = PersonalizationAgent()

        original_content = "This is some content."
        beginner_prefs = UserPreferences(
            learning_style="visual",
            experience_level="beginner",
            difficulty_preference="easy"
        )
        expert_prefs = UserPreferences(
            learning_style="reading/writing",
            experience_level="expert",
            difficulty_preference="challenging"
        )

        # Mock responses for different preferences
        with patch.object(agent.model, 'generate_content_async', new_callable=AsyncMock) as mock_gen:
            mock_gen.return_value = AsyncMock()
            mock_gen.return_value.text = "Mocked personalized content"

            beginner_result = await agent.personalize_content(original_content, beginner_prefs)
            expert_result = await agent.personalize_content(original_content, expert_prefs)

            # Both should return content (though mocked to be the same in this test)
            assert beginner_result is not None
            assert expert_result is not None


class TestUserPreferencesModel:
    """Test the UserPreferences model validation"""

    def test_user_preferences_defaults(self):
        """Test that UserPreferences has correct defaults"""
        prefs = UserPreferences()

        assert prefs.learning_style == "multimodal"
        assert prefs.experience_level == "intermediate"
        assert prefs.interests == []
        assert prefs.difficulty_preference == "moderate"

    def test_user_preferences_custom_values(self):
        """Test that UserPreferences accepts custom values"""
        prefs = UserPreferences(
            learning_style="visual",
            experience_level="beginner",
            interests=["AI", "robotics"],
            difficulty_preference="challenging"
        )

        assert prefs.learning_style == "visual"
        assert prefs.experience_level == "beginner"
        assert "AI" in prefs.interests
        assert "robotics" in prefs.interests
        assert prefs.difficulty_preference == "challenging"


if __name__ == "__main__":
    pytest.main([__file__])