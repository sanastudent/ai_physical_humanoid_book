"""
Backend API tests for AI-Driven Book + RAG Chatbot
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.main import app, qdrant_manager, embedding_generator, rag_engine
from src.schema import EmbedRequest, QueryRequest

# Create test client
client = TestClient(app)


def test_health_check():
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["service"] == "AI Book RAG API"


def test_embed_endpoint():
    """Test the embed endpoint"""
    test_data = {
        "content": "This is a test chapter content for embedding.",
        "chapter": "Test Chapter"
    }

    response = client.post("/embed", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert data["status"] == "success"
    assert data["chapter"] == "Test Chapter"
    assert data["chunks_created"] >= 0  # May be 0 if content is too short


def test_query_endpoint_global():
    """Test the query endpoint with global mode"""
    test_data = {
        "query": "What is this book about?",
        "mode": "global"
    }

    response = client.post("/query", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert "answer" in data
    assert "citations" in data
    assert "sources" in data


def test_query_endpoint_selected():
    """Test the query endpoint with selected mode"""
    test_data = {
        "query": "What does this text mean?",
        "mode": "selected",
        "context": "This is the selected text for testing purposes."
    }

    response = client.post("/query", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert "answer" in data
    assert "citations" in data
    assert "sources" in data


def test_select_endpoint():
    """Test the select endpoint"""
    test_data = {
        "query": "Explain this concept",
        "context": "This is sample context text for testing."
    }

    response = client.post("/select", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert "answer" in data
    assert "citations" in data
    assert "sources" in data


def test_select_endpoint_missing_context():
    """Test the select endpoint with missing context"""
    test_data = {
        "query": "Explain this concept"
        # Missing context
    }

    response = client.post("/select", json=test_data)
    assert response.status_code == 400


def test_embed_book_endpoint():
    """Test the embed-book endpoint"""
    test_data = {
        "Chapter 1": "This is the content of chapter 1",
        "Chapter 2": "This is the content of chapter 2"
    }

    response = client.post("/embed-book", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert data["status"] == "success"
    assert data["total_chunks"] >= 0
    assert "Chapter 1" in data["chapters"]
    assert "Chapter 2" in data["chapters"]


# Test agent functionality
def test_book_outline_agent():
    """Test BookOutlineAgent functionality"""
    from src.agents.book_outline_agent import BookOutlineAgent

    agent = BookOutlineAgent()

    # Test with a simple syllabus
    syllabus = "Introduction to AI: Basic concepts, machine learning fundamentals, neural networks"

    # We'll mock the LLM call to avoid actual API calls in tests
    with patch.object(agent.client.chat.completions, 'create') as mock_create:
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "title": "AI Introduction",
            "chapters": [
                {
                    "title": "Basic Concepts",
                    "subtopics": ["Definition", "History"],
                    "learning_outcomes": ["Understand basics", "Know history"],
                    "exercises": ["Exercise 1"]
                }
            ],
            "glossary_terms": ["AI", "ML"],
            "references": ["Book 1"]
        }
        '''
        mock_create.return_value = mock_response

        outline = asyncio.run(agent.generate_outline(syllabus))
        assert outline.title == "AI Introduction"
        assert len(outline.chapters) == 1


def test_chapter_writer_agent():
    """Test ChapterWriterAgent functionality"""
    from src.agents.chapter_writer_agent import ChapterWriterAgent, ChapterOutline

    agent = ChapterWriterAgent()

    # Create a simple chapter outline
    chapter_outline = ChapterOutline(
        title="Test Chapter",
        subtopics=["Topic 1", "Topic 2"],
        learning_outcomes=["Learn topic 1", "Learn topic 2"],
        exercises=["Exercise 1"]
    )

    # Mock the LLM call to avoid actual API calls in tests
    with patch.object(agent.client.chat.completions, 'create') as mock_create:
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "content": "This is the chapter content",
            "exercises": ["Exercise 1"],
            "image_placeholders": [{"id": "img1", "description": "test image"}],
            "citation_placeholders": [{"id": "cite1", "context": "test citation"}]
        }
        '''
        mock_create.return_value = mock_response

        chapter = asyncio.run(agent.generate_chapter(chapter_outline))
        assert chapter.title == "Test Chapter"
        assert chapter.content == "This is the chapter content"


if __name__ == "__main__":
    pytest.main([__file__])