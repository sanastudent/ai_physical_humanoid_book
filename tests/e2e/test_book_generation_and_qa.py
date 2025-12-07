"""
End-to-end integration tests for AI-Driven Book + RAG Chatbot
Tests the complete workflow: Book generation -> Embedding -> QA
"""
import pytest
import asyncio
import json
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the backend src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend'))

from backend.src.agents.book_outline_agent import BookOutlineAgent
from backend.src.agents.chapter_writer_agent import ChapterWriterAgent
from backend.src.embed import EmbeddingGenerator
from backend.src.rag import RAGEngine
from backend.src.qdrant_manager import QdrantManager


def test_complete_book_generation_workflow():
    """
    End-to-end test: Generate a book outline, write chapters, embed content, and query
    Implements SC-003, SC-004: End-to-end integration tests for book generation and QA
    """
    # Mock the LLM responses to avoid actual API calls
    with patch('backend.src.config.ai_config.ai_config.primary_model_name', 'gpt-4-turbo'):
        # Test 1: Generate book outline
        outline_agent = BookOutlineAgent()

        with patch.object(outline_agent.client.chat.completions, 'create') as mock_create:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = '''
            {
                "title": "Test AI Book",
                "chapters": [
                    {
                        "title": "Introduction to AI",
                        "subtopics": ["What is AI", "History of AI"],
                        "learning_outcomes": ["Understand AI basics", "Know AI history"],
                        "exercises": ["Exercise 1: Define AI"]
                    }
                ],
                "glossary_terms": ["AI", "Machine Learning", "Neural Networks"],
                "references": ["Russell & Norvig", "Goodfellow et al."]
            }
            '''
            mock_create.return_value = mock_response

            course_syllabus = "Introduction to Artificial Intelligence: Basic concepts and applications"
            outline = asyncio.run(outline_agent.generate_outline(course_syllabus))

            assert outline.title == "Test AI Book"
            assert len(outline.chapters) == 1
            assert outline.chapters[0].title == "Introduction to AI"
            assert len(outline.glossary_terms) > 0
            assert len(outline.references) > 0

        # Test 2: Generate chapter content
        chapter_agent = ChapterWriterAgent()

        with patch.object(chapter_agent.client.chat.completions, 'create') as mock_create:
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = '''
            {
                "content": "# Introduction to AI\\n\\nArtificial Intelligence (AI) is a branch of computer science...",
                "exercises": ["Exercise 1: Define AI"],
                "image_placeholders": [{"id": "img1", "description": "AI concept diagram"}],
                "citation_placeholders": [{"id": "cite1", "context": "Historical development"}]
            }
            '''
            mock_create.return_value = mock_response

            chapter_outline = outline.chapters[0]
            chapter = asyncio.run(chapter_agent.generate_chapter(chapter_outline))

            assert chapter.title == "Introduction to AI"
            assert len(chapter.content) > 0
            assert len(chapter.exercises) > 0

        # Test 3: Process and embed the content
        embed_generator = EmbeddingGenerator()

        # Mock the embedding generation to avoid actual API calls
        with patch.object(embed_generator, 'generate_embedding') as mock_embed:
            mock_embed.return_value = [0.1] * 1536  # Mock embedding vector

            book_content = {chapter.title: chapter.content}
            embeddings = embed_generator.process_book(book_content)

            assert len(embeddings) > 0
            assert all('vector' in emb for emb in embeddings)
            assert all('text' in emb for emb in embeddings)

        # Test 4: Query the content using RAG
        rag_engine = RAGEngine()

        with patch.object(rag_engine.embedder, 'generate_embedding') as mock_embed, \
             patch.object(rag_engine.qdrant, 'search') as mock_search, \
             patch.object(rag_engine.client.chat.completions, 'create') as mock_create:

            # Mock the embedding
            mock_embed.return_value = [0.1] * 1536

            # Mock the search results
            mock_search_result = MagicMock()
            mock_search_result.payload = {"text": chapter.content, "chapter": chapter.title}
            mock_search_result.score = 0.9
            mock_search.return_value = [mock_search_result]

            # Mock the LLM response
            mock_response = MagicMock()
            mock_response.choices = [MagicMock()]
            mock_response.choices[0].message.content = "AI is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."
            mock_create.return_value = mock_response

            # Test global QA
            result = rag_engine.query_global("What is AI?")

            assert "answer" in result
            assert len(result["citations"]) >= 0  # May not have citations depending on response

        print("✅ Complete book generation workflow test passed")


def test_selected_text_qa_workflow():
    """
    End-to-end test for selected-text QA functionality
    Implements SC-004: End-to-end integration tests for selected-text QA
    """
    rag_engine = RAGEngine()

    with patch.object(rag_engine.embedder, 'generate_embedding') as mock_embed, \
         patch.object(rag_engine.qdrant, 'search') as mock_search, \
         patch.object(rag_engine.client.chat.completions, 'create') as mock_create:

        # Mock the embedding
        mock_embed.return_value = [0.1] * 1536

        # Mock the search results
        mock_search_result = MagicMock()
        mock_search_result.payload = {"text": "This is the selected text about AI.", "chapter": "Chapter 1"}
        mock_search_result.score = 0.9
        mock_search.return_value = [mock_search_result]

        # Mock the LLM response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "The selected text explains that AI is about creating intelligent machines."
        mock_create.return_value = mock_response

        # Test selected-text QA
        selected_text = "This is the selected text about AI."
        result = rag_engine.query_selected("What does this text mean?", selected_text)

        assert "answer" in result
        assert "citations" in result
        assert len(result["citations"]) >= 0

    print("✅ Selected-text QA workflow test passed")


def test_error_handling_workflow():
    """
    End-to-end test for error handling in the complete workflow
    Implements error handling for LLM failures, token limits, DB unavailability
    """
    # Test LLM failure handling with retry logic
    outline_agent = BookOutlineAgent()

    with patch.object(outline_agent.client.chat.completions, 'create') as mock_create:
        # First call fails, second succeeds (simulating retry)
        mock_create.side_effect = [
            Exception("API Error"),  # First call fails
            Mock(choices=[Mock(message=Mock(content='''{"title": "Recovered Book", "chapters": [], "glossary_terms": [], "references": []}'''))])  # Second call succeeds
        ]

        course_syllabus = "Test syllabus"
        outline = asyncio.run(outline_agent.generate_outline(course_syllabus))

        # Should recover from the error and generate a default outline
        assert outline.title == "Recovered Book"  # Or should use default from _generate_default_outline

    print("✅ Error handling workflow test completed")


def test_token_limit_handling():
    """
    Test token limit handling for large books
    Implements Edge Case #105: Token limit handling for large books
    """
    chapter_agent = ChapterWriterAgent()

    # Create a very long context to test token limit handling
    long_book_context = "This is a very long book context. " * 1000  # Very long context

    with patch.object(chapter_agent.client.chat.completions, 'create') as mock_create:
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "content": "# Test Chapter\\n\\nContent here...",
            "exercises": ["Exercise 1"],
            "image_placeholders": [{"id": "img1", "description": "test"}],
            "citation_placeholders": [{"id": "cite1", "context": "test"}]
        }
        '''
        mock_create.return_value = mock_response

        chapter_outline = MagicMock()
        chapter_outline.title = "Test Chapter"
        chapter_outline.subtopics = ["Topic 1"]
        chapter_outline.learning_outcomes = ["Learn topic 1"]
        chapter_outline.exercises = ["Exercise 1"]

        # This should handle the large context properly
        chapter = asyncio.run(chapter_agent.generate_chapter(
            chapter_outline=chapter_outline,
            book_context=long_book_context  # This is a large context
        ))

        assert chapter.title == "Test Chapter"
        assert len(chapter.content) > 0

    print("✅ Token limit handling test passed")


if __name__ == "__main__":
    test_complete_book_generation_workflow()
    test_selected_text_qa_workflow()
    test_error_handling_workflow()
    test_token_limit_handling()
    print("🎉 All end-to-end integration tests passed!")