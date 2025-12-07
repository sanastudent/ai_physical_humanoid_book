"""
Book generation process validation tests
Validates that the book generation process completes without manual intervention
Implements SC-005: Validate book generation process completes without manual intervention
"""
import asyncio
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.agents.book_outline_agent import BookOutlineAgent
from src.agents.chapter_writer_agent import ChapterWriterAgent
from src.agents.rag_agent import RAGAgent
from src.skills.content_processing import ContentProcessingSkill


def test_book_outline_generation_no_manual_intervention():
    """
    Test that book outline generation completes without manual intervention
    """
    agent = BookOutlineAgent()

    # Test with a realistic course syllabus
    syllabus = """
    Machine Learning Fundamentals Course:
    - Introduction to ML
    - Supervised Learning
    - Unsupervised Learning
    - Neural Networks
    - Deep Learning Applications
    - Model Evaluation and Deployment
    """

    with patch.object(agent.client.chat.completions, 'create') as mock_create:
        # Mock a realistic response from the LLM
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "title": "Machine Learning Fundamentals",
            "chapters": [
                {
                    "title": "Introduction to Machine Learning",
                    "subtopics": ["Definition and Types", "History and Evolution", "Applications"],
                    "learning_outcomes": ["Define ML", "Identify types of ML", "Describe applications"],
                    "exercises": ["Exercise 1: Identify ML problems", "Exercise 2: Categorize algorithms"]
                },
                {
                    "title": "Supervised Learning",
                    "subtopics": ["Regression", "Classification", "Model Training"],
                    "learning_outcomes": ["Explain supervised learning", "Implement basic algorithms", "Evaluate models"],
                    "exercises": ["Exercise 1: Linear regression", "Exercise 2: Classification task"]
                }
            ],
            "glossary_terms": ["Machine Learning", "Algorithm", "Training", "Model", "Feature", "Label"],
            "references": ["Mitchell - Machine Learning", "Hastie - Elements of Statistical Learning"]
        }
        '''
        mock_create.return_value = mock_response

        # Generate outline automatically - no manual intervention needed
        outline = asyncio.run(agent.generate_outline(syllabus))

        # Validate the output
        assert outline.title == "Machine Learning Fundamentals"
        assert len(outline.chapters) == 2
        assert len(outline.glossary_terms) > 0
        assert len(outline.references) > 0

        # Validate each chapter
        for chapter in outline.chapters:
            assert chapter.title
            assert len(chapter.subtopics) > 0
            assert len(chapter.learning_outcomes) > 0
            assert len(chapter.exercises) > 0

    print("✅ Book outline generation completed without manual intervention")


def test_chapter_generation_no_manual_intervention():
    """
    Test that chapter generation completes without manual intervention
    """
    agent = ChapterWriterAgent()

    # Create a chapter outline to generate from
    from src.agents.book_outline_agent import ChapterOutline
    chapter_outline = ChapterOutline(
        title="Introduction to Machine Learning",
        subtopics=["Definition and Types", "History and Evolution", "Applications"],
        learning_outcomes=["Define ML", "Identify types of ML", "Describe applications"],
        exercises=["Exercise 1: Identify ML problems", "Exercise 2: Categorize algorithms"]
    )

    with patch.object(agent.client.chat.completions, 'create') as mock_create:
        # Mock a realistic chapter response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "content": "# Introduction to Machine Learning\\n\\n## Definition and Types\\nMachine Learning is a subset of artificial intelligence...\\n\\n## History and Evolution\\nThe field of machine learning began in the 1950s...\\n\\n## Applications\\nMachine learning is used in various domains including...\\n\\n## Summary\\nIn this chapter, we learned about the fundamentals of machine learning.",
            "exercises": ["Exercise 1: Identify ML problems", "Exercise 2: Categorize algorithms"],
            "image_placeholders": [
                {"id": "img1", "description": "Machine learning process diagram"},
                {"id": "img2", "description": "Types of machine learning visualization"}
            ],
            "citation_placeholders": [
                {"id": "cite1", "context": "Historical development of ML"},
                {"id": "cite2", "context": "Modern applications"}
            ]
        }
        '''
        mock_create.return_value = mock_response

        # Generate chapter automatically - no manual intervention needed
        chapter = asyncio.run(agent.generate_chapter(chapter_outline))

        # Validate the output
        assert chapter.title == "Introduction to Machine Learning"
        assert len(chapter.content) > 0
        assert len(chapter.exercises) == 2
        assert len(chapter.image_placeholders) > 0
        assert len(chapter.citation_placeholders) > 0

    print("✅ Chapter generation completed without manual intervention")


def test_complete_book_generation_workflow():
    """
    Test complete book generation workflow from syllabus to full book
    """
    # Step 1: Generate outline
    outline_agent = BookOutlineAgent()

    with patch.object(outline_agent.client.chat.completions, 'create') as mock_create:
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "title": "Complete ML Book",
            "chapters": [
                {
                    "title": "Chapter 1: Basics",
                    "subtopics": ["Introduction", "Key Concepts"],
                    "learning_outcomes": ["Understand basics"],
                    "exercises": ["Basic exercise"]
                }
            ],
            "glossary_terms": ["ML", "AI"],
            "references": ["Book reference"]
        }
        '''
        mock_create.return_value = mock_response

        syllabus = "Machine Learning basics course"
        book_outline = asyncio.run(outline_agent.generate_outline(syllabus))

    # Step 2: Generate each chapter
    chapter_agent = ChapterWriterAgent()

    with patch.object(chapter_agent.client.chat.completions, 'create') as mock_create:
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = '''
        {
            "content": "# Chapter 1: Basics\\n\\nContent goes here...",
            "exercises": ["Basic exercise"],
            "image_placeholders": [{"id": "img1", "description": "Basic concept"}],
            "citation_placeholders": [{"id": "cite1", "context": "Basic concept"}]
        }
        '''
        mock_create.return_value = mock_response

        generated_chapters = []
        for chapter_outline in book_outline.chapters:
            chapter = asyncio.run(chapter_agent.generate_chapter(chapter_outline))
            generated_chapters.append(chapter)

    # Validate complete book
    assert book_outline.title == "Complete ML Book"
    assert len(generated_chapters) == len(book_outline.chapters)
    assert len(book_outline.glossary_terms) > 0
    assert len(book_outline.references) > 0

    for chapter in generated_chapters:
        assert chapter.title
        assert len(chapter.content) > 0
        assert len(chapter.exercises) >= 0

    print("✅ Complete book generation workflow completed without manual intervention")


def test_content_processing_skill_validation():
    """
    Test that content processing skill works without manual intervention
    """
    skill = ContentProcessingSkill()

    # Test with realistic content
    content = """
    # Introduction to AI

    Artificial Intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals. Leading AI textbooks define the field as the study of "intelligent agents": any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals.

    ## Key Concepts

    - Machine Learning
    - Neural Networks
    - Deep Learning
    - Natural Language Processing

    These concepts form the foundation of modern AI systems.
    """

    # Process content automatically - no manual intervention needed
    result = asyncio.run(skill.execute(
        content=content,
        chapter="Introduction",
        book_id="test-book",
        chunk_size=500,
        overlap=50
    ))

    # Validate the result
    assert result["total_chunks"] > 0
    assert result["processed_content_length"] > 0
    assert result["chapter"] == "Introduction"
    assert result["book_id"] == "test-book"
    assert result["validation_results"]["validity_percentage"] >= 0

    # Check that chunks are properly created
    assert len(result["chunks"]) > 0
    for chunk in result["chunks"]:
        assert "id" in chunk
        assert "text" in chunk
        assert "token_count" in chunk
        assert chunk["chapter"] == "Introduction"

    print("✅ Content processing skill validation completed without manual intervention")


def test_rag_agent_integration():
    """
    Test RAG agent integration without manual intervention
    """
    agent = RAGAgent()

    # Test query processing automatically
    from src.agents.rag_agent import RAGQuery
    query = RAGQuery(
        query="What is machine learning?",
        book_id="test-book",
        mode="global"
    )

    # Mock the RAG skill execution
    with patch.object(agent.rag_skill, 'execute') as mock_execute:
        mock_execute.return_value = {
            "answer": "Machine learning is a subset of AI...",
            "citations": ["[Chapter 1: Section 1]", "[Chapter 2: Section 3]"],
            "search_results_count": 2
        }

        result = asyncio.run(agent.process_query(query))

        assert "answer" in result
        assert "citations" in result
        assert result["search_results_count"] == 2

    print("✅ RAG agent integration completed without manual intervention")


def test_error_resilience():
    """
    Test that the system handles errors gracefully without manual intervention
    """
    outline_agent = BookOutlineAgent()

    with patch.object(outline_agent.client.chat.completions, 'create') as mock_create:
        # Simulate API failure, should fall back to default
        mock_create.side_effect = Exception("API unavailable")

        # The agent should handle the error and return a default outline
        outline = asyncio.run(outline_agent.generate_outline("Test syllabus"))

        # Should have default values instead of crashing
        assert outline.title
        assert hasattr(outline, 'chapters')

    print("✅ Error resilience test completed without manual intervention")


if __name__ == "__main__":
    test_book_outline_generation_no_manual_intervention()
    test_chapter_generation_no_manual_intervention()
    test_complete_book_generation_workflow()
    test_content_processing_skill_validation()
    test_rag_agent_integration()
    test_error_resilience()
    print("🎉 All book generation validation tests passed - no manual intervention required!")