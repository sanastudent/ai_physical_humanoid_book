"""
RAG Agent for handling Retrieval Augmented Generation queries

Implements FR-010: The system MUST support two QA modes:
- Global QA: Questions about the entire book content
- Selected-text QA: Questions limited to highlighted/selected text
"""
import asyncio
import os
import json
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
import google.generativeai as genai
from dotenv import load_dotenv

from ..config.ai_config import ai_config
from ..skills.rag import RAGSkill
from ..qdrant_manager import QdrantManager
from ..embed import EmbeddingGenerator

load_dotenv()

class RAGQuery(BaseModel):
    """Represents a RAG query with context and mode"""
    query: str
    book_id: str = "default"
    mode: str = "global"  # "global" or "selected"
    context: Optional[str] = None  # For selected-text mode


class RAGResponse(BaseModel):
    """Represents a RAG response with answer and citations"""
    answer: str
    citations: List[str]
    sources: List[Dict[str, Any]]
    search_results_count: int


class RAGAgent:
    """
    Agent responsible for handling RAG queries using the RAG skill.

    Implements FR-010: The system MUST support two QA modes:
    - Global QA: Questions about the entire book content
    - Selected-text QA: Questions limited to highlighted/selected text
    """

    def __init__(self):
        self.rag_skill = RAGSkill()
        self.qdrant_manager = QdrantManager()
        self.embedding_generator = EmbeddingGenerator()
        self.gemini_key = ai_config.primary_api_key or os.getenv("GOOGLE_API_KEY")
        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel(ai_config.primary_model_name or 'gemini-pro')

    async def query_global(self, query: str, book_id: str = "default") -> RAGResponse:
        """
        Handle global QA queries (questions about the entire book).

        Args:
            query: The user's question
            book_id: Identifier for the book to search in

        Returns:
            RAGResponse with answer and citations
        """
        try:
            # Use the RAG skill for global QA
            result = await self.rag_skill.execute(
                query=query,
                book_id=book_id,
                mode="global"
            )

            return RAGResponse(
                answer=result.get("answer", ""),
                citations=result.get("citations", []),
                sources=[],  # Sources can be added if needed
                search_results_count=result.get("search_results_count", 0)
            )
        except Exception as e:
            print(f"Error in global QA: {e}")
            return RAGResponse(
                answer="Sorry, I encountered an error processing your question.",
                citations=[],
                sources=[],
                search_results_count=0
            )

    async def query_selected_text(self, query: str, selected_text: str, book_id: str = "default") -> RAGResponse:
        """
        Handle selected-text QA queries (questions limited to highlighted text).

        Args:
            query: The user's question
            selected_text: The highlighted/selected text context
            book_id: Identifier for the book to search in

        Returns:
            RAGResponse with answer and citations
        """
        try:
            # Use the RAG skill for selected-text QA
            result = await self.rag_skill.execute(
                query=query,
                book_id=book_id,
                mode="selected",
                context=selected_text
            )

            return RAGResponse(
                answer=result.get("answer", ""),
                citations=result.get("citations", []),
                sources=[],  # Sources can be added if needed
                search_results_count=result.get("search_results_count", 0)
            )
        except Exception as e:
            print(f"Error in selected-text QA: {e}")
            return RAGResponse(
                answer="Sorry, I encountered an error processing your question.",
                citations=[],
                sources=[],
                search_results_count=0
            )

    async def process_query(self, rag_query: RAGQuery) -> RAGResponse:
        """
        Process a RAG query based on its mode.

        Args:
            rag_query: The query with mode and context

        Returns:
            RAGResponse with answer and citations
        """
        if rag_query.mode == "selected" and rag_query.context:
            return await self.query_selected_text(
                query=rag_query.query,
                selected_text=rag_query.context,
                book_id=rag_query.book_id
            )
        else:
            return await self.query_global(
                query=rag_query.query,
                book_id=rag_query.book_id
            )

    async def validate_response(self, response: RAGResponse) -> Dict[str, Any]:
        """
        Validate the RAG response for quality and completeness.

        Args:
            response: The RAG response to validate

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "has_answer": bool(response.answer and response.answer.strip()),
            "has_citations": len(response.citations) > 0,
            "citation_format_valid": all(
                "[Chapter" in citation and ":" in citation and "]" in citation
                for citation in response.citations
            ),
            "answer_length": len(response.answer),
            "citation_count": len(response.citations),
            "search_results_count": response.search_results_count
        }

        validation_results["is_valid"] = (
            validation_results["has_answer"] and
            validation_results["has_citations"]
        )

        return validation_results

    async def generate_followup_questions(self, query: str, answer: str, book_id: str = "default") -> List[str]:
        """
        Generate potential follow-up questions based on the query and answer.

        Args:
            query: The original query
            answer: The answer provided
            book_id: Identifier for the book

        Returns:
            List of potential follow-up questions
        """
        prompt = f"""
        Based on this question and answer about a book, generate 3-5 relevant follow-up questions:

        Original question: {query}
        Answer: {answer}

        Return only the follow-up questions as a JSON array.
        """

        try:
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.7,
                    max_output_tokens=200
                )
            )

            result = json.loads(response.text)
            return result if isinstance(result, list) else []
        except Exception as e:
            print(f"Error generating follow-up questions: {e}")
            return [f"How does this relate to {query.split()[0] if query.split() else 'the topic'}?"]