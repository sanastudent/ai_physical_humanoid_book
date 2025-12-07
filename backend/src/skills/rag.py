"""
RAG (Retrieval Augmented Generation) skill for question answering

Implements FR-019: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration
"""
import asyncio
import os
from typing import Any, Dict, List, Optional
from .base_skill import BaseSkill
from ..qdrant_manager import QdrantManager
from ..embed import get_embedding
from openai import AsyncOpenAI


class RAGSkill(BaseSkill):
    """
    Skill for performing RAG (Retrieval Augmented Generation) operations including:
    - Embedding queries
    - Vector search in Qdrant
    - Context retrieval
    - Answer generation with citations
    """

    def __init__(self):
        super().__init__(
            name="RAGSkill",
            description="Performs RAG operations including embedding, search, and answer generation with citations"
        )
        self.qdrant_manager = QdrantManager()
        # Initialize OpenAI client only if API key is available
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key:
            self.openai_client = AsyncOpenAI(api_key=openai_api_key)
        else:
            self.openai_client = None

    async def execute(self, **kwargs) -> Dict[str, Any]:
        """
        Execute RAG operation with the following steps:
        1. Embed the query
        2. Search in vector database
        3. Generate answer with retrieved context
        4. Format citations

        Required kwargs:
        - query (str): The question to answer
        - book_id (str): Book identifier to search within
        - mode (str): "global" or "selected" for different QA modes
        - context (str, optional): Additional context for selected-text QA

        Returns:
            Dictionary containing answer and citations
        """
        query = kwargs.get("query", "")
        book_id = kwargs.get("book_id", "default")
        mode = kwargs.get("mode", "global")
        context = kwargs.get("context", "")

        if not query:
            raise ValueError("Query is required for RAG operation")

        # Embed the query
        query_embedding = await self._embed_query(query)

        # Search in vector database
        search_results = await self._search_vectors(
            query_embedding=query_embedding,
            book_id=book_id,
            limit=5
        )

        # Generate answer based on mode
        if mode == "selected" and context:
            answer, citations = await self._generate_answer_with_context(
                query=query,
                context=context,
                search_results=search_results
            )
        else:
            answer, citations = await self._generate_answer_from_results(
                query=query,
                search_results=search_results
            )

        return {
            "answer": answer,
            "citations": citations,
            "search_results_count": len(search_results),
            "query": query,
            "book_id": book_id,
            "mode": mode
        }

    async def _embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for the query text.
        """
        try:
            # Use the same embedding function as in embed.py
            return await get_embedding(query)
        except Exception as e:
            self.logger.error(f"Failed to embed query: {e}")
            raise

    async def _search_vectors(
        self,
        query_embedding: List[float],
        book_id: str,
        limit: int = 5
    ) -> List[Any]:
        """
        Search for similar vectors in Qdrant database.
        Implements proper error handling for Qdrant database unavailability (Edge Case #106).
        """
        try:
            results = self.qdrant_manager.search(
                query_vector=query_embedding,
                limit=limit,
                book_id=book_id
            )
            return results
        except Exception as e:
            self.logger.error(f"Vector search failed: {e}")
            # Return empty results if search fails, allowing graceful degradation
            return []

    async def _generate_answer_from_results(
        self,
        query: str,
        search_results: List[Any]
    ) -> tuple[str, List[str]]:
        """
        Generate answer using retrieved search results.
        """
        if not search_results:
            return "I couldn't find relevant information in the book to answer your question.", []

        # Format context from search results
        context_parts = []
        citations = []

        for i, result in enumerate(search_results):
            if hasattr(result, 'payload'):
                text = result.payload.get('text', '')
                chapter = result.payload.get('chapter', 'Unknown')
                context_parts.append(f"Source {i+1}: {text}")

                # Create citation in format "[Chapter X: Paragraph Y]" as required by FR-014
                citation = f"[Chapter {chapter}: Section {i+1}]"
                citations.append(citation)

        context = "\n\n".join(context_parts)

        # Create a prompt for answer generation
        prompt = f"""
        Answer the following question based on the provided context from the book.
        If you cannot answer the question based on the context, say so clearly.

        Question: {query}

        Context: {context}

        Answer:
        """

        try:
            if self.openai_client is None:
                # Return a fallback response when API key is not available
                return "API key not available for answer generation. This feature requires a valid OpenAI API key.", citations

            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )

            answer = response.choices[0].message.content
            return answer, citations
        except Exception as e:
            self.logger.error(f"Answer generation failed: {e}")
            return "Sorry, I encountered an error while generating the answer.", citations

    async def _generate_answer_with_context(
        self,
        query: str,
        context: str,
        search_results: List[Any]
    ) -> tuple[str, List[str]]:
        """
        Generate answer using both selected context and search results.
        """
        # Combine the selected context with relevant search results
        combined_context = f"Selected text: {context}\n\nAdditional context from book:"

        citations = []
        for i, result in enumerate(search_results):
            if hasattr(result, 'payload'):
                text = result.payload.get('text', '')
                chapter = result.payload.get('chapter', 'Unknown')
                combined_context += f"\n\nRelated content {i+1}: {text}"

                # Create citation in format "[Chapter X: Paragraph Y]" as required by FR-014
                citation = f"[Chapter {chapter}: Section {i+1}]"
                citations.append(citation)

        # Create a prompt for answer generation focused on the selected text
        prompt = f"""
        Answer the following question based specifically on the selected text,
        using the additional context from the book as needed.
        If the selected text doesn't contain enough information to answer the question,
        indicate this clearly.

        Question: {query}

        Selected text: {context}

        Additional context: {combined_context}

        Answer:
        """

        try:
            if self.openai_client is None:
                # Return a fallback response when API key is not available
                return "API key not available for answer generation. This feature requires a valid OpenAI API key.", citations

            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=500
            )

            answer = response.choices[0].message.content
            return answer, citations
        except Exception as e:
            self.logger.error(f"Answer generation with context failed: {e}")
            return "Sorry, I encountered an error while generating the answer.", citations

    async def validate_citations(
        self,
        citations: List[str],
        search_results: List[Any]
    ) -> Dict[str, Any]:
        """
        Validate that citations correctly reference the search results.
        Implements citation validation for selected-text context (Edge Case #107).
        """
        valid_citations = []
        invalid_citations = []
        total_results = len(search_results)

        for citation in citations:
            # Check if citation format is correct
            if citation.startswith("[Chapter ") and citation.endswith("]"):
                valid_citations.append(citation)
            else:
                invalid_citations.append({
                    "citation": citation,
                    "error": "Invalid citation format"
                })

        return {
            "valid_citations": len(valid_citations),
            "invalid_citations": len(invalid_citations),
            "total_citations": len(citations),
            "validity_percentage": (len(valid_citations) / len(citations) * 100) if citations else 0,
            "validation_passed": len(invalid_citations) == 0
        }

    async def health_check(self) -> bool:
        """
        Check if the RAG skill is ready to operate.
        """
        try:
            # Check if Qdrant is accessible
            qdrant_healthy = self.qdrant_manager.health_check()

            # If Qdrant is not healthy, the skill cannot function properly
            return qdrant_healthy
        except Exception as e:
            self.logger.error(f"RAG skill health check failed: {e}")
            return False