"""
RAG (Retrieval Augmented Generation) logic using Google Gemini API
"""
import os
from typing import List, Dict, Optional, Any
import google.generativeai as genai
from dotenv import load_dotenv

from .qdrant_manager import QdrantManager
from .embed import EmbeddingGenerator

load_dotenv()


class RAGEngine:
    """Handles RAG queries and response generation using Google Gemini API"""

    def __init__(self):
        self.qdrant = QdrantManager()
        self.embedder = EmbeddingGenerator()

        # Initialize Gemini client for RAG responses
        self.gemini_key = os.getenv("GOOGLE_API_KEY")

        if not self.gemini_key:
            raise ValueError("GOOGLE_API_KEY not found in environment variables")

        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel('gemini-pro')  # Default model for RAG

    def retrieve_context(self, query: str, limit: int = 5) -> List[Dict]:
        """Retrieve relevant context from vector database

        Args:
            query: User query
            limit: Number of chunks to retrieve

        Returns:
            List of relevant chunks with scores
        """
        # Generate query embedding
        query_vector = self.embedder.generate_embedding(query)

        # Search Qdrant with error handling for database unavailability (Edge Case #106)
        try:
            results = self.qdrant.search(query_vector, limit=limit)
        except Exception as e:
            print(f"Qdrant database unavailable: {e}")
            # Return empty results but allow graceful degradation
            # The LLM will still generate an answer, just without retrieved context
            return []

        # Format results
        context_chunks = []
        for result in results:
            context_chunks.append({
                "text": result.payload["text"],
                "chapter": result.payload["chapter"],
                "score": result.score
            })

        return context_chunks

    def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict],
        mode: str = "global"
    ) -> Dict:
        """Generate answer using LLM with retrieved context

        Args:
            query: User query
            context_chunks: Retrieved context chunks
            mode: "global" or "selected"

        Returns:
            Dict with answer, citations, and sources
        """
        # Build context string
        context_str = "\n\n".join([
            f"[{chunk['chapter']}]\n{chunk['text']}"
            for chunk in context_chunks
        ])

        # Build prompt
        if mode == "selected":
            prompt = f"""You are an expert assistant answering questions about a book.
Use ONLY the provided context to answer the question. Do not use external knowledge.

Context:
{context_str}

Question: {query}

Provide a detailed answer with inline citations in the format [Chapter X: Paragraph Y] where X is the chapter number and Y is the paragraph number.
If the answer cannot be found in the context, say so."""
        else:
            prompt = f"""You are an expert assistant answering questions about a book on Embodied AI and Robotics.
Use the provided context from the book to answer the question accurately.

Context from the book:
{context_str}

Question: {query}

Provide a comprehensive answer with inline citations in the format [Chapter X: Paragraph Y] where X is the chapter number and Y is the paragraph number.
Use your knowledge to supplement, but prioritize information from the provided context."""

        # Generate response using Gemini API
        response = self.model.generate_content(
            prompt,
            generation_config=genai.GenerationConfig(
                max_output_tokens=1024,
                temperature=0.7
            )
        )
        answer = response.text

        # Extract citations
        citations = self._extract_citations(answer)

        # Validate citations for Edge Case #107
        citation_validation = self.validate_citations(citations, context_chunks)

        # Build sources list
        sources = [
            {
                "chapter": chunk["chapter"],
                "text": chunk["text"][:200] + "...",  # Truncate for brevity
                "score": chunk["score"]
            }
            for chunk in context_chunks
        ]

        return {
            "answer": answer,
            "citations": citations,
            "sources": sources,
            "citation_validation": citation_validation
        }

    def query_global(self, query: str) -> Dict:
        """Process global QA query

        Args:
            query: User question

        Returns:
            Response dict with answer and citations
        """
        context_chunks = self.retrieve_context(query, limit=5)

        # Add warning if no context was retrieved due to Qdrant issues
        if not context_chunks:
            print("Warning: No context retrieved. Qdrant database may be unavailable.")

        return self.generate_answer(query, context_chunks, mode="global")

    def query_selected(self, query: str, selected_text: str) -> Dict:
        """Process selected-text QA query

        Args:
            query: User question
            selected_text: Highlighted text from the book

        Returns:
            Response dict with answer and citations
        """
        # For selected text mode, we use the selected text as context
        # and still retrieve similar chunks to provide additional context
        context_chunks = self.retrieve_context(selected_text, limit=3)

        # Add warning if no additional context was retrieved due to Qdrant issues
        if not context_chunks or len(context_chunks) == 1:  # Only the selected text was added
            print("Warning: Limited context retrieved. Qdrant database may be unavailable.")

        # Add the selected text as the primary context
        context_chunks.insert(0, {
            "text": selected_text,
            "chapter": "Selected Text",
            "score": 1.0
        })

        return self.generate_answer(query, context_chunks, mode="selected")

    def _extract_citations(self, text: str) -> List[str]:
        """Extract citation references from generated text

        Args:
            text: Generated answer text

        Returns:
            List of citation strings
        """
        import re
        # Match patterns like [Chapter X: Paragraph Y] as specified in FR-014
        pattern = r'\[Chapter\s+\d+:\s*Paragraph\s+\d+\]'
        citations = re.findall(pattern, text)

        # Also capture any other citation formats that might be present
        all_matches = re.findall(r'\[([^\]]+)\]', text)
        # Filter to only include citations that match the required format
        formatted_citations = []
        for match in all_matches:
            full_match = f'[{match}]'
            if re.match(r'Chapter\s+\d+:\s*Paragraph\s+\d+', match, re.IGNORECASE):
                formatted_citations.append(full_match)

        # If no properly formatted citations found, return all matches as fallback
        if not formatted_citations:
            formatted_citations = [f'[{c}]' for c in all_matches]

        return list(set(formatted_citations))  # Remove duplicates

    def validate_citations(self, citations: List[str], context_chunks: List[Dict]) -> Dict[str, Any]:
        """
        Validate citations to ensure they correspond to actual content in context chunks.
        Implements citation validation when no clear citations exist (Edge Case #107).

        Args:
            citations: List of extracted citations
            context_chunks: List of context chunks that were used

        Returns:
            Dictionary with validation results
        """
        import re

        validation_results = {
            "total_citations": len(citations),
            "valid_citations": [],
            "invalid_citations": [],
            "has_valid_citations": len(citations) > 0,
            "all_citations_valid": True,
            "validation_details": []
        }

        for citation in citations:
            # Extract chapter and paragraph info from citation
            match = re.search(r'Chapter\s+(\d+)', citation)
            if match:
                chapter_num = match.group(1)
                # Check if this chapter exists in our context
                chapter_exists = any(
                    chunk.get('chapter', '').lower().startswith(f'chapter {chapter_num}') or
                    str(chapter_num) in chunk.get('chapter', '')
                    for chunk in context_chunks
                )

                if chapter_exists:
                    validation_results["valid_citations"].append(citation)
                    validation_results["validation_details"].append({
                        "citation": citation,
                        "valid": True,
                        "reason": f"Chapter {chapter_num} found in context"
                    })
                else:
                    validation_results["invalid_citations"].append(citation)
                    validation_results["all_citations_valid"] = False
                    validation_results["validation_details"].append({
                        "citation": citation,
                        "valid": False,
                        "reason": f"Chapter {chapter_num} not found in provided context"
                    })
            else:
                # If citation doesn't match expected format, mark as invalid
                validation_results["invalid_citations"].append(citation)
                validation_results["all_citations_valid"] = False
                validation_results["validation_details"].append({
                    "citation": citation,
                    "valid": False,
                    "reason": "Citation format does not match expected pattern"
                })

        # Special handling for Edge Case #107: when no clear citations exist
        if len(citations) == 0:
            validation_results["has_valid_citations"] = False
            validation_results["validation_details"].append({
                "citation": "None",
                "valid": False,
                "reason": "No citations were generated in the response"
            })
            # This is particularly important for selected-text mode where context is limited
            print("Warning: No citations generated. This may indicate limited source material or LLM not following citation format requirements.")

        return validation_results
