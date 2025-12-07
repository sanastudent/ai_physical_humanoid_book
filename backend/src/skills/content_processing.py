"""
Content processing skill for book generation and processing

Implements FR-019: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration
"""
import asyncio
from typing import Any, Dict, List, Optional
from .base_skill import BaseSkill
from ..schema import BookChunk


class ContentProcessingSkill(BaseSkill):
    """
    Skill for processing book content including:
    - Text chunking according to FR-009 (500 tokens, 50 overlap)
    - Content validation
    - Format conversion
    """

    def __init__(self):
        super().__init__(
            name="ContentProcessingSkill",
            description="Processes book content including chunking, validation, and format conversion"
        )

    async def execute(self, **kwargs) -> Dict[str, Any]:
        """
        Execute content processing with the following operations:

        Required kwargs:
        - content (str): The content to process
        - chapter (str): Chapter identifier
        - book_id (str): Book identifier
        - chunk_size (int): Size of chunks in tokens (default 500 per FR-009)
        - overlap (int): Overlap between chunks in tokens (default 50 per FR-009)

        Returns:
            Dictionary containing processed chunks and metadata
        """
        content = kwargs.get("content", "")
        chapter = kwargs.get("chapter", "unknown")
        book_id = kwargs.get("book_id", "default")
        chunk_size = kwargs.get("chunk_size", 500)  # FR-009: chunk size 500 tokens
        overlap = kwargs.get("overlap", 50)  # FR-009: overlap 50 tokens

        if not content:
            raise ValueError("Content is required for processing")

        # Process the content into chunks
        chunks = await self._chunk_content(
            content=content,
            chapter=chapter,
            book_id=book_id,
            chunk_size=chunk_size,
            overlap=overlap
        )

        # Validate the chunks
        validation_results = await self._validate_chunks(chunks)

        return {
            "chunks": chunks,
            "total_chunks": len(chunks),
            "validation_results": validation_results,
            "processed_content_length": len(content),
            "chapter": chapter,
            "book_id": book_id
        }

    async def _chunk_content(
        self,
        content: str,
        chapter: str,
        book_id: str,
        chunk_size: int,
        overlap: int
    ) -> List[Dict[str, Any]]:
        """
        Split content into chunks with specified size and overlap.
        This implements FR-009: content processing rules (chunk size: 500 tokens, overlap: 50 tokens).
        """
        # For now, we'll implement a simple character-based chunking
        # In a real implementation, this would use token counting
        chunks = []
        start = 0

        while start < len(content):
            # Calculate end position
            end = start + chunk_size

            # If we're near the end, just take the rest
            if end >= len(content):
                end = len(content)
            else:
                # Try to break at a sentence or paragraph boundary
                # Find the last sentence break before the chunk_size limit
                chunk_content = content[start:end]
                last_period = chunk_content.rfind('.')
                last_space = chunk_content.rfind(' ')

                # Choose the break point closest to the end but before chunk_size
                break_point = max(last_period, last_space)
                if break_point > chunk_size // 2:  # Only break if we're not cutting too early
                    end = start + break_point + 1

            # Create the chunk
            chunk_text = content[start:end]
            chunk_id = f"{book_id}_{chapter}_chunk_{len(chunks)}"

            chunk_obj = {
                "id": chunk_id,
                "chapter": chapter,
                "text": chunk_text,
                "token_count": len(chunk_text.split()),  # Simple token count (in real impl, use tiktoken)
                "book_id": book_id,
                "position": len(chunks)
            }

            chunks.append(chunk_obj)

            # Move start position forward by chunk_size - overlap
            start = end - overlap if end < len(content) else len(content)

        return chunks

    async def _validate_chunks(self, chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate the generated chunks for quality and consistency.
        """
        total_chunks = len(chunks)
        valid_chunks = 0
        invalid_chunks = []

        for i, chunk in enumerate(chunks):
            # Basic validation checks
            is_valid = True
            errors = []

            if not chunk.get("text") or len(chunk["text"].strip()) == 0:
                is_valid = False
                errors.append("Empty chunk text")

            if not chunk.get("id"):
                is_valid = False
                errors.append("Missing chunk ID")

            if not chunk.get("chapter"):
                is_valid = False
                errors.append("Missing chapter reference")

            if chunk.get("token_count", 0) <= 0:
                is_valid = False
                errors.append("Invalid token count")

            if is_valid:
                valid_chunks += 1
            else:
                invalid_chunks.append({
                    "index": i,
                    "chunk_id": chunk.get("id"),
                    "errors": errors
                })

        return {
            "total_chunks": total_chunks,
            "valid_chunks": valid_chunks,
            "invalid_chunks": invalid_chunks,
            "validity_percentage": (valid_chunks / total_chunks * 100) if total_chunks > 0 else 0
        }

    async def validate_content(self, content: str) -> Dict[str, Any]:
        """
        Validate content before processing.
        """
        errors = []

        if not content or len(content.strip()) == 0:
            errors.append("Content is empty")

        if len(content) < 10:  # Minimum content length check
            errors.append("Content is too short to process")

        return {
            "is_valid": len(errors) == 0,
            "errors": errors
        }