"""
Qdrant schema definition for book embeddings

Implements FR-008: The backend MUST store processed content in a vector database
with a defined schema (e.g., collection_name: "book_embeddings", vector_size: 1536,
distance: "Cosine", payload_fields: id, chapter, text, token_count).
"""
from pydantic import BaseModel
from typing import Optional, Dict, Any
from qdrant_client.http import models


class BookChunk(BaseModel):
    """Represents a chunk of book text with metadata for embedding and RAG"""
    id: str
    chapter: str
    text: str
    token_count: int


class EmbedRequest(BaseModel):
    """Request model for embedding book content"""
    content: str
    chapter: str
    book_id: Optional[str] = None  # Added for book identification


class QueryRequest(BaseModel):
    """Request model for RAG queries"""
    query: str
    book_id: Optional[str] = None  # Added for book-specific queries
    mode: str = "global"  # "global" or "selected"
    context: Optional[str] = None  # For selected-text mode


class QueryResponse(BaseModel):
    """Response model for RAG queries"""
    answer: str
    citations: list[str]
    sources: list[dict]


class QdrantSchema:
    """Complete Qdrant schema definition implementing FR-008 requirements"""

    # FR-008 requirements
    COLLECTION_NAME = "book_embeddings"
    VECTOR_SIZE = 1536  # FR-008: vector_size: 1536
    DISTANCE_METRIC = "Cosine"  # FR-008: distance: "Cosine"

    # FR-008: payload_fields: id, chapter, text, token_count
    PAYLOAD_SCHEMA = {
        "id": models.PayloadSchemaType.KEYWORD,
        "chapter": models.PayloadSchemaType.TEXT,
        "text": models.PayloadSchemaType.TEXT,
        "token_count": models.PayloadSchemaType.INTEGER,
        "book_id": models.PayloadSchemaType.KEYWORD,  # Additional field for organization
    }

    @classmethod
    def get_collection_config(cls) -> models.CreateCollection:
        """Get Qdrant collection configuration as required by FR-008"""
        return models.CreateCollection(
            vectors_config=models.VectorParams(
                size=cls.VECTOR_SIZE,
                distance=models.Distance(cls.DISTANCE_METRIC.upper())
            ),
            # Additional payload field configurations
            on_disk_payload=True
        )

    @classmethod
    def get_payload_schema(cls) -> Dict[str, Any]:
        """Get payload schema definition"""
        return cls.PAYLOAD_SCHEMA


# Legacy constants for backward compatibility
COLLECTION_NAME = QdrantSchema.COLLECTION_NAME
VECTOR_SIZE = QdrantSchema.VECTOR_SIZE
DISTANCE_METRIC = QdrantSchema.DISTANCE_METRIC
