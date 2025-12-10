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
                distance=models.Distance.COSINE  # Use enum directly, not string
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


# ============================================================================
# Health Check Models (Feature: Backend Qdrant Readiness Verification)
# ============================================================================

from enum import Enum
from datetime import datetime
from typing import List


class HealthStatus(str, Enum):
    """Health status enumeration for component health checks"""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class ComponentStatus(BaseModel):
    """Status of a single system component"""
    name: str
    status: HealthStatus
    response_time_ms: Optional[int] = None
    message: Optional[str] = None
    error: Optional[str] = None
    metadata: Dict[str, Any] = {}

    class Config:
        json_schema_extra = {
            "example": {
                "name": "qdrant",
                "status": "healthy",
                "response_time_ms": 45,
                "message": "Qdrant connection successful",
                "error": None,
                "metadata": {
                    "collections": ["book_embeddings"],
                    "version": "1.7.3"
                }
            }
        }


class HealthCheckResult(BaseModel):
    """Overall system health check result"""
    status: HealthStatus
    timestamp: datetime = None
    version: str = "1.0.0"
    components: List[ComponentStatus] = []
    total_response_time_ms: Optional[int] = None
    errors: List[str] = []

    def __init__(self, **data):
        if data.get('timestamp') is None:
            data['timestamp'] = datetime.utcnow()
        super().__init__(**data)

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-12-08T10:30:00Z",
                "version": "1.0.0",
                "components": [
                    {
                        "name": "backend",
                        "status": "healthy",
                        "response_time_ms": 2,
                        "message": "Backend service operational"
                    }
                ],
                "total_response_time_ms": 167,
                "errors": []
            }
        }


class ConfigurationStatus(BaseModel):
    """Configuration validation status"""
    valid: bool
    missing_required: List[str] = []
    invalid_values: Dict[str, str] = {}
    warnings: List[str] = []
    validated_at: datetime = None

    def __init__(self, **data):
        if data.get('validated_at') is None:
            data['validated_at'] = datetime.utcnow()
        super().__init__(**data)

    class Config:
        json_schema_extra = {
            "example": {
                "valid": True,
                "missing_required": [],
                "invalid_values": {},
                "warnings": ["GOOGLE_API_KEY not set - Google embeddings unavailable"],
                "validated_at": "2025-12-08T10:30:00Z"
            }
        }


class TestWorkflowStep(BaseModel):
    """Single step in test workflow"""
    step_name: str
    status: HealthStatus
    duration_ms: int
    details: Optional[str] = None


class TestWorkflowResult(BaseModel):
    """End-to-end test workflow result"""
    success: bool
    total_duration_ms: int
    documents_processed: int = 0
    steps: List[TestWorkflowStep] = []
    errors: List[str] = []
    executed_at: datetime = None

    def __init__(self, **data):
        if data.get('executed_at') is None:
            data['executed_at'] = datetime.utcnow()
        super().__init__(**data)


class EndToEndTestRequest(BaseModel):
    """End-to-end test workflow request"""
    num_documents: int = 10
    cleanup_on_failure: bool = True
    test_collection_name: Optional[str] = None

    class Config:
        json_schema_extra = {
            "example": {
                "num_documents": 10,
                "cleanup_on_failure": True,
                "test_collection_name": None
            }
        }
