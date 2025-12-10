# Data Model: Backend, Qdrant Connection, and Embeddings Readiness Verification

**Date**: 2025-12-08
**Feature**: Backend, Qdrant Connection, and Embeddings Readiness Verification
**Plan**: [plan.md](./plan.md) | **Research**: [research.md](./research.md)

## Overview

This document defines the data models (Pydantic schemas) used for health check requests, responses, and configuration validation. All models are designed to be serializable to JSON and compatible with FastAPI's automatic OpenAPI documentation generation.

---

## Configuration Models

### HealthCheckSettings

**Purpose**: Validates and loads environment variables required for health checks

**Location**: `backend/src/health/config.py` (new file)

```python
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, field_validator
from typing import Optional

class HealthCheckSettings(BaseSettings):
    """
    Configuration for health check system.
    Loads from environment variables with validation.
    """
    model_config = SettingsConfigDict(env_file='.env', extra='ignore')

    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    qdrant_api_key: Optional[str] = None

    # Embeddings Provider Configuration
    anthropic_api_key: Optional[str] = None
    openai_api_key: Optional[str] = None
    google_api_key: Optional[str] = None

    # Health Check Configuration
    health_check_timeout: int = Field(default=30, ge=5, le=300, description="Global timeout in seconds")
    component_timeout: int = Field(default=5, ge=1, le=30, description="Per-component timeout in seconds")
    test_collection_name: str = "book_embeddings_test"

    @field_validator('qdrant_url', 'qdrant_host')
    @classmethod
    def validate_qdrant_connection(cls, v, info):
        """Ensure either URL or host+port is provided"""
        if not info.data.get('qdrant_url') and not info.data.get('qdrant_host'):
            raise ValueError("Either QDRANT_URL or QDRANT_HOST must be set")
        return v
```

**Fields**:
- `qdrant_url`: Full URL for cloud Qdrant instances
- `qdrant_host`: Host for local Qdrant instances (default: localhost)
- `qdrant_port`: Port for local Qdrant instances (default: 6333)
- `qdrant_api_key`: API key for authenticated Qdrant instances
- `anthropic_api_key`: Anthropic API key (primary embeddings provider)
- `openai_api_key`: OpenAI API key (optional)
- `google_api_key`: Google Generative AI API key (optional)
- `health_check_timeout`: Global timeout for all health checks (5-300 seconds)
- `component_timeout`: Timeout for individual component checks (1-30 seconds)
- `test_collection_name`: Name for test vector collection

---

## Response Models

### ComponentStatus

**Purpose**: Status information for a single system component

**Location**: `backend/src/schema.py` (add to existing file)

```python
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum

class HealthStatus(str, Enum):
    """Health status enumeration"""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"

class ComponentStatus(BaseModel):
    """Status of a single system component"""
    name: str = Field(..., description="Component name (backend, qdrant, embeddings)")
    status: HealthStatus = Field(..., description="Component health status")
    response_time_ms: Optional[int] = Field(None, description="Component response time in milliseconds")
    message: Optional[str] = Field(None, description="Human-readable status message")
    error: Optional[str] = Field(None, description="Error message if unhealthy")
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Component-specific metadata")

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
```

**Fields**:
- `name`: Component identifier (backend, qdrant, embeddings, config)
- `status`: Health status enum (healthy, degraded, unhealthy)
- `response_time_ms`: Time taken for health check in milliseconds
- `message`: Human-readable status description
- `error`: Detailed error message if component is unhealthy
- `metadata`: Component-specific information (collections, versions, etc.)

---

### HealthCheckResult

**Purpose**: Overall system health check result with all component statuses

**Location**: `backend/src/schema.py` (add to existing file)

```python
from typing import List

class HealthCheckResult(BaseModel):
    """Overall system health check result"""
    status: HealthStatus = Field(..., description="Overall system status")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Health check timestamp (UTC)")
    version: str = Field(default="1.0.0", description="API version")
    components: List[ComponentStatus] = Field(default_factory=list, description="Individual component statuses")
    total_response_time_ms: Optional[int] = Field(None, description="Total health check duration")
    errors: List[str] = Field(default_factory=list, description="List of error messages")

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
                    },
                    {
                        "name": "qdrant",
                        "status": "healthy",
                        "response_time_ms": 45,
                        "message": "Qdrant connection successful",
                        "metadata": {"collections": ["book_embeddings"]}
                    },
                    {
                        "name": "embeddings",
                        "status": "healthy",
                        "response_time_ms": 120,
                        "message": "Embeddings service operational",
                        "metadata": {"provider": "anthropic", "dimensions": 1024}
                    }
                ],
                "total_response_time_ms": 167,
                "errors": []
            }
        }
```

**Fields**:
- `status`: Overall system status (worst status among components)
- `timestamp`: ISO 8601 UTC timestamp of health check execution
- `version`: API version for compatibility tracking
- `components`: List of individual component statuses
- `total_response_time_ms`: Sum of all component response times
- `errors`: Aggregated list of all component errors

**Status Determination Logic**:
- If any component is `UNHEALTHY`: overall status is `UNHEALTHY`
- If any component is `DEGRADED`: overall status is `DEGRADED`
- If all components are `HEALTHY`: overall status is `HEALTHY`

---

### ConnectionInfo

**Purpose**: Detailed connection metadata for Qdrant

**Location**: `backend/src/schema.py` (add to existing file)

```python
class ConnectionInfo(BaseModel):
    """Qdrant connection information"""
    endpoint: str = Field(..., description="Qdrant endpoint (URL or host:port)")
    connected: bool = Field(..., description="Connection status")
    collections: List[str] = Field(default_factory=list, description="Available collections")
    version: Optional[str] = Field(None, description="Qdrant server version")
    collection_schema: Optional[Dict[str, Any]] = Field(None, description="Collection schema details")

    class Config:
        json_schema_extra = {
            "example": {
                "endpoint": "localhost:6333",
                "connected": True,
                "collections": ["book_embeddings", "book_embeddings_test"],
                "version": "1.7.3",
                "collection_schema": {
                    "name": "book_embeddings",
                    "vector_size": 1536,
                    "distance": "Cosine",
                    "points_count": 1247
                }
            }
        }
```

**Fields**:
- `endpoint`: Qdrant connection endpoint (URL or host:port)
- `connected`: Boolean connection status
- `collections`: List of collection names available
- `version`: Qdrant server version
- `collection_schema`: Schema details for primary collection

---

### EmbeddingsInfo

**Purpose**: Embeddings service validation information

**Location**: `backend/src/schema.py` (add to existing file)

```python
class EmbeddingsInfo(BaseModel):
    """Embeddings service information"""
    provider: str = Field(..., description="Embeddings provider (anthropic, openai, google)")
    available: bool = Field(..., description="Provider availability")
    dimensions: Optional[int] = Field(None, description="Embedding vector dimensions")
    test_embedding_generated: bool = Field(False, description="Whether test embedding was successfully generated")
    error: Optional[str] = Field(None, description="Error message if unavailable")

    class Config:
        json_schema_extra = {
            "example": {
                "provider": "anthropic",
                "available": True,
                "dimensions": 1024,
                "test_embedding_generated": True,
                "error": None
            }
        }
```

**Fields**:
- `provider`: Embeddings provider name (anthropic, openai, google)
- `available`: Boolean availability status
- `dimensions`: Detected embedding dimensions
- `test_embedding_generated`: Whether test embedding succeeded
- `error`: Error message if provider unavailable

---

### ConfigurationStatus

**Purpose**: Configuration validation results

**Location**: `backend/src/schema.py` (add to existing file)

```python
class ConfigurationStatus(BaseModel):
    """Configuration validation status"""
    valid: bool = Field(..., description="Overall configuration validity")
    missing_required: List[str] = Field(default_factory=list, description="Missing required parameters")
    invalid_values: Dict[str, str] = Field(default_factory=dict, description="Invalid parameter values with reasons")
    warnings: List[str] = Field(default_factory=list, description="Configuration warnings")
    validated_at: datetime = Field(default_factory=datetime.utcnow, description="Validation timestamp")

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
```

**Fields**:
- `valid`: Boolean overall configuration validity
- `missing_required`: List of missing required env vars
- `invalid_values`: Map of invalid parameters to error reasons
- `warnings`: Non-critical configuration warnings
- `validated_at`: Timestamp of validation

---

### TestWorkflowResult

**Purpose**: End-to-end test workflow execution results

**Location**: `backend/src/schema.py` (add to existing file)

```python
class TestWorkflowStep(BaseModel):
    """Single step in test workflow"""
    step_name: str = Field(..., description="Step name")
    status: HealthStatus = Field(..., description="Step status")
    duration_ms: int = Field(..., description="Step duration in milliseconds")
    details: Optional[str] = Field(None, description="Step details or error message")

class TestWorkflowResult(BaseModel):
    """End-to-end test workflow result"""
    success: bool = Field(..., description="Overall workflow success")
    total_duration_ms: int = Field(..., description="Total workflow duration")
    documents_processed: int = Field(0, description="Number of test documents processed")
    steps: List[TestWorkflowStep] = Field(default_factory=list, description="Individual workflow steps")
    errors: List[str] = Field(default_factory=list, description="Workflow errors")
    executed_at: datetime = Field(default_factory=datetime.utcnow, description="Execution timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "success": True,
                "total_duration_ms": 8742,
                "documents_processed": 10,
                "steps": [
                    {
                        "step_name": "create_test_collection",
                        "status": "healthy",
                        "duration_ms": 234,
                        "details": "Test collection created successfully"
                    },
                    {
                        "step_name": "generate_embeddings",
                        "status": "healthy",
                        "duration_ms": 3450,
                        "details": "10 embeddings generated"
                    },
                    {
                        "step_name": "store_vectors",
                        "status": "healthy",
                        "duration_ms": 1028,
                        "details": "10 vectors stored in Qdrant"
                    },
                    {
                        "step_name": "retrieve_vectors",
                        "status": "healthy",
                        "duration_ms": 245,
                        "details": "Search returned 5 results"
                    },
                    {
                        "step_name": "cleanup",
                        "status": "healthy",
                        "duration_ms": 185,
                        "details": "Test collection deleted"
                    }
                ],
                "errors": [],
                "executed_at": "2025-12-08T10:30:00Z"
            }
        }
```

**Fields**:
- `success`: Boolean overall workflow success
- `total_duration_ms`: Total workflow execution time
- `documents_processed`: Count of test documents processed
- `steps`: List of workflow steps with individual statuses
- `errors`: List of error messages encountered
- `executed_at`: Workflow execution timestamp

---

## Request Models

### EndToEndTestRequest

**Purpose**: Optional parameters for end-to-end test workflow

**Location**: `backend/src/schema.py` (add to existing file)

```python
class EndToEndTestRequest(BaseModel):
    """End-to-end test workflow request"""
    num_documents: int = Field(default=10, ge=1, le=100, description="Number of test documents to process")
    cleanup_on_failure: bool = Field(default=True, description="Whether to cleanup test collection on failure")
    test_collection_name: Optional[str] = Field(None, description="Override default test collection name")

    class Config:
        json_schema_extra = {
            "example": {
                "num_documents": 10,
                "cleanup_on_failure": True,
                "test_collection_name": None
            }
        }
```

**Fields**:
- `num_documents`: Number of test documents to process (1-100)
- `cleanup_on_failure`: Whether to delete test collection on failure
- `test_collection_name`: Optional override for test collection name

---

## Entity Relationships

```
HealthCheckResult
├── status: HealthStatus (enum)
├── timestamp: datetime
├── version: str
├── components: List[ComponentStatus]
│   ├── ComponentStatus (backend)
│   ├── ComponentStatus (qdrant) → metadata contains ConnectionInfo data
│   ├── ComponentStatus (embeddings) → metadata contains EmbeddingsInfo data
│   └── ComponentStatus (config) → metadata contains ConfigurationStatus data
├── total_response_time_ms: int
└── errors: List[str]

TestWorkflowResult
├── success: bool
├── total_duration_ms: int
├── documents_processed: int
├── steps: List[TestWorkflowStep]
│   ├── TestWorkflowStep (create_collection)
│   ├── TestWorkflowStep (generate_embeddings)
│   ├── TestWorkflowStep (store_vectors)
│   ├── TestWorkflowStep (retrieve_vectors)
│   └── TestWorkflowStep (cleanup)
├── errors: List[str]
└── executed_at: datetime
```

---

## Model Usage in Endpoints

| Endpoint | Request Model | Response Model |
|----------|--------------|----------------|
| GET /health | None | HealthCheckResult |
| GET /health/ready | None | HealthCheckResult |
| GET /health/backend | None | ComponentStatus |
| GET /health/qdrant | None | ComponentStatus (with ConnectionInfo in metadata) |
| GET /health/embeddings | None | ComponentStatus (with EmbeddingsInfo in metadata) |
| GET /health/config | None | ConfigurationStatus |
| POST /health/test/end-to-end | EndToEndTestRequest | TestWorkflowResult |

---

## Validation Rules

### HealthCheckSettings Validation
- At least one of: `qdrant_url` OR (`qdrant_host` + `qdrant_port`)
- At least one embeddings provider API key
- `health_check_timeout`: 5-300 seconds
- `component_timeout`: 1-30 seconds

### EndToEndTestRequest Validation
- `num_documents`: 1-100 (prevents excessive resource usage)

### ComponentStatus Validation
- `name`: Must be one of: backend, qdrant, embeddings, config
- `status`: Must be valid HealthStatus enum value
- `response_time_ms`: Non-negative integer

---

## Error Handling

All models include optional `error` fields for failure scenarios:

```python
# Example: Unhealthy component
ComponentStatus(
    name="qdrant",
    status=HealthStatus.UNHEALTHY,
    response_time_ms=None,
    message="Qdrant connection failed",
    error="Connection refused: localhost:6333",
    metadata={}
)

# Example: Failed workflow step
TestWorkflowStep(
    step_name="generate_embeddings",
    status=HealthStatus.UNHEALTHY,
    duration_ms=120,
    details="API rate limit exceeded: anthropic.com"
)
```

---

## Implementation Checklist

- [ ] Create `backend/src/health/config.py` with HealthCheckSettings
- [ ] Add models to `backend/src/schema.py`: HealthStatus enum, ComponentStatus, HealthCheckResult, ConnectionInfo, EmbeddingsInfo, ConfigurationStatus, TestWorkflowStep, TestWorkflowResult, EndToEndTestRequest
- [ ] Add Pydantic examples to all models for OpenAPI documentation
- [ ] Ensure all models are JSON-serializable
- [ ] Add type hints for all fields
- [ ] Test Pydantic validation rules with invalid inputs
