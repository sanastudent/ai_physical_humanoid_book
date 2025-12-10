# Research: Backend, Qdrant Connection, and Embeddings Readiness Verification

**Date**: 2025-12-08
**Feature**: Backend, Qdrant Connection, and Embeddings Readiness Verification
**Plan**: [plan.md](./plan.md)

## Research Findings

### 1. Qdrant Health Check Best Practices

**Decision**: Use Qdrant client's built-in health check capabilities combined with collection-specific validation

**Rationale**:
- Qdrant client provides `client.get_collections()` to list collections without modifying data
- Collection info endpoint returns schema details for validation: `client.get_collection(collection_name)`
- Health checks can verify:
  - Client connection (basic ping-like operation)
  - Collection existence
  - Collection schema matches expected configuration (vector size, distance metric)
  - Index readiness status

**Alternatives Considered**:
- **Custom ping operations**: More complex, doesn't leverage Qdrant's built-in capabilities
- **Test vector insertion**: Destructive, requires cleanup, higher risk
- **Direct HTTP calls**: Less maintainable, bypasses client's error handling

**Implementation Approach**:
```python
# Connection check
try:
    collections = client.get_collections()
    # Success indicates connection is healthy
except Exception as e:
    # Connection failed

# Collection validation
collection_info = client.get_collection(COLLECTION_NAME)
# Verify: vector_params.size, vector_params.distance, status
```

**Connection Pooling**: qdrant-client 1.7.3 manages connections internally; no manual pooling needed

**Timeout Configuration**: Use `timeout` parameter in QdrantClient constructor (default 5s per request)

**Non-Destructive Test Strategy**: Use separate test collection `book_embeddings_test` for write operations

---

### 2. Embeddings Service Validation

**Decision**: Validate primary embeddings provider only (Anthropic) with minimal test text

**Rationale**:
- System uses Anthropic as primary embeddings provider (based on backend/src/embed.py usage patterns)
- OpenAI and Google Generative AI may be configured but are secondary
- Minimal test text ("test") costs <$0.001 per health check
- Dimension validation ensures model compatibility without full document processing

**Alternatives Considered**:
- **Test all configured providers**: Higher cost, slower, unnecessary for basic readiness check
- **Mock embeddings**: Doesn't validate real API connectivity
- **Cached test embeddings**: Doesn't verify current API availability

**Implementation Approach**:
```python
# Test embedding generation
test_text = "test"
embedding = await embedding_generator.generate_embedding(test_text)

# Verify dimensions
expected_dimensions = 1536  # For text-embedding-ada-002 (OpenAI) or similar
if len(embedding) != expected_dimensions:
    raise ValidationError(f"Expected {expected_dimensions} dimensions, got {len(embedding)}")
```

**Model-Specific Dimensions**:
- Anthropic (Voyage): 1024 dimensions
- OpenAI (text-embedding-ada-002): 1536 dimensions
- OpenAI (text-embedding-3-small): 1536 dimensions
- Google (models/embedding-001): 768 dimensions

**Configuration**: Read expected dimensions from config or detect from first successful embedding

---

### 3. FastAPI Health Check Patterns

**Decision**: Implement Kubernetes-style /health (liveness) and /health/ready (readiness) endpoints

**Rationale**:
- **/health** (liveness): Simple check that service is running (always returns 200 unless crashed)
- **/health/ready** (readiness): Comprehensive check of all dependencies (Qdrant, embeddings)
- Follows Kubernetes health check conventions for cloud deployment compatibility
- Separate endpoints allow load balancers to distinguish between "restart needed" vs "not ready to serve traffic"

**Alternatives Considered**:
- **Single /health endpoint**: Doesn't distinguish between liveness and readiness
- **/healthz and /readyz**: Less conventional than /health paths
- **Custom health endpoint names**: Less intuitive for operations teams

**Response Format**: JSON with standardized structure
```json
{
  "status": "healthy" | "degraded" | "unhealthy",
  "timestamp": "2025-12-08T10:30:00Z",
  "version": "1.0.0",
  "components": {
    "backend": {"status": "healthy", "response_time_ms": 2},
    "qdrant": {"status": "healthy", "response_time_ms": 45, "collections": ["book_embeddings"]},
    "embeddings": {"status": "healthy", "response_time_ms": 120, "provider": "anthropic"}
  },
  "errors": []
}
```

**Status Codes**:
- 200 OK: All components healthy
- 503 Service Unavailable: One or more critical components unhealthy
- 207 Multi-Status: Partial health (some components degraded but system operational)

**Async vs Sync**: Use async health checks for parallel component validation (faster total time)

---

### 4. Configuration Validation Strategies

**Decision**: Use Pydantic Settings with custom validators for environment variables

**Rationale**:
- Backend already uses Pydantic 2.5.3 and python-dotenv 1.0.0
- Pydantic Settings provides automatic env var loading and validation
- Custom validators can enforce required combinations (e.g., QDRANT_URL XOR QDRANT_HOST+PORT)
- Type-safe configuration with IDE autocomplete support

**Alternatives Considered**:
- **Manual os.getenv() checks**: Error-prone, lacks type safety, scattered validation logic
- **python-decouple**: Additional dependency, less Pythonic than Pydantic
- **dynaconf**: Overcomplicated for simple env var validation

**Implementation Approach**:
```python
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, field_validator

class HealthCheckSettings(BaseSettings):
    model_config = SettingsConfigDict(env_file='.env', extra='ignore')

    qdrant_url: str | None = None
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    qdrant_api_key: str | None = None

    anthropic_api_key: str | None = None
    openai_api_key: str | None = None
    google_api_key: str | None = None

    health_check_timeout: int = Field(default=30, ge=5, le=300)
    test_collection_name: str = "book_embeddings_test"

    @field_validator('qdrant_url', 'qdrant_host')
    def validate_qdrant_connection(cls, v, info):
        # Ensure either URL or host+port is provided
        if not info.data.get('qdrant_url') and not info.data.get('qdrant_host'):
            raise ValueError("Either QDRANT_URL or QDRANT_HOST must be set")
        return v

    @field_validator('anthropic_api_key', 'openai_api_key', 'google_api_key')
    def validate_at_least_one_provider(cls, v, info):
        # At least one embeddings provider must be configured
        if not any([
            info.data.get('anthropic_api_key'),
            info.data.get('openai_api_key'),
            info.data.get('google_api_key')
        ]):
            raise ValueError("At least one embeddings provider API key must be configured")
        return v
```

**Required vs Optional**:
- Required: Qdrant connection (URL or host+port), at least one embeddings API key
- Optional: Qdrant API key (for cloud instances), timeout overrides, test collection name

---

### 5. End-to-End Testing Without Production Impact

**Decision**: Use separate test collection with automatic cleanup in try/finally blocks

**Rationale**:
- Separate collection (`book_embeddings_test`) completely isolates test data from production
- Cleanup in finally blocks ensures test vectors are removed even if test fails
- Collection can be recreated for each test run (idempotent)
- Production collection remains untouched

**Alternatives Considered**:
- **Metadata filtering in production collection**: Risk of accidental query against test data
- **Namespace-based isolation**: Qdrant doesn't support namespaces natively
- **Manual cleanup**: Easy to forget, leaves orphaned test data
- **Temporary collection per test**: Slower, higher resource usage

**Implementation Pattern**:
```python
async def end_to_end_test_workflow():
    test_collection = "book_embeddings_test"
    test_vectors = []

    try:
        # Create test collection
        await qdrant_client.recreate_collection(
            collection_name=test_collection,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )

        # Generate test documents
        test_docs = [f"Test document {i}" for i in range(10)]

        # Test workflow: embed -> store -> retrieve
        for i, doc in enumerate(test_docs):
            embedding = await embedding_generator.generate_embedding(doc)
            point_id = f"test_{i}"
            await qdrant_client.upsert(
                collection_name=test_collection,
                points=[PointStruct(id=point_id, vector=embedding, payload={"text": doc})]
            )
            test_vectors.append(point_id)

        # Verify retrieval
        search_results = await qdrant_client.search(
            collection_name=test_collection,
            query_vector=await embedding_generator.generate_embedding(test_docs[0]),
            limit=5
        )

        assert len(search_results) > 0, "No results returned"

        return {"status": "success", "documents_processed": len(test_docs)}

    finally:
        # Guaranteed cleanup
        try:
            await qdrant_client.delete_collection(collection_name=test_collection)
        except Exception as e:
            logger.warning(f"Failed to cleanup test collection: {e}")
```

**Idempotency**: `recreate_collection()` ensures clean slate for each test run

**Cleanup Guarantee**: finally block ensures cleanup even on exceptions

---

## Technology Decisions Summary

| Decision Area | Choice | Key Reason |
|--------------|--------|------------|
| Health Check Endpoints | /health (liveness) + /health/ready (readiness) | Kubernetes convention, clear separation of concerns |
| Qdrant Test Strategy | Separate test collection with auto-cleanup | Complete isolation, no production impact |
| Embeddings Provider Testing | Primary provider only (Anthropic) with minimal text | Cost-effective, validates real connectivity |
| Configuration Validation | Pydantic Settings with custom validators | Type-safe, already in use, comprehensive validation |
| Response Format | JSON with status/components/errors structure | Standardized, actionable, machine-readable |
| Execution Model | Async/await with parallel component checks | Faster total health check time (<30s goal) |
| Timeout Configuration | Configurable via env vars (default 30s global, 5s per component) | Flexible for different network conditions |
| Error Reporting | Detailed error messages with component-specific diagnostics | Actionable troubleshooting information |

---

## Implementation Priorities

Based on spec.md user stories and priorities:

1. **P1 - System Health Verification**:
   - Implement /health and /health/ready endpoints
   - Backend service health check (always healthy if responding)
   - Qdrant connection and collection validation
   - Embeddings service validation

2. **P2 - Component Dependency Validation**:
   - End-to-end test workflow (embed → store → retrieve)
   - Inter-component communication verification

3. **P3 - Configuration Validation**:
   - Pydantic Settings for env var validation
   - /health/config endpoint for configuration status

---

## Open Questions Resolved

✅ **Which embeddings providers to test**: Primary provider (Anthropic) only
✅ **Test collection naming**: `book_embeddings_test` (configurable via env var)
✅ **Cleanup strategy**: try/finally with collection deletion
✅ **Timeout values**: 30s global, 5s per component (configurable)
✅ **Health check endpoint paths**: /health and /health/ready
✅ **Configuration validation approach**: Pydantic Settings with custom validators
✅ **Error response structure**: JSON with status, components, errors, timestamp
✅ **Async vs sync**: Async for parallel execution

---

## Next Phase

All research questions resolved. Ready to proceed to **Phase 1: Design & Contracts**:
- Create `data-model.md` with Pydantic models
- Create `contracts/health-api.yaml` with OpenAPI specification
- Create `quickstart.md` with usage instructions
