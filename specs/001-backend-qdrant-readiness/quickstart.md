# Quickstart Guide: Health Check System

**Date**: 2025-12-08
**Feature**: Backend, Qdrant Connection, and Embeddings Readiness Verification
**Plan**: [plan.md](./plan.md) | **Contracts**: [health-api.yaml](./contracts/health-api.yaml)

## Overview

This guide explains how to use the health check system to verify that all components of the AI-Driven Book RAG system are operational and ready to serve requests.

---

## Prerequisites

Before using the health check endpoints, ensure:

1. **Backend service is running**:
   ```bash
   cd backend
   uvicorn src.main:app --reload --port 8000
   ```

2. **Required environment variables are set** (see [Configuration](#configuration) section below)

3. **Qdrant is accessible** (cloud instance or local Docker container)

---

## Quick Health Check

### Check Overall System Health

```bash
curl http://localhost:8000/health/ready
```

**Expected Response** (all healthy):
```json
{
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
      "metadata": {
        "collections": ["book_embeddings"],
        "version": "1.7.3"
      }
    },
    {
      "name": "embeddings",
      "status": "healthy",
      "response_time_ms": 120,
      "message": "Embeddings service operational",
      "metadata": {
        "provider": "anthropic",
        "dimensions": 1024
      }
    }
  ],
  "total_response_time_ms": 167,
  "errors": []
}
```

### Check Individual Components

**Backend Only**:
```bash
curl http://localhost:8000/health/backend
```

**Qdrant Only**:
```bash
curl http://localhost:8000/health/qdrant
```

**Embeddings Only**:
```bash
curl http://localhost:8000/health/embeddings
```

**Configuration Only**:
```bash
curl http://localhost:8000/health/config
```

---

## Run End-to-End Test

The end-to-end test validates the complete workflow: document → embedding → storage → retrieval.

### Basic Test (10 documents)

```bash
curl -X POST http://localhost:8000/health/test/end-to-end
```

### Custom Test (50 documents)

```bash
curl -X POST http://localhost:8000/health/test/end-to-end \
  -H "Content-Type: application/json" \
  -d '{
    "num_documents": 50,
    "cleanup_on_failure": true,
    "test_collection_name": "my_test_collection"
  }'
```

**Expected Response** (successful test):
```json
{
  "success": true,
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
```

---

## Configuration

### Required Environment Variables

Create a `.env` file in the `backend/` directory with the following variables:

```bash
# Qdrant Configuration (choose ONE connection method)

# Option 1: Cloud Qdrant (URL-based)
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key

# Option 2: Local Qdrant (host/port-based)
QDRANT_HOST=localhost
QDRANT_PORT=6333

# Embeddings Provider (at least ONE required)
ANTHROPIC_API_KEY=your_anthropic_api_key
OPENAI_API_KEY=your_openai_api_key        # Optional
GOOGLE_API_KEY=your_google_api_key        # Optional

# Health Check Configuration (optional - defaults shown)
HEALTH_CHECK_TIMEOUT=30                    # Global timeout in seconds (5-300)
COMPONENT_TIMEOUT=5                        # Per-component timeout (1-30)
TEST_COLLECTION_NAME=book_embeddings_test  # Test collection name
```

### Local Qdrant Setup (Docker)

If you don't have Qdrant running, start a local instance:

```bash
docker run -p 6333:6333 qdrant/qdrant
```

Then set in `.env`:
```bash
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

---

## Interpreting Health Check Results

### Status Codes

| Status | Meaning | Action |
|--------|---------|--------|
| `healthy` | Component is fully operational | None - system ready |
| `degraded` | Component is operational but experiencing issues | Monitor and investigate warnings |
| `unhealthy` | Component is not operational | Investigate error messages and fix |

### HTTP Response Codes

| Code | Endpoint | Meaning |
|------|----------|---------|
| 200 OK | `/health` | Service is alive (liveness) |
| 200 OK | `/health/ready` | All components healthy (readiness) |
| 503 Service Unavailable | `/health/ready` | One or more components unhealthy |

### Component-Specific Metadata

**Qdrant Metadata**:
```json
{
  "endpoint": "localhost:6333",
  "connected": true,
  "collections": ["book_embeddings", "book_embeddings_test"],
  "version": "1.7.3",
  "collection_schema": {
    "name": "book_embeddings",
    "vector_size": 1536,
    "distance": "Cosine",
    "points_count": 1247
  }
}
```

**Embeddings Metadata**:
```json
{
  "provider": "anthropic",
  "available": true,
  "dimensions": 1024,
  "test_embedding_generated": true
}
```

---

## Troubleshooting

### Qdrant Connection Failed

**Error**: `Connection refused: localhost:6333`

**Solutions**:
1. Verify Qdrant is running: `docker ps | grep qdrant`
2. Check `QDRANT_HOST` and `QDRANT_PORT` in `.env`
3. For cloud Qdrant, verify `QDRANT_URL` and `QDRANT_API_KEY`

**Test Connection**:
```bash
curl http://localhost:6333/collections
```

### Embeddings Service Unavailable

**Error**: `API authentication failed: Invalid API key`

**Solutions**:
1. Verify API key in `.env`: `echo $ANTHROPIC_API_KEY`
2. Check API key is valid on provider's dashboard
3. Ensure at least one provider key is set

**Test Anthropic API**:
```bash
curl https://api.anthropic.com/v1/messages \
  -H "x-api-key: $ANTHROPIC_API_KEY" \
  -H "anthropic-version: 2023-06-01" \
  -H "content-type: application/json" \
  -d '{
    "model": "claude-3-haiku-20240307",
    "max_tokens": 10,
    "messages": [{"role": "user", "content": "test"}]
  }'
```

### Configuration Validation Failed

**Error**: `Missing required: QDRANT_URL, ANTHROPIC_API_KEY`

**Solutions**:
1. Check `.env` file exists in `backend/` directory
2. Verify all required variables are set
3. Restart backend service to reload environment variables

**Verify Configuration**:
```bash
curl http://localhost:8000/health/config
```

### End-to-End Test Failed

**Error**: `API rate limit exceeded: anthropic.com`

**Solutions**:
1. Reduce `num_documents` in test request
2. Wait for rate limit reset
3. Use different API key if available

**Test with Fewer Documents**:
```bash
curl -X POST http://localhost:8000/health/test/end-to-end \
  -H "Content-Type: application/json" \
  -d '{"num_documents": 3}'
```

### Timeouts

**Error**: `Health check timeout exceeded`

**Solutions**:
1. Increase timeout in `.env`: `HEALTH_CHECK_TIMEOUT=60`
2. Check network connectivity to external services
3. Reduce `num_documents` in end-to-end tests

---

## Integration with Monitoring Tools

### Kubernetes Liveness and Readiness Probes

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: ai-book-rag-backend
spec:
  containers:
  - name: backend
    image: ai-book-rag-backend:latest
    livenessProbe:
      httpGet:
        path: /health
        port: 8000
      initialDelaySeconds: 10
      periodSeconds: 30
      timeoutSeconds: 5
    readinessProbe:
      httpGet:
        path: /health/ready
        port: 8000
      initialDelaySeconds: 15
      periodSeconds: 10
      timeoutSeconds: 10
```

### Prometheus Metrics (Future Enhancement)

While not part of this implementation, health check data can be exported to Prometheus:

```python
# Example Prometheus metrics (not implemented yet)
health_check_duration_seconds{component="qdrant"}
health_check_status{component="qdrant", status="healthy"}
```

### Uptime Monitoring (UptimeRobot, Pingdom, etc.)

Configure your uptime monitoring tool to check:
- **URL**: `https://your-api.com/health/ready`
- **Expected Status**: `200 OK`
- **Check Interval**: Every 5 minutes
- **Alert**: If status is not `200` for 2 consecutive checks

---

## Best Practices

1. **Use /health for liveness, /health/ready for readiness**: This follows Kubernetes conventions
2. **Monitor response times**: If health checks take >30s, investigate performance issues
3. **Test regularly**: Run end-to-end tests in staging before deploying to production
4. **Don't run end-to-end tests in production frequently**: They consume API credits and resources
5. **Set appropriate timeouts**: Slower networks may need higher timeout values
6. **Check configuration on deployment**: Always verify `/health/config` after deploying

---

## Next Steps

After verifying system health:

1. **Implement monitoring**: Set up Kubernetes probes or uptime monitoring
2. **Create alerts**: Configure alerts for health check failures
3. **Document baselines**: Record typical response times for each component
4. **Test failure scenarios**: Deliberately break components to verify health checks detect issues
5. **Integrate with CI/CD**: Run health checks as part of deployment pipeline

---

## API Reference

For complete API documentation, see:
- [OpenAPI Specification](./contracts/health-api.yaml)
- [Data Models](./data-model.md)
- [Implementation Plan](./plan.md)

---

## Support

If you encounter issues not covered in this guide:

1. Check backend logs: `docker logs <container-id>` or application logs
2. Verify all environment variables are set correctly
3. Test individual components separately (Qdrant, embeddings API)
4. Review [data-model.md](./data-model.md) for response structure details
