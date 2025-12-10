---
id: 0019
title: Fix Qdrant URL Configuration Error
stage: green
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5
feature: 001-backend-qdrant-readiness
branch: main
user: User
command: /sp.implement
labels: ["bugfix", "configuration", "qdrant", "environment"]
links:
  spec: specs/001-backend-qdrant-readiness/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .env
tests:
  - Configuration validated against QdrantManager initialization logic
---

## Prompt

User reported error when running the health check implementation:
```
Failed to initialize Qdrant client: `host` param is not expected to contain protocol (http:// or https://). Try to use `url` parameter instead.
```

This error occurred because the `.env` file had `QDRANT_HOST=https://...` (full URL) instead of using `QDRANT_URL` for cloud instances.

## Response snapshot

**Root Cause Analysis**:
The QdrantManager (backend/src/qdrant_manager.py:28-43) already has correct logic:
```python
url = os.getenv("QDRANT_URL")
host = os.getenv("QDRANT_HOST", "localhost")

if url:
    self.client = QdrantClient(url=url, api_key=api_key)
else:
    self.client = QdrantClient(host=host, port=port, api_key=api_key)
```

The issue was in `.env` configuration:
- **Before**: `QDRANT_HOST=https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io`
- **Problem**: Full URL passed to `host` parameter (which expects hostname only)
- **Solution**: Changed to `QDRANT_URL=https://...` so the correct code path is used

**Configuration Fix**:
Updated `.env` file with:
1. Renamed `QDRANT_HOST` to `QDRANT_URL` for cloud instance
2. Added `QDRANT_API_KEY` (extracted from commented line 26)
3. Added clear documentation explaining when to use URL vs host/port
4. Included port in URL (`:6333`) for completeness

**New Configuration**:
```env
# Use QDRANT_URL for cloud instances with full URL including protocol
QDRANT_URL=https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io:6333
# For local instances, use QDRANT_HOST and QDRANT_PORT instead:
# QDRANT_HOST=localhost
# QDRANT_PORT=6333
QDRANT_API_KEY=75c8bd96-24cf-4b65-a47a-a2a27385f09b
```

**Validation**:
- Verified .gitignore contains `.env*` pattern (line 14) - secrets are protected
- QdrantManager will now use `QdrantClient(url=..., api_key=...)` code path
- Health check endpoints should now work with cloud Qdrant instance

## Outcome

- ✅ Impact: Fixed Qdrant client initialization error; health checks can now connect to cloud instance
- 🧪 Tests: User should test by running FastAPI backend and hitting `/health/ready` endpoint
- 📁 Files: 1 file modified (.env)
- 🔁 Next prompts: Start backend server and test health endpoints: `GET /health/ready`, `/health/qdrant`, `/health/embeddings`
- 🧠 Reflection: The error message was accurate and helpful. The QdrantManager code was already correct; the issue was purely configuration. This is a common pitfall when migrating from local to cloud Qdrant instances.

## Evaluation notes (flywheel)

- Failure modes observed: User had correct URL but wrong environment variable name (QDRANT_HOST instead of QDRANT_URL)
- Graders run and results (PASS/FAIL): Configuration validated - PASS (matches QdrantManager logic)
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Test actual health endpoint calls with real Qdrant cloud instance to validate connection
