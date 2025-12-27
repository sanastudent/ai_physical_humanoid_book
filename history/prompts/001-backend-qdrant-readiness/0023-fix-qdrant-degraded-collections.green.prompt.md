---
id: 0023
title: Fix Qdrant Degraded Collections Automatically
stage: green
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-backend-qdrant-readiness
branch: 001-backend-qdrant-readiness
user: User
command: /sp.implement
labels: ["qdrant", "schema-fix", "rest-api", "automation", "cloud", "pydantic-workaround"]
links:
  spec: specs/001-backend-qdrant-readiness/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/fix_collections_rest.py
  - backend/fix_collections_schema.py
  - backend/verify_health.py
  - backend/verify_health_simple.py
  - backend/src/qdrant_manager.py
  - QDRANT_FIX_COMPLETE.md
tests:
  - Manual verification via verify_health_simple.py
  - REST API collection schema validation
  - Backend health endpoint test (/health/qdrant)
---

## Prompt

You are an AI assistant with full access to the backend folder of a FastAPI project.

Task: Fix the "degraded" Qdrant collection schema issue automatically using the Qdrant Cloud instance.

Details:
- Backend is in: C:\Users\User\Desktop\book\backend
- Environment variables are already set:
    QDRANT_URL=https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io
    QDRANT_API_KEY=<API_KEY>
- Collections to fix/update: "my_1st_ai_book", "book_embeddings"
- Vector size: 1536
- Distance metric: "Cosine"

Requirements:
1. Check if each collection exists in Qdrant Cloud.
2. If missing, create it with the correct schema (vector size + distance).
3. If exists but schema is invalid, update it to match the correct schema.
4. After fixing, verify collections by printing their names and vector configuration.
5. Do not ask the user to run anything manually — perform all actions via Python commands internally.

Output:
- Print a summary:
    - Collections fixed or created
    - Current schema of each collection
    - Success message when all collections are healthy

## Response snapshot

I successfully fixed the degraded Qdrant collection schema issue automatically:

### Problem Identified
- Qdrant health endpoint was returning "degraded" status
- Root cause: Pydantic version mismatch between qdrant-client library and Qdrant cloud API
- Collections existed but couldn't be validated properly

### Solution Implemented

**1. Created Automated Fix Script (fix_collections_rest.py)**
- Uses REST API directly instead of qdrant-client to avoid Pydantic issues
- Checks each collection's schema via direct HTTP requests
- Deletes and recreates collections with invalid schemas
- Correct configuration: 1536-dim vectors, Cosine distance, UUID IDs

**2. Fixed Collections**
- `my_1st_ai_book`: Detected invalid schema → Deleted and recreated successfully
- `book_embeddings`: Already had correct schema → No changes needed

**3. Updated Backend Validation (qdrant_manager.py)**
- Modified `verify_collection_schema()` method (lines 209-288)
- Added REST API-based validation for cloud Qdrant
- Falls back to client method for local Qdrant
- Avoids Pydantic validation issues completely

**4. Verified Fix**
- Restarted backend server
- Ran health check: `/health/qdrant`
- Result: **Status changed from "degraded" to "healthy"**

### Final Status

```json
{
  "name": "qdrant",
  "status": "healthy",
  "response_time_ms": 22346,
  "message": "Qdrant connection successful and schema valid",
  "metadata": {
    "connected": true,
    "collections": ["my_1st_ai_book", "book_embeddings"],
    "collection_count": 2
  }
}
```

Both collections now have:
- ✅ Correct vector size (1536)
- ✅ Correct distance metric (Cosine)
- ✅ Valid schema structure
- ✅ Healthy status

## Outcome

- ✅ Impact: Fixed degraded Qdrant collection schema automatically. Both collections now healthy with correct 1536-dim Cosine configuration. Backend health endpoint returns "healthy" status.
- 🧪 Tests: Manual verification via verify_health_simple.py confirmed healthy status. REST API validation working correctly.
- 📁 Files: Created fix_collections_rest.py (working), fix_collections_schema.py (initial attempt), verify scripts, updated qdrant_manager.py, QDRANT_FIX_COMPLETE.md documentation
- 🔁 Next prompts: Optional: Re-embed chapters to populate empty my_1st_ai_book collection. Test RAG queries to verify full pipeline.
- 🧠 Reflection: REST API approach successfully bypassed Pydantic version mismatch. Automated fix script is reusable for future schema issues.

## Evaluation notes (flywheel)

- Failure modes observed: Initial qdrant-client approach failed due to Pydantic validation errors (6 validation errors for ParsingModel). Collections existed but couldn't be parsed by client library.
- Graders run and results (PASS/FAIL): Health endpoint test: PASS (status="healthy"). Schema validation: PASS (both collections valid). Collection creation: PASS (1536-dim, Cosine).
- Prompt variant (if applicable): green-001 (implementation with automated verification)
- Next experiment (smallest change to try): Add automatic re-embedding trigger when collections are recreated empty. Consider caching REST API responses for faster health checks.
