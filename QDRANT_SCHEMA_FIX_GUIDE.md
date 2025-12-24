# Qdrant Collection Schema Fix - Complete Guide

## Problem

You're getting:
```
status: degraded
message: "Qdrant connected but collection schema invalid"
```

And RAG queries return:
```
"No relevant context found — check embeddings ingestion or vector search configuration"
```

## Root Cause

The Qdrant collection was created with incompatible settings or contains invalid point IDs (string IDs instead of UUIDs).

## Solution Overview

We'll:
1. ✅ Check current status
2. ✅ Safely backup collection info
3. ✅ Delete invalid collection
4. ✅ Create new collection with correct schema
5. ✅ Re-embed all chapters with UUID IDs
6. ✅ Verify everything works

## Prerequisites

### 1. Ensure Backend is Running

```bash
cd backend
python -m src.main
```

Should see:
```
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 2. Ensure Qdrant is Running

**If using Docker:**
```bash
docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

**Or check existing container:**
```bash
docker ps | grep qdrant
```

**Test Qdrant:**
```bash
curl http://localhost:6333
```

Expected:
```json
{
  "title": "qdrant - vector search engine",
  "version": "1.7.0"
}
```

### 3. Install Required Python Packages

```bash
pip install qdrant-client requests
```

## Step-by-Step Fix

### Option 1: Automated Fix (Recommended)

```bash
python fix_qdrant_schema.py
```

The script will:
1. Check current Qdrant health via `/health/qdrant`
2. Backup collection info to `qdrant_backup_info.json`
3. Ask for confirmation before deletion
4. Delete invalid collection
5. Create new collection with correct schema:
   - Vector size: 1536
   - Distance: Cosine
   - ID type: UUID
6. Re-embed all 16 chapters
7. Verify health is now "healthy"

**Expected Output:**
```
======================================================================
 QDRANT COLLECTION SCHEMA FIX
======================================================================
This script will:
1. Check current Qdrant health
2. Backup collection info
3. Delete invalid collection
4. Create new collection with correct schema (UUID IDs)
5. Re-embed all chapters
6. Verify fix

======================================================================
 STEP 1: Check Current Qdrant Health
======================================================================
Status: degraded
Message: Qdrant connected but collection schema invalid

======================================================================
 STEP 2: Backup Collection Info (Safe Mode)
======================================================================
✓ Backup saved to: qdrant_backup_info.json
  Vectors count: 150
  Points count: 150

----------------------------------------------------------------------
Continue with collection deletion and recreation? (yes/no): yes

======================================================================
 STEP 3: Delete Invalid Collection
======================================================================
✓ Deleted collection: book_embeddings

======================================================================
 STEP 4: Create Collection with Correct Schema
======================================================================
Creating collection with:
  - Name: book_embeddings
  - Vector size: 1536 (OpenAI text-embedding-3-small)
  - Distance: Cosine
  - ID type: UUID (required by Qdrant)
✓ Collection created successfully
✓ Verified - Vector size: 1536
✓ Verified - Distance: COSINE

======================================================================
 STEP 5: Re-Embed All Chapters with UUID IDs
======================================================================
Found 16 chapters to embed

[1/16] Embedding: module1-architecture...
  ✓ Success: 12 chunks created
[2/16] Embedding: module1-communication...
  ✓ Success: 15 chunks created
...
✓ Successfully embedded 16/16 chapters
✓ Total chunks created: 245

======================================================================
 STEP 6: Verify Fix
======================================================================
Status: healthy
Message: Qdrant connection successful
Collection: book_embeddings
Vectors count: 245

Testing RAG query...
✓ RAG query successful - embeddings working!

======================================================================
 SUCCESS!
======================================================================
✓ Qdrant collection schema is now valid
✓ All chapters embedded with UUID IDs
✓ RAG queries should now work
```

### Option 2: Manual Fix

If you prefer manual control:

#### Step 1: Check Health

```bash
curl http://localhost:8000/health/qdrant
```

#### Step 2: Delete Collection (via Qdrant API)

```bash
curl -X DELETE http://localhost:6333/collections/book_embeddings
```

Response:
```json
{
  "result": true,
  "status": "ok",
  "time": 0.001
}
```

#### Step 3: Create Collection (via Backend)

The backend will auto-create on first embed, OR use Qdrant API:

```bash
curl -X PUT http://localhost:6333/collections/book_embeddings \
  -H "Content-Type: application/json" \
  -d '{
    "vectors": {
      "size": 1536,
      "distance": "Cosine"
    }
  }'
```

#### Step 4: Re-Embed Chapters

```bash
python embed_chapters_fixed.py
```

#### Step 5: Verify

```bash
curl http://localhost:8000/health/qdrant
```

Expected:
```json
{
  "status": "healthy",
  "message": "Qdrant connection successful",
  "metadata": {
    "collection": "book_embeddings",
    "vectors_count": 245
  }
}
```

## Verification Tests

### Test 1: Health Check

```bash
curl http://localhost:8000/health/qdrant
```

**Expected:**
- `status`: `"healthy"` (not "degraded")
- `vectors_count`: > 0

### Test 2: RAG Query

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?"
  }'
```

**Expected:**
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is the next generation...",
  "citations": ["[module1-intro]"],
  "sources": [
    {
      "chapter": "module1-intro",
      "text": "ROS 2 is a robotics middleware...",
      "score": 0.92
    }
  ]
}
```

**NOT:**
```json
{
  "answer": "No relevant context found — check embeddings ingestion..."
}
```

### Test 3: Check Qdrant Directly

```bash
curl http://localhost:6333/collections/book_embeddings
```

**Expected:**
```json
{
  "result": {
    "status": "green",
    "vectors_count": 245,
    "points_count": 245,
    "config": {
      "params": {
        "vectors": {
          "size": 1536,
          "distance": "Cosine"
        }
      }
    }
  }
}
```

## Example /embed Payload

```json
{
  "content": "# Module 1: Introduction to ROS 2\n\n## Overview\n\nROS 2 (Robot Operating System 2) is the next generation of the most widely used robotics middleware framework...",
  "chapter": "module1-intro",
  "book_id": "physical-ai-humanoid"
}
```

**Backend Processing:**
1. Chunks content into 500-token pieces
2. Generates UUID for each chunk (e.g., `"a3bb189e-8bf9-4e57-a7f8-90ee0ff19b98"`)
3. Creates embeddings (1536-dim vectors)
4. Stores in Qdrant with payload:
   ```json
   {
     "id": "a3bb189e-8bf9-4e57-a7f8-90ee0ff19b98",
     "vector": [0.123, -0.456, ...],  // 1536 dimensions
     "payload": {
       "chapter": "module1-intro",
       "text": "ROS 2 is...",
       "token_count": 450,
       "book_id": "physical-ai-humanoid"
     }
   }
   ```

## What Each Script Does

### `fix_qdrant_schema.py`

**Safe Operations:**
- ✅ Backups collection info before deletion
- ✅ Asks for confirmation before destructive actions
- ✅ Uses proper UUID generation (fixed in backend)
- ✅ Verifies each step

**Steps:**
1. Connects to Qdrant via `qdrant-client`
2. Gets collection info and saves to `qdrant_backup_info.json`
3. Deletes old collection (after confirmation)
4. Creates new collection with correct schema
5. Calls `/embed` endpoint for each chapter
6. Backend generates UUIDs and stores properly
7. Verifies via `/health/qdrant`

### `embed_chapters_fixed.py`

**What it does:**
- Reads all `.md` files from `frontend/my-book/docs/chapters`
- Sends each to `/embed` endpoint
- Backend chunks content and generates UUID IDs
- Reports progress and success/failure

## Troubleshooting

### Error: "Connection refused" to Qdrant

**Fix:**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

### Error: "Connection refused" to Backend

**Fix:**
```bash
cd backend
python -m src.main
```

### Still Getting "No relevant context found"

**Possible causes:**
1. Embeddings not fully indexed yet (wait 30 seconds)
2. Query doesn't match embedded content (try different query)
3. Collection still empty (check `vectors_count`)

**Verify:**
```bash
curl http://localhost:6333/collections/book_embeddings
```

Should show `vectors_count > 0` and `status: "green"`

### UUID Error in Logs

If you still see UUID errors, ensure backend code is updated:

**Check `backend/src/embed.py` line 65:**
```python
"id": str(uuid.uuid4()),  # Must be UUID, not string
```

If it shows:
```python
"id": f"{chapter}_chunk_{chunk_id}",  # ❌ Old code
```

Update to:
```python
"id": str(uuid.uuid4()),  # ✅ Fixed
```

And add at top:
```python
import uuid
```

## Summary

After running `fix_qdrant_schema.py`:

✅ Collection schema valid (1536-dim, Cosine distance)
✅ All chunks have UUID IDs (Qdrant-compatible)
✅ Chapter metadata preserved
✅ RAG queries return relevant context
✅ Health endpoint reports "healthy"
✅ Ready for production use

The fix is complete and safe - your embeddings are now properly structured!
