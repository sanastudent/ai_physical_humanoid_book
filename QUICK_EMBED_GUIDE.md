# Quick Embedding Guide

## Problem Fixed

❌ **Before:** `"id": "module1-intro_chunk_0"` → Qdrant error
✅ **After:** `"id": "a3bb189e-8bf9-4e57-a7f8-90ee0ff19b98"` → Works!

## What Changed

**File:** `backend/src/embed.py` line 65
```python
# OLD
"id": f"{chapter}_chunk_{chunk_id}"

# NEW
"id": str(uuid.uuid4())
```

## Run Embedding

```bash
# 1. Start backend
cd backend
python -m src.main

# 2. Run fixed script
cd ..
python embed_chapters_fixed.py
```

## Verify

```bash
curl http://localhost:8000/health/qdrant
```

Expected: `"vectors_count": 245` (or similar number > 0)

## Test RAG

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

Should return answer with chapter citations.

## Why UUIDs?

Qdrant **requires** point IDs to be:
- Unsigned integers (1, 2, 3...), OR
- UUIDs (formatted as `xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx`)

String IDs like `"module1_chunk_0"` are **invalid**.

## What's Preserved?

Even with UUID IDs, all metadata is stored:
- `chapter`: Chapter name (e.g., "module1-intro")
- `text`: Chunk content
- `token_count`: Number of tokens
- `book_id`: Book identifier

RAG queries can still filter/retrieve by chapter name!
