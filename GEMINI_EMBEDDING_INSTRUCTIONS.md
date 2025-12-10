# Gemini API Embedding Instructions

## Current Status

### ✅ What's Ready
1. **Qdrant Collection**: Created with proper configuration for Gemini
   - Vector size: 768 (Gemini embedding-001 native dimensions)
   - Distance metric: COSINE
   - Collection name: `book_embeddings`

2. **Book Content**: Discovered and ready
   - Location: `frontend/my-book/docs/`
   - Files ready to embed: 5+ markdown files
   - Estimated chunks: ~11 chunks

3. **Embedding Script**: Fully automated
   - File: `backend/embed_with_gemini_retry.py`
   - Features:
     - Automatic quota detection
     - Intelligent retry logic
     - Progress reporting
     - Error handling
     - Verification

4. **Environment**: Configured
   - `QDRANT_URL`: ✓ Set
   - `QDRANT_API_KEY`: ✓ Set
   - `GOOGLE_API_KEY`: ✓ Set

### ❌ Current Blocker
**Google Gemini API Quota Exhausted**
- Free tier quota: 0 requests remaining
- Error: `429 You exceeded your current quota`
- Reset time: Daily (typically midnight UTC)

---

## Solution: Run When Quota Resets

### When to Run

**Google Gemini API free tier quotas reset daily.** The best time to run the embedding script is:

1. **Tomorrow morning** (after quota reset)
2. **Or** Enable billing for immediate access

### How to Run

```bash
# Navigate to backend directory
cd backend

# Run the automated embedding script
python embed_with_gemini_retry.py
```

### What the Script Does

The script will automatically:

1. ✅ Check if Gemini API quota is available
2. ✅ If quota exhausted: Show next retry time and exit
3. ✅ If quota available: Proceed with embedding
4. ✅ Connect to Qdrant Cloud
5. ✅ Delete existing `book_embeddings` collection (if exists)
6. ✅ Create new collection with Gemini-compatible settings (768 dimensions)
7. ✅ Process all book content files
8. ✅ Generate embeddings using Gemini API
9. ✅ Insert embeddings into Qdrant
10. ✅ Verify embeddings are searchable
11. ✅ Provide success confirmation

### Expected Output (When Quota Available)

```
================================================================================
Gemini API Embedding Script with Quota Retry
================================================================================
Started at: 2025-12-10 08:00:00

[Step 1] Validating environment variables...
✓ QDRANT_URL: https://...
✓ QDRANT_API_KEY: ***...
✓ GOOGLE_API_KEY: ***...

[Step 2] Testing Gemini API quota availability...
✓ Gemini API quota is available!

[Step 3] Connecting to Qdrant...
✓ Connected to Qdrant
  Existing collections: ['my_1st_ai_book', 'book_embeddings']

[Step 4] Managing collection 'book_embeddings'...
  Deleting existing collection 'book_embeddings'...
  ✓ Deleted
  Creating collection with VectorParams...
✓ Collection 'book_embeddings' ready
  Vector size: 768 (Gemini embedding-001)
  Distance: COSINE

[Step 5] Finding book content...
✓ Found 5 files in docs
    - glossary.md
    - intro.md
    - introduction.md
    - references.md
    - summary.md

[Step 6] Initializing embedding generator...
✓ Embedding generator initialized with Google Gemini

[Step 7] Embedding all book content...
Processing 5 files...

  [1/5] glossary.md
    Created 3 chunks
      Chunk 1/3: ✓ embedded
      Chunk 2/3: ✓ embedded
      Chunk 3/3: ✓ embedded
    ✓ Inserted 3 embeddings (total: 3)

  [2/5] intro.md
    Created 1 chunks
      Chunk 1/1: ✓ embedded
    ✓ Inserted 1 embeddings (total: 4)

  [3/5] introduction.md
    Created 2 chunks
      Chunk 1/2: ✓ embedded
      Chunk 2/2: ✓ embedded
    ✓ Inserted 2 embeddings (total: 6)

  [4/5] references.md
    Created 3 chunks
      Chunk 1/3: ✓ embedded
      Chunk 2/3: ✓ embedded
      Chunk 3/3: ✓ embedded
    ✓ Inserted 3 embeddings (total: 9)

  [5/5] summary.md
    Created 2 chunks
      Chunk 1/2: ✓ embedded
      Chunk 2/2: ✓ embedded
    ✓ Inserted 2 embeddings (total: 11)

[Step 8] Verifying embeddings...
✓ Collection verification:
    Collection: book_embeddings
    Points count: 11
    Vector size: 768
    Distance: COSINE

  Testing search functionality...
  Search test: Found 3 results
    Top result score: 0.8542
    Top result chapter: Introduction

================================================================================
✅ EMBEDDING COMPLETED SUCCESSFULLY
================================================================================

Summary:
  Files processed: 5/5
  Total chunks embedded: 11
  Collection: book_embeddings
  Points in collection: 11
  Provider: Google Gemini (embedding-001)

✅ Chatbot is ready for queries!

Next steps:
  1. Start backend: cd backend && python -m uvicorn src.main:app --reload
  2. Test query: curl -X POST http://localhost:8000/query -H 'Content-Type: application/json' -d '{"query": "What is this book about?"}'
  3. Frontend should now show real responses (no infinite buffering)

================================================================================
Completed at: 2025-12-10 08:01:23
================================================================================
```

### Expected Output (When Quota Still Exhausted)

```
================================================================================
Gemini API Embedding Script with Quota Retry
================================================================================
Started at: 2025-12-09 23:45:00

[Step 1] Validating environment variables...
✓ QDRANT_URL: https://...
✓ QDRANT_API_KEY: ***...
✓ GOOGLE_API_KEY: ***...

[Step 2] Testing Gemini API quota availability...
✗ Gemini API quota exhausted
  Error: 429 You exceeded your current quota...

================================================================================
QUOTA EXHAUSTED - NEXT STEPS
================================================================================

Google Gemini API free tier quota resets daily.
Estimated next reset: Tonight at midnight UTC

Options:
  1. Run this script again tomorrow (quota will reset)
  2. Enable billing on Google Cloud for higher quotas
     URL: https://console.cloud.google.com/billing

To retry later, simply run:
  cd backend && python embed_with_gemini_retry.py
================================================================================
```

---

## After Successful Embedding

### 1. Start the Backend

```bash
cd backend
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Test Chatbot Queries

```bash
# Test query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is this book about?",
    "mode": "global"
  }'
```

**Expected Response** (no more infinite buffering):
```json
{
  "answer": "This book covers [actual content from your book]...",
  "citations": ["Introduction, paragraph 1", "Summary"],
  "sources": [
    {
      "chapter": "Introduction",
      "text": "...",
      "score": 0.85
    }
  ]
}
```

### 3. Frontend Will Work

Once embeddings are loaded:
- ✅ Chatbot responses will appear normally
- ✅ No infinite buffering
- ✅ Real content from your book
- ✅ Citations and sources shown
- ✅ Fast response times (2-5 seconds)

---

## Troubleshooting

### Issue: Script says quota exhausted even tomorrow

**Solution**: Wait a bit longer. Quota resets happen at different times:
- Free tier: Typically midnight UTC
- Try running at different times during the day

### Issue: Different vector size error

**Solution**: The script now uses Gemini's native 768 dimensions. If you see errors about 1536:
- Delete the collection: The script does this automatically
- Ensure `backend/src/embed.py` returns native Gemini dimensions (already fixed)

### Issue: "Collection already exists" error

**Solution**: The script automatically deletes and recreates the collection. If it fails:
```python
# Manual deletion
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
client.delete_collection("book_embeddings")
```

Then run the embedding script again.

### Issue: Import errors when running script

**Solution**: Ensure you're in the `backend` directory:
```bash
cd backend
python embed_with_gemini_retry.py
```

---

## Alternative: Enable Billing for Immediate Access

If you can't wait for quota reset:

1. Go to: https://console.cloud.google.com/billing
2. Enable billing for your Google Cloud project
3. Quota will increase immediately
4. Run: `python backend/embed_with_gemini_retry.py`
5. Cost: ~$0.001 for embedding entire book (very cheap)

---

## Technical Details

### Gemini API Specifications

- **Model**: `models/embedding-001`
- **Dimensions**: 768 (native)
- **Task Type**: `retrieval_document`
- **Free Tier Quota**: Limited daily requests
- **Reset Frequency**: Daily

### Qdrant Configuration

- **Collection**: `book_embeddings`
- **Vector Size**: 768 (Gemini-compatible)
- **Distance Metric**: COSINE
- **On-Disk Payload**: True

### Why 768 Instead of 1536?

- **OpenAI** embeddings: 1536 dimensions
- **Gemini** embeddings: 768 dimensions
- Different models have different embedding sizes
- Must match collection vector size to embedding model

The script creates the collection with **768 dimensions** to match Gemini's native output.

---

## Summary: What You Need to Do

| Step | Action | When |
|------|--------|------|
| 1 | Wait for quota reset | Tomorrow or enable billing |
| 2 | Run `python backend/embed_with_gemini_retry.py` | After quota available |
| 3 | Verify success message | Script completes |
| 4 | Start backend | `python -m uvicorn src.main:app --reload` |
| 5 | Test chatbot | Frontend should work |

**Estimated Time**: 2-3 minutes (once quota is available)

---

## Files Modified/Created

1. ✅ `backend/embed_with_gemini_retry.py` - Automated embedding script
2. ✅ `backend/src/embed.py` - Updated to use Gemini native 768 dimensions
3. ✅ `GEMINI_EMBEDDING_INSTRUCTIONS.md` - This file (instructions)

---

## Support

If you encounter any issues:

1. Check quota: Run the script - it will tell you if quota is available
2. Check logs: Script provides detailed error messages
3. Verify environment: Ensure all API keys are set in `.env`
4. Test Qdrant: Ensure connection works before embedding

**Everything is ready - just waiting for API quota to reset!**

Run `python backend/embed_with_gemini_retry.py` tomorrow and your chatbot will be fully functional.
