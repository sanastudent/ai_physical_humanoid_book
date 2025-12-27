# Embed All Chapters - Quick Start Guide

## Step 1: Start Backend Server

```bash
cd backend
python -m src.main
```

Server runs on `http://localhost:8000`

## Step 2: Run Embedding Script

### Option A: Python Script

```bash
# Install requests if needed
pip install requests

# Run script
python embed_all_chapters.py
```

### Option B: PowerShell Script

```powershell
# Run script
powershell -ExecutionPolicy Bypass -File embed_all_chapters.ps1
```

## Expected Output

```
============================================================
EMBEDDING ALL CHAPTERS
============================================================
Backend: http://localhost:8000
Chapters: frontend\my-book\docs\chapters
Book ID: physical-ai-humanoid
============================================================

Found 16 chapters

Embedding: module1-architecture...
  ✓ Success: 12 chunks created
Embedding: module1-communication...
  ✓ Success: 15 chunks created
Embedding: module1-intro...
  ✓ Success: 18 chunks created
...

============================================================
SUMMARY
============================================================
Total chapters: 16
Successful: 16
Failed: 0
============================================================
```

## Endpoint Format

**URL:** `POST http://localhost:8000/embed`

**Headers:**
```
Content-Type: application/json
```

**Body:**
```json
{
  "content": "# Chapter Title\n\nChapter content here...",
  "chapter": "module1-intro",
  "book_id": "physical-ai-humanoid"
}
```

**Response:**
```json
{
  "status": "success",
  "chapter": "module1-intro",
  "chunks_created": 18
}
```

## Manual Test (Single Chapter)

```bash
curl -X POST http://localhost:8000/embed \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Test Chapter\n\nThis is test content.",
    "chapter": "test-chapter",
    "book_id": "physical-ai-humanoid"
  }'
```

## Configuration

Edit variables at top of script:

**Python (`embed_all_chapters.py`):**
```python
BACKEND_URL = "http://localhost:8000"
CHAPTERS_DIR = "frontend/my-book/docs/chapters"
BOOK_ID = "physical-ai-humanoid"
```

**PowerShell (`embed_all_chapters.ps1`):**
```powershell
$BackendUrl = "http://localhost:8000"
$ChaptersDir = "frontend\my-book\docs\chapters"
$BookId = "physical-ai-humanoid"
```

## Troubleshooting

### Error: "Connection refused"
- Start backend server first
- Check server is running on port 8000

### Error: "Field required"
- Ensure all 3 fields present: `content`, `chapter`, `book_id`
- Check JSON format is correct

### Error: "No chapter files found"
- Verify `CHAPTERS_DIR` path is correct
- Check `.md` files exist in directory

### Verify Embeddings

Check Qdrant collection:
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
    "vectors_count": 250
  }
}
```

## Schema Reference

**EmbedRequest Model:**
```python
class EmbedRequest(BaseModel):
    content: str          # Chapter markdown content
    chapter: str          # Chapter identifier (e.g., "module1-intro")
    book_id: Optional[str] = None  # Book identifier
```

**Stored in Qdrant:**
- Collection: `book_embeddings`
- Vector size: 1536
- Distance: Cosine
- Payload: `id`, `chapter`, `text`, `token_count`, `book_id`
