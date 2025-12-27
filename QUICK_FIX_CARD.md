# Quick Fix Card - Qdrant Schema

## Problem
```
status: degraded
message: "Qdrant connected but collection schema invalid"
```

## Solution

### Python (Recommended)
```bash
python fix_qdrant_schema.py
```

### PowerShell
```powershell
powershell -ExecutionPolicy Bypass -File fix_qdrant_schema.ps1
```

## What It Does

1. ✅ Checks current status
2. ✅ Backs up collection info
3. ✅ Deletes invalid collection (with confirmation)
4. ✅ Creates new collection (1536-dim, Cosine, UUID IDs)
5. ✅ Re-embeds all 16 chapters
6. ✅ Verifies fix

## Prerequisites

**Backend running:**
```bash
cd backend
python -m src.main
```

**Qdrant running:**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

## After Fix

**Verify:**
```bash
curl http://localhost:8000/health/qdrant
```

**Expected:**
```json
{
  "status": "healthy",
  "vectors_count": 245
}
```

**Test RAG:**
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

Should return answer with chapter citations (not "No relevant context found").

## Files
- `fix_qdrant_schema.py` - Automated Python script
- `fix_qdrant_schema.ps1` - PowerShell version
- `QDRANT_SCHEMA_FIX_GUIDE.md` - Detailed guide
- `qdrant_backup_info.json` - Auto-generated backup

## Time
~2-3 minutes to complete all steps
