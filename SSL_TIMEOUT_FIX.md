# SSL Handshake Timeout Fix - Qdrant Cloud

## Problem
```
_ssl.c:989: The handshake operation timed out
```

## Solution Applied

### Step 1: Update `backend/src/qdrant_manager.py`

**Line 37-43 (Cloud Connection):**
```python
self.client = QdrantClient(
    url=url,
    api_key=api_key,
    timeout=60,  # Increase timeout for cloud SSL handshake
    https=True,  # Enforce HTTPS
    prefer_grpc=False  # Use HTTP for better cloud compatibility
)
```

**Key Changes:**
1. `timeout=60` - Increased from default 5s to 60s
2. `https=True` - Explicitly enforce HTTPS
3. `prefer_grpc=False` - Use REST API instead of gRPC

### Step 2: Restart Backend

```bash
cd backend
python -m src.main
```

### Step 3: Verify Connection

```bash
curl http://localhost:8000/health/qdrant
```

**Expected:**
```json
{
  "status": "healthy",
  "message": "Qdrant connection successful"
}
```

## Alternative: Manual httpx Configuration

If issue persists, add to `backend/src/qdrant_manager.py`:

```python
import httpx

# At top of __init__ method
http_client = httpx.Client(
    timeout=httpx.Timeout(60.0, connect=60.0),
    limits=httpx.Limits(max_connections=100, max_keepalive_connections=20),
    verify=True
)

self.client = QdrantClient(
    url=url,
    api_key=api_key,
    timeout=60,
    https=True,
    prefer_grpc=False,
    http_client=http_client  # Custom HTTP client
)
```

## Test Script

```python
# test_qdrant_connection.py
import os
from qdrant_client import QdrantClient

url = os.getenv("QDRANT_URL")
api_key = os.getenv("QDRANT_API_KEY")

client = QdrantClient(
    url=url,
    api_key=api_key,
    timeout=60,
    https=True,
    prefer_grpc=False
)

collections = client.get_collections()
print(f"✓ Connected to Qdrant cloud")
print(f"Collections: {collections}")
```

Run:
```bash
python test_qdrant_connection.py
```

## Environment Variables

Ensure `.env` has:
```env
QDRANT_URL=https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your-api-key-here
```

## Troubleshooting

### Still Timing Out?

**Option 1: Increase Timeout Further**
```python
timeout=120  # 2 minutes
```

**Option 2: Check Network**
```bash
curl -I https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io
```

**Option 3: Use Async Client**
```python
from qdrant_client import AsyncQdrantClient

self.client = await AsyncQdrantClient(
    url=url,
    api_key=api_key,
    timeout=60,
    https=True,
    prefer_grpc=False
)
```

### Firewall/Proxy Issues

If behind corporate firewall:
```python
import httpx

proxies = {
    "http://": "http://proxy.company.com:8080",
    "https://": "http://proxy.company.com:8080"
}

http_client = httpx.Client(
    timeout=60.0,
    proxies=proxies,
    verify=True
)

self.client = QdrantClient(
    url=url,
    api_key=api_key,
    http_client=http_client
)
```

## Summary

✅ Timeout increased from 5s → 60s
✅ HTTPS enforced
✅ gRPC disabled for REST API
✅ Backend should start without SSL timeout
✅ Cloud connection stable
