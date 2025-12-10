"""
Test script to verify Qdrant Cloud connection and collection creation

This script tests:
1. Environment variables are loaded correctly
2. Qdrant client connects successfully with API key
3. Collection can be created with Distance.COSINE enum
"""
import sys
import os

# Force UTF-8 encoding for Windows console
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

print("=" * 70)
print("Qdrant Cloud Connection Test")
print("=" * 70)

# Test 1: Check environment variables
print("\n[Test 1] Checking environment variables...")
from dotenv import load_dotenv
load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

print(f"  QDRANT_URL: {qdrant_url}")
print(f"  QDRANT_API_KEY: {'***' + qdrant_api_key[-10:] if qdrant_api_key else 'NOT SET'}")

if not qdrant_url:
    print("✗ QDRANT_URL is not set!")
    sys.exit(1)
if not qdrant_api_key:
    print("✗ QDRANT_API_KEY is not set!")
    sys.exit(1)

print("✓ Environment variables loaded successfully")

# Test 2: Import QdrantManager
print("\n[Test 2] Importing QdrantManager...")
try:
    from qdrant_manager import QdrantManager
    print("✓ QdrantManager imported successfully")
except Exception as e:
    print(f"✗ Failed to import QdrantManager: {e}")
    sys.exit(1)

# Test 3: Initialize Qdrant client
print("\n[Test 3] Initializing Qdrant client...")
try:
    manager = QdrantManager()
    print("✓ Qdrant client initialized successfully")
except Exception as e:
    print(f"✗ Failed to initialize Qdrant client: {e}")
    print(f"  Error details: {type(e).__name__}: {str(e)}")
    sys.exit(1)

# Test 4: Test connection (list collections)
print("\n[Test 4] Testing connection (listing collections)...")
try:
    collections = manager.client.get_collections()
    collection_names = [col.name for col in collections.collections]
    print(f"✓ Connected to Qdrant successfully!")
    print(f"  Found {len(collection_names)} collections: {collection_names}")
except Exception as e:
    print(f"✗ Failed to connect to Qdrant: {e}")
    print(f"  Error type: {type(e).__name__}")
    if "403" in str(e) or "Forbidden" in str(e):
        print("  → This is a 403 Forbidden error - check your API key")
    sys.exit(1)

# Test 5: Create or verify collection
print("\n[Test 5] Creating/verifying collection...")
try:
    manager.create_collection()
    print("✓ Collection created/verified successfully")

    # Get collection info
    info = manager.get_collection_info()
    print(f"  Collection: {info.config.params.vectors}")
    print(f"  Vector size: {info.config.params.vectors.size}")
    print(f"  Distance metric: {info.config.params.vectors.distance.name}")
    print(f"  Points count: {info.points_count}")
except Exception as e:
    print(f"✗ Failed to create/verify collection: {e}")
    print(f"  Error type: {type(e).__name__}")
    if "Distance" in str(e):
        print("  → This is a Distance enum error - check Distance.COSINE usage")
    sys.exit(1)

print("\n" + "=" * 70)
print("✅ ALL TESTS PASSED - Qdrant Cloud connection is working!")
print("=" * 70)
print("\nNext steps:")
print("  1. Embed book content using the /embed endpoint")
print("  2. Test chatbot queries using the /query endpoint")
print("  3. Verify responses are not buffering infinitely")
