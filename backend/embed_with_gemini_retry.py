"""
Automated embedding script with Gemini API quota retry logic

This script will:
1. Check if Gemini API quota is available
2. If quota exhausted, provide next retry time
3. If quota available, embed all content automatically
4. Verify embeddings and confirm chatbot is ready

Usage:
    python embed_with_gemini_retry.py
"""
import sys
import os
from pathlib import Path
from datetime import datetime, timedelta
import time

# Force UTF-8 encoding for Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from dotenv import load_dotenv
load_dotenv()

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import google.generativeai as genai
import src.qdrant_manager
import src.embed

print("=" * 80)
print("Gemini API Embedding Script with Quota Retry")
print("=" * 80)
print(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
COLLECTION_NAME = "book_embeddings"

# Validate environment variables
print("\n[Step 1] Validating environment variables...")
if not QDRANT_URL:
    print("✗ QDRANT_URL not set in .env")
    sys.exit(1)
if not QDRANT_API_KEY:
    print("✗ QDRANT_API_KEY not set in .env")
    sys.exit(1)
if not GOOGLE_API_KEY:
    print("✗ GOOGLE_API_KEY not set in .env")
    sys.exit(1)

print(f"✓ QDRANT_URL: {QDRANT_URL}")
print(f"✓ QDRANT_API_KEY: ***{QDRANT_API_KEY[-10:]}")
print(f"✓ GOOGLE_API_KEY: ***{GOOGLE_API_KEY[-10:]}")

# Test Gemini API quota
print("\n[Step 2] Testing Gemini API quota availability...")
genai.configure(api_key=GOOGLE_API_KEY)

try:
    # Try a minimal embedding to check quota
    test_result = genai.embed_content(
        model="models/embedding-001",
        content="test",
        task_type="retrieval_document"
    )
    print("✓ Gemini API quota is available!")
    quota_available = True
except Exception as e:
    error_str = str(e)
    if "429" in error_str or "quota" in error_str.lower():
        print(f"✗ Gemini API quota exhausted")
        print(f"  Error: {error_str[:200]}...")
        print("\n" + "=" * 80)
        print("QUOTA EXHAUSTED - NEXT STEPS")
        print("=" * 80)
        print("\nGoogle Gemini API free tier quota resets daily.")
        print("Estimated next reset: Tonight at midnight UTC")
        print("\nOptions:")
        print("  1. Run this script again tomorrow (quota will reset)")
        print("  2. Enable billing on Google Cloud for higher quotas")
        print("     URL: https://console.cloud.google.com/billing")
        print("\nTo retry later, simply run:")
        print("  cd backend && python embed_with_gemini_retry.py")
        print("=" * 80)
        sys.exit(1)
    else:
        print(f"✗ Unexpected error testing Gemini API: {e}")
        sys.exit(1)

# Connect to Qdrant
print("\n[Step 3] Connecting to Qdrant...")
try:
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    collections = client.get_collections()
    print(f"✓ Connected to Qdrant")
    print(f"  Existing collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"✗ Failed to connect to Qdrant: {e}")
    sys.exit(1)

# Delete and recreate collection
print(f"\n[Step 4] Managing collection '{COLLECTION_NAME}'...")
try:
    existing_collections = [c.name for c in client.get_collections().collections]
    if COLLECTION_NAME in existing_collections:
        print(f"  Deleting existing collection '{COLLECTION_NAME}'...")
        client.delete_collection(COLLECTION_NAME)
        print(f"  ✓ Deleted")

    print(f"  Creating collection with VectorParams...")
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=768,  # Gemini embedding-001 uses 768 dimensions
            distance=Distance.COSINE
        )
    )
    print(f"✓ Collection '{COLLECTION_NAME}' ready")
    print(f"  Vector size: 768 (Gemini embedding-001)")
    print(f"  Distance: COSINE")
except Exception as e:
    print(f"✗ Collection management failed: {e}")
    sys.exit(1)

# Find book content
print("\n[Step 5] Finding book content...")
book_dir = Path(__file__).parent.parent / "frontend" / "my-book" / "docs"

if not book_dir.exists():
    print(f"✗ Book directory not found: {book_dir}")
    sys.exit(1)

content_files = sorted(list(book_dir.glob("*.txt")) + list(book_dir.glob("*.md")))
print(f"✓ Found {len(content_files)} files in {book_dir.name}")
for f in content_files[:10]:  # Show first 10
    print(f"    - {f.name}")
if len(content_files) > 10:
    print(f"    ... and {len(content_files) - 10} more")

# Initialize embedding generator
print("\n[Step 6] Initializing embedding generator...")
try:
    emb_generator = src.embed.EmbeddingGenerator()
    if emb_generator.provider != "google":
        print(f"⚠ Warning: Embedding generator is using '{emb_generator.provider}' instead of 'google'")
        print("  This script is designed for Gemini API only")
        print("  Check backend/src/embed.py provider priority")
    else:
        print(f"✓ Embedding generator initialized with Google Gemini")
except Exception as e:
    print(f"✗ Failed to initialize embedding generator: {e}")
    sys.exit(1)

# Embed all content
print("\n[Step 7] Embedding all book content...")
print(f"Processing {len(content_files)} files...")

total_chunks = 0
failed_files = []

for idx, file_path in enumerate(content_files, 1):
    print(f"\n  [{idx}/{len(content_files)}] {file_path.name}")

    try:
        # Read content
        content = file_path.read_text(encoding='utf-8')
        chapter = file_path.stem.replace('_', ' ').title()

        # Chunk content
        chunks = emb_generator.chunk_text(content, chapter)
        print(f"    Created {len(chunks)} chunks")

        # Generate embeddings
        embeddings = []
        for chunk_idx, chunk in enumerate(chunks, 1):
            try:
                vector = emb_generator.generate_embedding(chunk["text"])
                embeddings.append({
                    "id": chunk["id"],
                    "vector": vector,
                    "chapter": chunk["chapter"],
                    "text": chunk["text"],
                    "token_count": chunk["token_count"]
                })
                print(f"      Chunk {chunk_idx}/{len(chunks)}: ✓ embedded")
            except Exception as e:
                if "429" in str(e) or "quota" in str(e).lower():
                    print(f"      Chunk {chunk_idx}/{len(chunks)}: ✗ quota exhausted mid-process")
                    print(f"\n⚠ Gemini API quota exhausted during embedding!")
                    print(f"   Successfully embedded {total_chunks} chunks before quota limit")
                    print(f"   Run this script again tomorrow to complete remaining files")
                    sys.exit(1)
                else:
                    print(f"      Chunk {chunk_idx}/{len(chunks)}: ✗ error: {e}")
                    raise

        # Insert into Qdrant
        if embeddings:
            manager = src.qdrant_manager.QdrantManager()
            manager.insert_embeddings(embeddings)
            total_chunks += len(embeddings)
            print(f"    ✓ Inserted {len(embeddings)} embeddings (total: {total_chunks})")

    except Exception as e:
        print(f"    ✗ Failed: {e}")
        failed_files.append(file_path.name)
        continue

# Verify embeddings
print("\n[Step 8] Verifying embeddings...")
try:
    info = client.get_collection(COLLECTION_NAME)
    points_count = info.points_count

    print(f"✓ Collection verification:")
    print(f"    Collection: {COLLECTION_NAME}")
    print(f"    Points count: {points_count}")
    print(f"    Vector size: {info.config.params.vectors.size}")
    print(f"    Distance: {info.config.params.vectors.distance.name}")

    if points_count == 0:
        print(f"\n⚠ Warning: No points in collection!")
        print(f"  This means no embeddings were successfully inserted")
        sys.exit(1)

    # Test search
    print(f"\n  Testing search functionality...")
    test_vector = emb_generator.generate_embedding("What is this book about?")
    results = client.search(
        collection_name=COLLECTION_NAME,
        query_vector=test_vector,
        limit=3
    )

    print(f"  Search test: Found {len(results)} results")
    if results:
        print(f"    Top result score: {results[0].score:.4f}")
        print(f"    Top result chapter: {results[0].payload.get('chapter', 'Unknown')}")

except Exception as e:
    print(f"✗ Verification failed: {e}")
    sys.exit(1)

# Final summary
print("\n" + "=" * 80)
print("✅ EMBEDDING COMPLETED SUCCESSFULLY")
print("=" * 80)

print(f"\nSummary:")
print(f"  Files processed: {len(content_files) - len(failed_files)}/{len(content_files)}")
print(f"  Total chunks embedded: {total_chunks}")
print(f"  Collection: {COLLECTION_NAME}")
print(f"  Points in collection: {points_count}")
print(f"  Provider: Google Gemini (embedding-001)")

if failed_files:
    print(f"\n⚠ Failed files ({len(failed_files)}):")
    for f in failed_files:
        print(f"    - {f}")

print(f"\n✅ Chatbot is ready for queries!")
print(f"\nNext steps:")
print(f"  1. Start backend: cd backend && python -m uvicorn src.main:app --reload")
print(f"  2. Test query: curl -X POST http://localhost:8000/query -H 'Content-Type: application/json' -d '{{\"query\": \"What is this book about?\"}}'")
print(f"  3. Frontend should now show real responses (no infinite buffering)")

print("\n" + "=" * 80)
print(f"Completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
print("=" * 80)
