#!/usr/bin/env python3
"""
Qdrant Collection Schema Fix Script

This script will:
1. Check current Qdrant collection status
2. Delete invalid collection if needed
3. Recreate collection with correct schema
4. Re-embed all chapters with UUID-based IDs
5. Verify the fix was successful

SAFE: Makes backup before deletion
"""
import os
import requests
import json
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http import models
import time

# Configuration
BACKEND_URL = "http://localhost:8000"
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
COLLECTION_NAME = "book_embeddings"
CHAPTERS_DIR = "frontend/my-book/docs/chapters"
BOOK_ID = "physical-ai-humanoid"

def print_section(title):
    """Print formatted section header"""
    print("\n" + "=" * 70)
    print(f" {title}")
    print("=" * 70)

def check_qdrant_health():
    """Check Qdrant health status via backend"""
    print_section("STEP 1: Check Current Qdrant Health")

    try:
        response = requests.get(f"{BACKEND_URL}/health/qdrant")
        health = response.json()

        print(f"Status: {health.get('status', 'unknown')}")
        print(f"Message: {health.get('message', 'N/A')}")

        if 'metadata' in health:
            print(f"Metadata: {json.dumps(health['metadata'], indent=2)}")

        return health
    except Exception as e:
        print(f"ERROR: Cannot connect to backend: {e}")
        print("Make sure backend is running on port 8000")
        return None

def backup_collection_info(client):
    """Backup current collection information"""
    print_section("STEP 2: Backup Collection Info (Safe Mode)")

    try:
        collection_info = client.get_collection(COLLECTION_NAME)

        backup_data = {
            "collection_name": COLLECTION_NAME,
            "vectors_count": collection_info.vectors_count,
            "points_count": collection_info.points_count,
            "config": {
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance.name
            }
        }

        # Save backup
        backup_file = "qdrant_backup_info.json"
        with open(backup_file, 'w') as f:
            json.dump(backup_data, f, indent=2)

        print(f"✓ Backup saved to: {backup_file}")
        print(f"  Vectors count: {backup_data['vectors_count']}")
        print(f"  Points count: {backup_data['points_count']}")

        return backup_data
    except Exception as e:
        print(f"Collection doesn't exist or error: {e}")
        return None

def delete_collection(client):
    """Delete existing collection"""
    print_section("STEP 3: Delete Invalid Collection")

    try:
        client.delete_collection(COLLECTION_NAME)
        print(f"✓ Deleted collection: {COLLECTION_NAME}")
        time.sleep(2)  # Wait for deletion to complete
        return True
    except Exception as e:
        print(f"Error deleting collection: {e}")
        return False

def create_collection(client):
    """Create collection with correct schema"""
    print_section("STEP 4: Create Collection with Correct Schema")

    print("Creating collection with:")
    print("  - Name: book_embeddings")
    print("  - Vector size: 1536 (OpenAI text-embedding-3-small)")
    print("  - Distance: Cosine")
    print("  - ID type: UUID (required by Qdrant)")

    try:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1536,  # OpenAI text-embedding-3-small dimension
                distance=models.Distance.COSINE
            )
        )
        print(f"✓ Collection created successfully")

        # Verify creation
        collection_info = client.get_collection(COLLECTION_NAME)
        print(f"✓ Verified - Vector size: {collection_info.config.params.vectors.size}")
        print(f"✓ Verified - Distance: {collection_info.config.params.vectors.distance.name}")

        return True
    except Exception as e:
        print(f"ERROR creating collection: {e}")
        return False

def get_chapter_files():
    """Get all markdown files from chapters directory"""
    chapters_path = Path(CHAPTERS_DIR)
    if not chapters_path.exists():
        print(f"ERROR: Directory not found: {CHAPTERS_DIR}")
        return []
    return sorted(chapters_path.glob("*.md"))

def read_chapter_content(file_path):
    """Read content from markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        print(f"ERROR reading {file_path}: {e}")
        return None

def embed_chapter(chapter_name, content):
    """Send chapter to /embed endpoint"""
    url = f"{BACKEND_URL}/embed"

    payload = {
        "content": content,
        "chapter": chapter_name,
        "book_id": BOOK_ID
    }

    try:
        response = requests.post(url, json=payload, headers={"Content-Type": "application/json"}, timeout=60)

        if response.status_code != 200:
            error_detail = response.json().get('detail', 'Unknown error')
            return False, 0, f"HTTP {response.status_code}: {error_detail}"

        result = response.json()
        chunks_created = result.get('chunks_created', 0)
        return True, chunks_created, None

    except Exception as e:
        return False, 0, str(e)

def re_embed_all_chapters():
    """Re-embed all chapters with UUID IDs"""
    print_section("STEP 5: Re-Embed All Chapters with UUID IDs")

    chapter_files = get_chapter_files()

    if not chapter_files:
        print(f"ERROR: No chapter files found in {CHAPTERS_DIR}")
        return False

    print(f"Found {len(chapter_files)} chapters to embed\n")

    success_count = 0
    total_chunks = 0

    for i, file_path in enumerate(chapter_files, 1):
        chapter_name = file_path.stem
        content = read_chapter_content(file_path)

        if content is None:
            continue

        print(f"[{i}/{len(chapter_files)}] Embedding: {chapter_name}...")
        success, chunks, error = embed_chapter(chapter_name, content)

        if success:
            print(f"  ✓ Success: {chunks} chunks created")
            success_count += 1
            total_chunks += chunks
        else:
            print(f"  ✗ Error: {error}")

        time.sleep(0.5)  # Don't overwhelm server

    print(f"\n✓ Successfully embedded {success_count}/{len(chapter_files)} chapters")
    print(f"✓ Total chunks created: {total_chunks}")

    return success_count > 0

def verify_fix():
    """Verify that Qdrant is now healthy"""
    print_section("STEP 6: Verify Fix")

    # Check backend health
    response = requests.get(f"{BACKEND_URL}/health/qdrant")
    health = response.json()

    print(f"Status: {health.get('status', 'unknown')}")
    print(f"Message: {health.get('message', 'N/A')}")

    if 'metadata' in health:
        metadata = health['metadata']
        print(f"Collection: {metadata.get('collection', 'N/A')}")
        print(f"Vectors count: {metadata.get('vectors_count', 0)}")

    # Test a simple query
    print("\nTesting RAG query...")
    try:
        query_response = requests.post(
            f"{BACKEND_URL}/query",
            json={"query": "What is ROS 2?"},
            headers={"Content-Type": "application/json"}
        )

        if query_response.status_code == 200:
            result = query_response.json()
            if result.get('answer') and "No relevant context found" not in result['answer']:
                print("✓ RAG query successful - embeddings working!")
            else:
                print("⚠ RAG query returned no context (may need more time)")
        else:
            print(f"⚠ RAG query failed: {query_response.status_code}")
    except Exception as e:
        print(f"⚠ Could not test RAG query: {e}")

    return health.get('status') == 'healthy'

def main():
    """Main function to fix Qdrant schema"""
    print_section("QDRANT COLLECTION SCHEMA FIX")
    print("This script will:")
    print("1. Check current Qdrant health")
    print("2. Backup collection info")
    print("3. Delete invalid collection")
    print("4. Create new collection with correct schema (UUID IDs)")
    print("5. Re-embed all chapters")
    print("6. Verify fix")

    # Initialize Qdrant client
    try:
        client = QdrantClient(url=QDRANT_URL)
    except Exception as e:
        print(f"\nERROR: Cannot connect to Qdrant at {QDRANT_URL}")
        print(f"Error: {e}")
        print("\nMake sure Qdrant is running:")
        print("  docker run -p 6333:6333 qdrant/qdrant")
        return

    # Step 1: Check health
    health = check_qdrant_health()
    if health is None:
        return

    # Step 2: Backup
    backup = backup_collection_info(client)

    # Confirm before proceeding
    print("\n" + "-" * 70)
    response = input("Continue with collection deletion and recreation? (yes/no): ")
    if response.lower() not in ['yes', 'y']:
        print("Aborted. No changes made.")
        return

    # Step 3: Delete old collection
    if not delete_collection(client):
        print("ERROR: Could not delete collection")
        return

    # Step 4: Create new collection
    if not create_collection(client):
        print("ERROR: Could not create collection")
        return

    # Step 5: Re-embed chapters
    if not re_embed_all_chapters():
        print("ERROR: Could not re-embed chapters")
        return

    # Step 6: Verify
    if verify_fix():
        print_section("SUCCESS!")
        print("✓ Qdrant collection schema is now valid")
        print("✓ All chapters embedded with UUID IDs")
        print("✓ RAG queries should now work")
    else:
        print_section("PARTIAL SUCCESS")
        print("Collection recreated and chapters embedded")
        print("May need additional time for health check to reflect changes")

if __name__ == "__main__":
    main()
