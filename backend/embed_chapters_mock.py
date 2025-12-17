"""
Embed Book Chapters to Qdrant Collection (MOCK VERSION - For Testing)

This script uses mock embeddings instead of OpenAI API to avoid quota issues.
For production, use embed_chapters_to_collection.py with valid OpenAI API key.

This script:
1. Reads book chapters from frontend/my-book/docs/chapters/*.md
2. Generates MOCK embeddings (1536-dim random vectors for testing)
3. Inserts embeddings into Qdrant cloud collection via REST API
4. Verifies collection has points
5. Returns JSON summary
"""

import os
import sys
import uuid
import json
import requests
import random
from pathlib import Path
from dotenv import load_dotenv
import logging

# Load environment variables from root directory
REPO_ROOT_FOR_ENV = Path(__file__).parent.parent
load_dotenv(REPO_ROOT_FOR_ENV / ".env")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "my_1st_ai_book"
EMBEDDING_DIMENSIONS = 1536

# Paths
REPO_ROOT = Path(__file__).parent.parent
CHAPTERS_DIR = REPO_ROOT / "frontend" / "my-book" / "docs" / "chapters"


def validate_environment():
    """Validate required environment variables"""
    missing = []
    if not QDRANT_URL:
        missing.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY")

    if missing:
        logger.error(f"Missing environment variables: {', '.join(missing)}")
        sys.exit(1)

    logger.info("✓ Environment variables validated")


def get_qdrant_base_url():
    """Get Qdrant base URL with port"""
    url = QDRANT_URL.rstrip('/')
    if not url.endswith(':6333'):
        url += ':6333'
    return url


def get_qdrant_headers():
    """Get headers for Qdrant API requests"""
    return {
        "api-key": QDRANT_API_KEY,
        "Content-Type": "application/json"
    }


def read_chapters(limit=None):
    """Read chapter files"""
    if not CHAPTERS_DIR.exists():
        logger.error(f"Chapters directory not found: {CHAPTERS_DIR}")
        sys.exit(1)

    chapter_files = sorted(CHAPTERS_DIR.glob("*.md"))

    if limit:
        chapter_files = chapter_files[:limit]

    if not chapter_files:
        logger.error(f"No chapter files found in {CHAPTERS_DIR}")
        sys.exit(1)

    logger.info(f"Found {len(chapter_files)} chapter files")

    chapters = []
    for file_path in chapter_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            chapter_name = file_path.stem
            chapters.append({
                "name": chapter_name,
                "content": content,
                "file": str(file_path)
            })
            logger.info(f"  ✓ Read chapter: {chapter_name}")

        except Exception as e:
            logger.warning(f"  ✗ Failed to read {file_path.name}: {e}")
            continue

    return chapters


def generate_mock_embedding():
    """Generate mock embedding (random 1536-dim vector)"""
    # Generate random vector with values roughly in the range of text-embedding-3-small
    return [random.uniform(-1.0, 1.0) for _ in range(EMBEDDING_DIMENSIONS)]


def insert_points_to_qdrant(points):
    """Insert points into Qdrant collection via REST API"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}/points"

    payload = {
        "points": points
    }

    try:
        response = requests.put(
            url,
            headers=get_qdrant_headers(),
            json=payload,
            timeout=60
        )

        if response.status_code in [200, 201]:
            return True, "Success"
        else:
            return False, f"{response.status_code}: {response.text}"

    except Exception as e:
        return False, str(e)


def get_collection_info():
    """Get collection information from Qdrant"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}"

    try:
        response = requests.get(
            url,
            headers=get_qdrant_headers(),
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()
            return data.get('result', {})
        else:
            logger.error(f"Failed to get collection info: {response.status_code}")
            return None

    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        return None


def embed_and_insert_chapters():
    """Main function to embed chapters and insert into Qdrant"""
    logger.info(f"\n{'='*70}")
    logger.info("Embedding Chapters to Qdrant Collection (MOCK MODE)")
    logger.info(f"{'='*70}\n")
    logger.info("⚠ Using MOCK embeddings (random vectors) for testing")
    logger.info("⚠ For production, use embed_chapters_to_collection.py with OpenAI API\n")

    # Validate environment
    validate_environment()

    # Read chapters (limit to first 3 for testing)
    logger.info("Reading chapter files (first 3 for testing)...")
    chapters = read_chapters(limit=3)
    logger.info(f"✓ Read {len(chapters)} chapters\n")

    # Generate embeddings and prepare points
    logger.info("Generating MOCK embeddings...")
    points = []
    success_count = 0

    for i, chapter in enumerate(chapters, 1):
        try:
            logger.info(f"[{i}/{len(chapters)}] Processing: {chapter['name']}")

            # Generate mock embedding
            embedding = generate_mock_embedding()

            # Create point
            point_id = str(uuid.uuid4())
            point = {
                "id": point_id,
                "vector": embedding,
                "payload": {
                    "chapter_name": chapter['name'],
                    "content": chapter['content'][:500] + "...",  # Truncate for storage
                    "book_id": "physical-ai-humanoid",
                    "source_file": chapter['file'],
                    "is_mock": True
                }
            }

            points.append(point)
            success_count += 1
            logger.info(f"  ✓ Generated MOCK embedding (UUID: {point_id[:8]}...)")

        except Exception as e:
            logger.error(f"  ✗ Failed to process chapter: {e}")
            continue

    logger.info(f"\n✓ Successfully generated {success_count}/{len(chapters)} MOCK embeddings\n")

    # Insert points into Qdrant
    logger.info(f"Inserting {len(points)} points into Qdrant...")
    logger.info(f"Collection: {COLLECTION_NAME}")
    logger.info(f"Endpoint: {get_qdrant_base_url()}/collections/{COLLECTION_NAME}/points\n")

    success, message = insert_points_to_qdrant(points)

    if success:
        logger.info(f"✓ Successfully inserted all points to Qdrant")
    else:
        logger.error(f"✗ Failed to insert points: {message}")
        sys.exit(1)

    # Verify collection
    logger.info("\nVerifying collection...")
    collection_info = get_collection_info()

    if collection_info:
        points_count = collection_info.get('points_count', 0)
        vectors_count = collection_info.get('vectors_count', 0)

        logger.info(f"✓ Collection verified:")
        logger.info(f"  - Points count: {points_count}")
        logger.info(f"  - Vectors count: {vectors_count}")

        # Create summary
        summary = {
            "collection_name": COLLECTION_NAME,
            "points_inserted": len(points),
            "points_count": points_count,
            "vectors_count": vectors_count,
            "embedding_model": "MOCK (random vectors)",
            "embedding_dimensions": EMBEDDING_DIMENSIONS,
            "chapters_processed": success_count,
            "health_status": "healthy" if points_count > 0 else "empty",
            "rag_ready": False,  # MOCK embeddings won't work for real RAG
            "is_mock": True,
            "note": "These are MOCK embeddings for testing only. Use embed_chapters_to_collection.py with OpenAI API for production.",
            "confirmation": f"Collection now has {points_count} MOCK entries. RAG queries will NOT work properly with mock embeddings. Use real embeddings for production."
        }

        # Print summary
        logger.info(f"\n{'='*70}")
        logger.info("Summary")
        logger.info(f"{'='*70}\n")
        print(json.dumps(summary, indent=2))

        return summary
    else:
        logger.error("✗ Failed to verify collection")
        sys.exit(1)


if __name__ == "__main__":
    try:
        summary = embed_and_insert_chapters()
        sys.exit(0)
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
