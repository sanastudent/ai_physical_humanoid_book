"""
Embed Book Chapters to Qdrant Collection (my_1st_ai_book)

This script:
1. Reads all book chapters from frontend/my-book/docs/chapters/*.md
2. Generates embeddings using OpenAI text-embedding-3-small (1536 dimensions)
3. Inserts embeddings into Qdrant cloud collection via REST API
4. Verifies collection has points
5. Returns JSON summary

Note: Uses text-embedding-3-small (1536-dim) to match collection schema.
"""

import os
import sys
import glob
import uuid
import json
import requests
from pathlib import Path
from dotenv import load_dotenv
from openai import OpenAI
import logging

# Load environment variables from root directory
REPO_ROOT_FOR_ENV = Path(__file__).parent.parent
load_dotenv(REPO_ROOT_FOR_ENV / ".env")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY") or os.getenv("OPENAI_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "my_1st_ai_book"
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSIONS = 1536  # Must match collection schema (verified: 1536-dim Cosine)

# Paths
REPO_ROOT = Path(__file__).parent.parent
CHAPTERS_DIR = REPO_ROOT / "frontend" / "my-book" / "docs" / "chapters"


def validate_environment():
    """Validate all required environment variables are set"""
    missing = []
    if not OPENAI_API_KEY:
        missing.append("OPENAI_API_KEY")
    if not QDRANT_URL:
        missing.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY")

    if missing:
        logger.error(f"Missing environment variables: {', '.join(missing)}")
        logger.error("Please ensure .env file has all required variables")
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


def read_chapters():
    """Read all chapter files from the chapters directory"""
    if not CHAPTERS_DIR.exists():
        logger.error(f"Chapters directory not found: {CHAPTERS_DIR}")
        sys.exit(1)

    chapter_files = sorted(CHAPTERS_DIR.glob("*.md"))

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


def generate_embedding(text, client):
    """Generate embedding for text using OpenAI"""
    try:
        response = client.embeddings.create(
            model=EMBEDDING_MODEL,
            input=text,
            encoding_format="float"
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        raise


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
    logger.info("Embedding Chapters to Qdrant Collection")
    logger.info(f"{'='*70}\n")

    # Validate environment
    validate_environment()

    # Initialize OpenAI client
    logger.info(f"Initializing OpenAI client...")
    from openai import OpenAI as OpenAIClient
    client = OpenAIClient(api_key=OPENAI_API_KEY)
    logger.info(f"✓ OpenAI client initialized")
    logger.info(f"Model: {EMBEDDING_MODEL}")
    logger.info(f"Dimensions: {EMBEDDING_DIMENSIONS}\n")

    # Read chapters
    logger.info("Reading chapter files...")
    chapters = read_chapters()
    logger.info(f"✓ Read {len(chapters)} chapters\n")

    # Generate embeddings and prepare points
    logger.info("Generating embeddings...")
    points = []
    success_count = 0

    for i, chapter in enumerate(chapters, 1):
        try:
            logger.info(f"[{i}/{len(chapters)}] Processing: {chapter['name']}")

            # Generate embedding
            embedding = generate_embedding(chapter['content'], client)

            # Create point
            point_id = str(uuid.uuid4())
            point = {
                "id": point_id,
                "vector": embedding,
                "payload": {
                    "chapter_name": chapter['name'],
                    "content": chapter['content'],
                    "book_id": "physical-ai-humanoid",
                    "source_file": chapter['file']
                }
            }

            points.append(point)
            success_count += 1
            logger.info(f"  ✓ Generated embedding (UUID: {point_id[:8]}...)")

        except Exception as e:
            logger.error(f"  ✗ Failed to process chapter: {e}")
            continue

    logger.info(f"\n✓ Successfully generated {success_count}/{len(chapters)} embeddings\n")

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
            "embedding_model": EMBEDDING_MODEL,
            "embedding_dimensions": EMBEDDING_DIMENSIONS,
            "chapters_processed": success_count,
            "health_status": "healthy" if points_count > 0 else "empty",
            "rag_ready": points_count > 0,
            "confirmation": f"RAG queries will now work with {points_count} embedded chapters" if points_count > 0 else "Collection is empty - RAG queries will not work"
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
        sys.exit(1)
