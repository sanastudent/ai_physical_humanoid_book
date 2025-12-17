"""
Migrate from OpenAI (1536-dim) to Gemini (768-dim) embeddings

This script:
1. Deletes existing Qdrant collection
2. Creates new collection with 768-dim vectors for Gemini
3. Embeds all chapters using Gemini text-embedding-004
4. Verifies the migration
"""

import os
import sys
import uuid
import json
import requests
import google.generativeai as genai
from pathlib import Path
from dotenv import load_dotenv
import logging

# Load environment variables
REPO_ROOT = Path(__file__).parent.parent
load_dotenv(REPO_ROOT / ".env")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
COLLECTION_NAME = "book_embeddings"
EMBEDDING_MODEL = "models/text-embedding-004"
EMBEDDING_DIMENSIONS = 768
CHAPTERS_DIR = REPO_ROOT / "frontend" / "my-book" / "docs" / "chapters"


def validate_environment():
    """Validate required environment variables"""
    missing = []
    if not QDRANT_URL:
        missing.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY")
    if not GOOGLE_API_KEY:
        missing.append("GOOGLE_API_KEY")

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


def delete_collection():
    """Delete existing collection"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}"

    try:
        response = requests.delete(url, headers=get_qdrant_headers(), timeout=30)

        if response.status_code in [200, 204, 404]:
            logger.info(f"✓ Deleted/cleared collection: {COLLECTION_NAME}")
            return True
        else:
            logger.warning(f"Delete response: {response.status_code} - {response.text}")
            return True  # Continue anyway

    except Exception as e:
        logger.warning(f"Error deleting collection: {e}")
        return True  # Continue anyway


def create_collection():
    """Create new collection with 768-dim Gemini vectors"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}"

    payload = {
        "vectors": {
            "size": EMBEDDING_DIMENSIONS,
            "distance": "Cosine"
        }
    }

    try:
        response = requests.put(url, headers=get_qdrant_headers(), json=payload, timeout=30)

        if response.status_code in [200, 201]:
            logger.info(f"✓ Created collection: {COLLECTION_NAME}")
            logger.info(f"  - Vector size: {EMBEDDING_DIMENSIONS}")
            logger.info(f"  - Distance: Cosine")
            return True
        else:
            logger.error(f"Failed to create collection: {response.status_code} - {response.text}")
            return False

    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        return False


def read_chapters():
    """Read all chapter files"""
    if not CHAPTERS_DIR.exists():
        logger.error(f"Chapters directory not found: {CHAPTERS_DIR}")
        sys.exit(1)

    chapter_files = sorted(CHAPTERS_DIR.glob("*.md"))

    if not chapter_files:
        logger.error(f"No chapter files found in {CHAPTERS_DIR}")
        sys.exit(1)

    chapters = []
    for file_path in chapter_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            chapters.append({
                "name": file_path.stem,
                "content": content,
                "file": str(file_path)
            })

        except Exception as e:
            logger.warning(f"Failed to read {file_path.name}: {e}")
            continue

    logger.info(f"✓ Read {len(chapters)} chapters")
    return chapters


def generate_gemini_embedding(text):
    """Generate embedding using Gemini text-embedding-004"""
    try:
        result = genai.embed_content(
            model=EMBEDDING_MODEL,
            content=text,
            task_type="retrieval_document"
        )
        return result['embedding']
    except Exception as e:
        logger.error(f"Failed to generate Gemini embedding: {e}")
        raise


def insert_points_to_qdrant(points):
    """Insert points into Qdrant collection via REST API"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}/points"

    payload = {"points": points}

    try:
        response = requests.put(url, headers=get_qdrant_headers(), json=payload, timeout=60)

        if response.status_code in [200, 201]:
            return True, "Success"
        else:
            return False, f"{response.status_code}: {response.text}"

    except Exception as e:
        return False, str(e)


def get_collection_info():
    """Get collection information"""
    base_url = get_qdrant_base_url()
    url = f"{base_url}/collections/{COLLECTION_NAME}"

    try:
        response = requests.get(url, headers=get_qdrant_headers(), timeout=30)

        if response.status_code == 200:
            data = response.json()
            return data.get('result', {})
        else:
            return None

    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        return None


def main():
    """Main migration function"""
    logger.info(f"\n{'='*70}")
    logger.info("Migrating to Gemini Embeddings (768-dim)")
    logger.info(f"{'='*70}\n")

    # Validate environment
    validate_environment()

    # Configure Gemini
    logger.info("Configuring Gemini API...")
    genai.configure(api_key=GOOGLE_API_KEY)
    logger.info(f"✓ Gemini configured (model: {EMBEDDING_MODEL})\n")

    # Step 1: Delete old collection
    logger.info("Step 1: Deleting old collection...")
    delete_collection()
    import time
    time.sleep(2)

    # Step 2: Create new collection
    logger.info("\nStep 2: Creating new collection...")
    if not create_collection():
        logger.error("Failed to create collection")
        sys.exit(1)
    time.sleep(2)

    # Step 3: Read chapters
    logger.info("\nStep 3: Reading chapters...")
    chapters = read_chapters()

    # Step 4: Generate embeddings and insert
    logger.info(f"\nStep 4: Generating Gemini embeddings for {len(chapters)} chapters...")

    points = []
    success_count = 0

    for i, chapter in enumerate(chapters, 1):
        try:
            logger.info(f"[{i}/{len(chapters)}] Processing: {chapter['name']}")

            # Generate Gemini embedding
            embedding = generate_gemini_embedding(chapter['content'])

            # Verify dimension
            if len(embedding) != EMBEDDING_DIMENSIONS:
                logger.error(f"  Dimension mismatch: got {len(embedding)}, expected {EMBEDDING_DIMENSIONS}")
                continue

            # Create point
            point_id = str(uuid.uuid4())
            point = {
                "id": point_id,
                "vector": embedding,
                "payload": {
                    "chapter": chapter['name'],
                    "text": chapter['content'][:1000] + "..." if len(chapter['content']) > 1000 else chapter['content'],
                    "book_id": "physical-ai-humanoid",
                    "source_file": chapter['file'],
                    "embedding_model": "gemini-text-embedding-004"
                }
            }

            points.append(point)
            success_count += 1
            logger.info(f"  ✓ Generated embedding (UUID: {point_id[:8]}..., dim: {len(embedding)})")

        except Exception as e:
            logger.error(f"  ✗ Failed: {e}")
            continue

    logger.info(f"\n✓ Generated {success_count}/{len(chapters)} Gemini embeddings")

    # Step 5: Insert into Qdrant
    logger.info(f"\nStep 5: Inserting {len(points)} points into Qdrant...")

    success, message = insert_points_to_qdrant(points)

    if success:
        logger.info(f"✓ Successfully inserted all points")
    else:
        logger.error(f"✗ Failed to insert: {message}")
        sys.exit(1)

    # Step 6: Verify
    logger.info("\nStep 6: Verifying migration...")
    collection_info = get_collection_info()

    if collection_info:
        points_count = collection_info.get('points_count', 0)
        vectors_count = collection_info.get('vectors_count', 0)

        logger.info(f"✓ Collection verified:")
        logger.info(f"  - Collection: {COLLECTION_NAME}")
        logger.info(f"  - Points count: {points_count}")
        logger.info(f"  - Vectors count: {vectors_count}")
        logger.info(f"  - Vector size: {EMBEDDING_DIMENSIONS}")
        logger.info(f"  - Distance: Cosine")

        # Summary
        summary = {
            "migration_status": "success",
            "collection_name": COLLECTION_NAME,
            "embedding_model": "gemini-text-embedding-004",
            "vector_dimensions": EMBEDDING_DIMENSIONS,
            "chapters_embedded": success_count,
            "points_count": points_count,
            "vectors_count": vectors_count,
            "rag_ready": points_count > 0
        }

        logger.info(f"\n{'='*70}")
        logger.info("Migration Summary")
        logger.info(f"{'='*70}\n")
        print(json.dumps(summary, indent=2))

        return 0
    else:
        logger.error("✗ Failed to verify collection")
        return 1


if __name__ == "__main__":
    sys.exit(main())
