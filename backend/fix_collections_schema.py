"""
Automated Qdrant Collection Schema Fix Script

This script fixes degraded collection schemas by:
1. Checking existing collections
2. Validating their schemas
3. Fixing invalid schemas (delete + recreate with correct config)
4. Verifying final state
"""

import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTIONS_TO_FIX = ["my_1st_ai_book", "book_embeddings"]
EXPECTED_VECTOR_SIZE = 1536
EXPECTED_DISTANCE = Distance.COSINE


def connect_to_qdrant():
    """Connect to Qdrant cloud instance"""
    if not QDRANT_URL or not QDRANT_API_KEY:
        logger.error("Missing QDRANT_URL or QDRANT_API_KEY in environment variables")
        sys.exit(1)

    try:
        client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=60,
            https=True,
            prefer_grpc=False
        )
        logger.info(f"✓ Connected to Qdrant cloud: {QDRANT_URL}")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        sys.exit(1)


def get_collection_info(client, collection_name):
    """Get collection information if it exists"""
    try:
        info = client.get_collection(collection_name)
        return info
    except Exception as e:
        logger.warning(f"Collection '{collection_name}' does not exist or cannot be accessed: {e}")
        return None


def validate_collection_schema(info):
    """Validate collection has correct vector size and distance metric"""
    if not info:
        return False, "Collection does not exist"

    try:
        vector_config = info.config.params.vectors

        # Check vector size
        if vector_config.size != EXPECTED_VECTOR_SIZE:
            return False, f"Invalid vector size: {vector_config.size} (expected {EXPECTED_VECTOR_SIZE})"

        # Check distance metric
        if vector_config.distance != EXPECTED_DISTANCE:
            return False, f"Invalid distance metric: {vector_config.distance.name} (expected {EXPECTED_DISTANCE.name})"

        return True, "Schema valid"
    except Exception as e:
        return False, f"Schema validation error: {e}"


def delete_collection(client, collection_name):
    """Delete a collection"""
    try:
        client.delete_collection(collection_name)
        logger.info(f"  ✓ Deleted collection: {collection_name}")
        return True
    except Exception as e:
        logger.error(f"  ✗ Failed to delete collection '{collection_name}': {e}")
        return False


def create_collection(client, collection_name):
    """Create collection with correct schema"""
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=EXPECTED_VECTOR_SIZE,
                distance=EXPECTED_DISTANCE
            )
        )
        logger.info(f"  ✓ Created collection: {collection_name}")
        logger.info(f"    - Vector size: {EXPECTED_VECTOR_SIZE}")
        logger.info(f"    - Distance metric: {EXPECTED_DISTANCE.name}")
        return True
    except Exception as e:
        logger.error(f"  ✗ Failed to create collection '{collection_name}': {e}")
        return False


def fix_collection_schema(client, collection_name):
    """Fix a single collection's schema"""
    logger.info(f"\n{'='*70}")
    logger.info(f"Processing collection: {collection_name}")
    logger.info(f"{'='*70}")

    # Step 1: Check if collection exists
    info = get_collection_info(client, collection_name)

    if not info:
        logger.info(f"Collection '{collection_name}' does not exist - creating...")
        if create_collection(client, collection_name):
            return True
        else:
            return False

    # Step 2: Validate schema
    is_valid, message = validate_collection_schema(info)

    if is_valid:
        logger.info(f"✓ Collection '{collection_name}' schema is already valid")
        logger.info(f"  - Vector size: {info.config.params.vectors.size}")
        logger.info(f"  - Distance: {info.config.params.vectors.distance.name}")
        logger.info(f"  - Vectors count: {info.vectors_count}")
        logger.info(f"  - Points count: {info.points_count}")
        return True

    # Step 3: Schema is invalid - need to fix
    logger.warning(f"⚠ Collection '{collection_name}' has invalid schema: {message}")
    logger.info(f"  Current config:")
    logger.info(f"    - Vector size: {info.config.params.vectors.size}")
    logger.info(f"    - Distance: {info.config.params.vectors.distance.name}")
    logger.info(f"    - Vectors count: {info.vectors_count}")

    # Step 4: Delete and recreate
    logger.info(f"\nFixing collection '{collection_name}'...")

    if not delete_collection(client, collection_name):
        return False

    if not create_collection(client, collection_name):
        return False

    logger.info(f"✓ Collection '{collection_name}' schema fixed successfully")
    return True


def verify_all_collections(client):
    """Verify all collections have correct schema"""
    logger.info(f"\n{'='*70}")
    logger.info("Final Verification")
    logger.info(f"{'='*70}\n")

    all_valid = True

    for collection_name in COLLECTIONS_TO_FIX:
        info = get_collection_info(client, collection_name)

        if not info:
            logger.error(f"✗ Collection '{collection_name}' does not exist")
            all_valid = False
            continue

        is_valid, message = validate_collection_schema(info)

        if is_valid:
            logger.info(f"✓ Collection '{collection_name}':")
            logger.info(f"  - Status: HEALTHY")
            logger.info(f"  - Vector size: {info.config.params.vectors.size}")
            logger.info(f"  - Distance: {info.config.params.vectors.distance.name}")
            logger.info(f"  - Vectors count: {info.vectors_count}")
            logger.info(f"  - Points count: {info.points_count}")
        else:
            logger.error(f"✗ Collection '{collection_name}': {message}")
            all_valid = False

    return all_valid


def main():
    """Main execution function"""
    logger.info(f"\n{'#'*70}")
    logger.info("# Qdrant Collection Schema Fix - Automated")
    logger.info(f"{'#'*70}\n")

    logger.info(f"Target collections: {', '.join(COLLECTIONS_TO_FIX)}")
    logger.info(f"Expected vector size: {EXPECTED_VECTOR_SIZE}")
    logger.info(f"Expected distance metric: {EXPECTED_DISTANCE.name}\n")

    # Connect to Qdrant
    client = connect_to_qdrant()

    # Fix each collection
    results = {}
    for collection_name in COLLECTIONS_TO_FIX:
        results[collection_name] = fix_collection_schema(client, collection_name)

    # Verify all collections
    all_valid = verify_all_collections(client)

    # Final summary
    logger.info(f"\n{'='*70}")
    logger.info("Summary")
    logger.info(f"{'='*70}\n")

    logger.info("Collections processed:")
    for collection_name, success in results.items():
        status = "✓ SUCCESS" if success else "✗ FAILED"
        logger.info(f"  - {collection_name}: {status}")

    if all_valid:
        logger.info(f"\n{'✓'*35}")
        logger.info("✓ ALL COLLECTIONS ARE HEALTHY")
        logger.info(f"{'✓'*35}\n")
        logger.info("Next steps:")
        logger.info("  1. Restart backend if needed: cd backend && python -m src.main")
        logger.info("  2. Verify health: curl http://localhost:8000/health/qdrant")
        logger.info("  3. Re-embed chapters if collections were recreated:")
        logger.info("     python embed_chapters_fixed.py")
        return 0
    else:
        logger.error(f"\n{'✗'*35}")
        logger.error("✗ SOME COLLECTIONS HAVE ISSUES")
        logger.error(f"{'✗'*35}\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
