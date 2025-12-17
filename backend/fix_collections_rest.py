"""
Automated Qdrant Collection Schema Fix Script (REST API)

This script uses direct REST API calls to avoid Pydantic version mismatches.
"""

import os
import sys
import requests
from dotenv import load_dotenv
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
EXPECTED_DISTANCE = "Cosine"


def get_base_url():
    """Get base URL without trailing slash"""
    url = QDRANT_URL.rstrip('/')
    if not url.endswith(':6333'):
        url += ':6333'
    return url


def get_headers():
    """Get headers for API requests"""
    return {
        "api-key": QDRANT_API_KEY,
        "Content-Type": "application/json"
    }


def get_collection_info(collection_name):
    """Get collection information using REST API"""
    url = f"{get_base_url()}/collections/{collection_name}"

    try:
        response = requests.get(url, headers=get_headers(), timeout=30)

        if response.status_code == 404:
            logger.info(f"  Collection '{collection_name}' does not exist")
            return None

        if response.status_code != 200:
            logger.error(f"  Failed to get collection info: {response.status_code} - {response.text}")
            return None

        data = response.json()
        return data.get('result', {})

    except Exception as e:
        logger.error(f"  Error getting collection info: {e}")
        return None


def validate_collection_schema(info):
    """Validate collection has correct vector size and distance metric"""
    if not info:
        return False, "Collection does not exist"

    try:
        vector_config = info['config']['params']['vectors']
        vector_size = vector_config['size']
        distance = vector_config['distance']

        # Check vector size
        if vector_size != EXPECTED_VECTOR_SIZE:
            return False, f"Invalid vector size: {vector_size} (expected {EXPECTED_VECTOR_SIZE})"

        # Check distance metric
        if distance.upper() != EXPECTED_DISTANCE.upper():
            return False, f"Invalid distance metric: {distance} (expected {EXPECTED_DISTANCE})"

        return True, "Schema valid"
    except Exception as e:
        return False, f"Schema validation error: {e}"


def delete_collection(collection_name):
    """Delete a collection using REST API"""
    url = f"{get_base_url()}/collections/{collection_name}"

    try:
        response = requests.delete(url, headers=get_headers(), timeout=30)

        if response.status_code in [200, 204]:
            logger.info(f"  ✓ Deleted collection: {collection_name}")
            return True
        else:
            logger.error(f"  ✗ Failed to delete: {response.status_code} - {response.text}")
            return False

    except Exception as e:
        logger.error(f"  ✗ Error deleting collection: {e}")
        return False


def create_collection(collection_name):
    """Create collection with correct schema using REST API"""
    url = f"{get_base_url()}/collections/{collection_name}"

    payload = {
        "vectors": {
            "size": EXPECTED_VECTOR_SIZE,
            "distance": EXPECTED_DISTANCE
        }
    }

    try:
        response = requests.put(url, headers=get_headers(), json=payload, timeout=30)

        if response.status_code in [200, 201]:
            logger.info(f"  ✓ Created collection: {collection_name}")
            logger.info(f"    - Vector size: {EXPECTED_VECTOR_SIZE}")
            logger.info(f"    - Distance metric: {EXPECTED_DISTANCE}")
            return True
        else:
            logger.error(f"  ✗ Failed to create: {response.status_code} - {response.text}")
            return False

    except Exception as e:
        logger.error(f"  ✗ Error creating collection: {e}")
        return False


def fix_collection_schema(collection_name):
    """Fix a single collection's schema"""
    logger.info(f"\n{'='*70}")
    logger.info(f"Processing collection: {collection_name}")
    logger.info(f"{'='*70}")

    # Step 1: Check if collection exists
    info = get_collection_info(collection_name)

    if not info:
        logger.info(f"Collection '{collection_name}' does not exist - creating...")
        return create_collection(collection_name)

    # Step 2: Validate schema
    is_valid, message = validate_collection_schema(info)

    if is_valid:
        logger.info(f"✓ Collection '{collection_name}' schema is already valid")
        logger.info(f"  - Vector size: {info['config']['params']['vectors']['size']}")
        logger.info(f"  - Distance: {info['config']['params']['vectors']['distance']}")
        logger.info(f"  - Vectors count: {info.get('vectors_count', 0)}")
        logger.info(f"  - Points count: {info.get('points_count', 0)}")
        return True

    # Step 3: Schema is invalid - need to fix
    logger.warning(f"⚠ Collection '{collection_name}' has invalid schema: {message}")
    logger.info(f"  Current config:")
    try:
        logger.info(f"    - Vector size: {info['config']['params']['vectors']['size']}")
        logger.info(f"    - Distance: {info['config']['params']['vectors']['distance']}")
        logger.info(f"    - Vectors count: {info.get('vectors_count', 0)}")
    except:
        logger.info(f"    - Unable to parse current config")

    # Step 4: Delete and recreate
    logger.info(f"\nFixing collection '{collection_name}'...")

    if not delete_collection(collection_name):
        return False

    # Wait a moment for deletion to complete
    import time
    time.sleep(2)

    if not create_collection(collection_name):
        return False

    logger.info(f"✓ Collection '{collection_name}' schema fixed successfully")
    return True


def verify_all_collections():
    """Verify all collections have correct schema"""
    logger.info(f"\n{'='*70}")
    logger.info("Final Verification")
    logger.info(f"{'='*70}\n")

    all_valid = True

    for collection_name in COLLECTIONS_TO_FIX:
        info = get_collection_info(collection_name)

        if not info:
            logger.error(f"✗ Collection '{collection_name}' does not exist")
            all_valid = False
            continue

        is_valid, message = validate_collection_schema(info)

        if is_valid:
            logger.info(f"✓ Collection '{collection_name}':")
            logger.info(f"  - Status: HEALTHY")
            logger.info(f"  - Vector size: {info['config']['params']['vectors']['size']}")
            logger.info(f"  - Distance: {info['config']['params']['vectors']['distance']}")
            logger.info(f"  - Vectors count: {info.get('vectors_count', 0)}")
            logger.info(f"  - Points count: {info.get('points_count', 0)}")
        else:
            logger.error(f"✗ Collection '{collection_name}': {message}")
            all_valid = False

    return all_valid


def main():
    """Main execution function"""
    logger.info(f"\n{'#'*70}")
    logger.info("# Qdrant Collection Schema Fix - REST API Method")
    logger.info(f"{'#'*70}\n")

    # Validate environment
    if not QDRANT_URL or not QDRANT_API_KEY:
        logger.error("Missing QDRANT_URL or QDRANT_API_KEY in environment variables")
        return 1

    logger.info(f"Target URL: {get_base_url()}")
    logger.info(f"Target collections: {', '.join(COLLECTIONS_TO_FIX)}")
    logger.info(f"Expected vector size: {EXPECTED_VECTOR_SIZE}")
    logger.info(f"Expected distance metric: {EXPECTED_DISTANCE}\n")

    # Fix each collection
    results = {}
    for collection_name in COLLECTIONS_TO_FIX:
        results[collection_name] = fix_collection_schema(collection_name)

    # Verify all collections
    all_valid = verify_all_collections()

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
        logger.info("  1. Backend should automatically detect healthy collections")
        logger.info("  2. Verify health: curl http://localhost:8000/health/qdrant")
        logger.info("  3. If collections were recreated (empty), re-embed chapters:")
        logger.info("     python embed_chapters_fixed.py")
        return 0
    else:
        logger.error(f"\n{'✗'*35}")
        logger.error("✗ SOME COLLECTIONS HAVE ISSUES")
        logger.error(f"{'✗'*35}\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
