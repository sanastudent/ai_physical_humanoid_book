#!/usr/bin/env python3
"""
Test script to verify backend connectivity and fix issues
"""
import os
import sys
import asyncio
from dotenv import load_dotenv

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.rag import RAGEngine
from src.qdrant_manager import QdrantManager
from src.embed import EmbeddingGenerator

load_dotenv()

def test_connectivity():
    """Test all backend components connectivity"""
    print("="*60)
    print("Testing Backend Connectivity")
    print("="*60)

    # Test 1: Environment variables
    print("\n1. Testing Environment Variables:")
    google_key = os.getenv("GOOGLE_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    print(f"   [OK] GOOGLE_API_KEY: {'Set' if google_key else '[ERROR] NOT SET'}")
    print(f"   [OK] QDRANT_URL: {'Set' if qdrant_url else '[ERROR] NOT SET'}")
    print(f"   [OK] QDRANT_API_KEY: {'Set' if qdrant_api_key else '[ERROR] NOT SET'}")

    if not google_key:
        print("   [ERROR] GOOGLE_API_KEY is not set in .env")
        return False
    if not qdrant_url:
        print("   [ERROR] QDRANT_URL is not set in .env")
        return False
    if not qdrant_api_key:
        print("   [ERROR] QDRANT_API_KEY is not set in .env")
        return False

    # Test 2: Qdrant connection
    print("\n2. Testing Qdrant Connection:")
    try:
        qdrant_manager = QdrantManager()
        print("   [OK] Qdrant manager initialized successfully")

        # Test health check
        async def check_qdrant():
            health = await qdrant_manager.health_check()
            return health
        health = asyncio.run(check_qdrant())
        print(f"   [OK] Qdrant health check: Connected to {health['collection_count']} collections")

        # Check if collection exists
        schema_ok = asyncio.run(qdrant_manager.verify_collection_schema())
        print(f"   [OK] Collection schema: {'Valid' if schema_ok else 'Invalid'}")

    except Exception as e:
        print(f"   [ERROR] Qdrant connection failed: {e}")
        return False

    # Test 3: Embedding generator
    print("\n3. Testing Embedding Generator:")
    try:
        embedder = EmbeddingGenerator()
        print(f"   [OK] Embedding generator initialized with provider: {embedder.provider}")

        # Test embedding generation
        test_embedding = embedder.generate_embedding("test")
        print(f"   [OK] Test embedding generated: {len(test_embedding)} dimensions")

    except Exception as e:
        print(f"   [ERROR] Embedding generator failed: {e}")
        return False

    # Test 4: RAG Engine
    print("\n4. Testing RAG Engine:")
    try:
        rag_engine = RAGEngine()
        print("   [OK] RAG engine initialized successfully")

        # Test context retrieval
        context = rag_engine.retrieve_context("test", limit=1)
        print(f"   [OK] Context retrieval: Found {len(context)} results")

    except Exception as e:
        print(f"   [ERROR] RAG engine failed: {e}")
        return False

    print("\n" + "="*60)
    print("[SUCCESS] ALL CONNECTIVITY TESTS PASSED!")
    print("The backend server should work correctly.")
    print("Start it with: python -m src.main")
    print("="*60)

    return True

if __name__ == "__main__":
    success = test_connectivity()
    if not success:
        print("\n[ERROR] Some tests failed. Please check the errors above.")
        sys.exit(1)
    else:
        print("\n[SUCCESS] Backend connectivity verified successfully!")