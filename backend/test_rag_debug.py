"""
RAG Chatbot Debugging Script

This script helps diagnose issues with the RAG chatbot by checking:
1. Qdrant collection status and embedding count
2. Embedding generation and dimension verification
3. Sample query retrieval and response generation
4. API key configuration

Usage:
    cd backend
    python test_rag_debug.py
"""

import os
import sys
import logging
from dotenv import load_dotenv

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

load_dotenv()

def print_section(title):
    """Print a section header"""
    print("\n" + "="*80)
    print(f"  {title}")
    print("="*80 + "\n")

def check_api_keys():
    """Check which API keys are configured"""
    print_section("1. API Key Configuration")

    openai_key = os.getenv("OPENAI_API_KEY")
    google_key = os.getenv("GOOGLE_API_KEY")
    anthropic_key = os.getenv("ANTHROPIC_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = os.getenv("QDRANT_PORT", "6333")

    print(f"✓ OPENAI_API_KEY: {'Set' if openai_key else '❌ NOT SET'}")
    print(f"✓ GOOGLE_API_KEY: {'Set' if google_key else '❌ NOT SET'}")
    print(f"✓ ANTHROPIC_API_KEY: {'Set' if anthropic_key else '❌ NOT SET'}")
    print(f"✓ QDRANT_URL: {qdrant_url if qdrant_url else f'Not set (using host/port)'}")
    print(f"✓ QDRANT_HOST: {qdrant_host}")
    print(f"✓ QDRANT_PORT: {qdrant_port}")

    if not openai_key and not google_key:
        print("\n⚠️  WARNING: No embedding API key configured!")
        print("   Set either OPENAI_API_KEY or GOOGLE_API_KEY for embeddings.")
        return False

    return True

def check_qdrant_collection():
    """Check Qdrant collection status"""
    print_section("2. Qdrant Collection Status")

    try:
        from src.qdrant_manager import QdrantManager

        qdrant = QdrantManager()
        print(f"✓ Qdrant client initialized")
        print(f"✓ Collection name: {qdrant.collection_name}")

        # Get collection info
        try:
            collection_info = qdrant.client.get_collection(qdrant.collection_name)
            print(f"✓ Collection exists")
            print(f"  - Vectors count: {collection_info.vectors_count}")
            print(f"  - Points count: {collection_info.points_count}")
            print(f"  - Vector size: {collection_info.config.params.vectors.size}")

            if collection_info.points_count == 0:
                print("\n⚠️  WARNING: Collection is empty! No embeddings stored.")
                print("   Use /embed or /embed-book endpoint to store book content.")
                return False

            return True

        except Exception as e:
            print(f"❌ Collection does not exist or error accessing it: {e}")
            print("   Run qdrant.create_collection() or start the backend to create it.")
            return False

    except Exception as e:
        logger.error(f"Failed to check Qdrant: {e}")
        print(f"❌ Error: {e}")
        return False

def test_embedding_generation():
    """Test embedding generation"""
    print_section("3. Embedding Generation Test")

    try:
        from src.embed import EmbeddingGenerator

        embedder = EmbeddingGenerator()
        print(f"✓ Embedding generator initialized")
        print(f"  - Provider: {embedder.provider}")

        # Generate test embedding
        test_text = "What is embodied AI?"
        print(f"\nGenerating embedding for: '{test_text}'")

        embedding = embedder.generate_embedding(test_text)
        print(f"✓ Embedding generated successfully")
        print(f"  - Dimensions: {len(embedding)}")
        print(f"  - First 5 values: {embedding[:5]}")

        # Check if it's dummy embeddings
        if all(v == 0.0 for v in embedding):
            print("\n⚠️  WARNING: Dummy embeddings detected (all zeros)")
            print("   This means no valid API key is configured.")
            print("   RAG will not work properly with dummy embeddings.")
            return False

        if len(embedding) != 1536:
            print(f"\n⚠️  WARNING: Embedding dimension mismatch!")
            print(f"   Expected 1536, got {len(embedding)}")
            print("   This will cause Qdrant search to fail.")
            return False

        print(f"✓ Embedding dimensions correct (1536)")
        return True

    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        print(f"❌ Error: {e}")
        return False

def test_qdrant_search():
    """Test Qdrant search with sample query"""
    print_section("4. Qdrant Search Test")

    try:
        from src.qdrant_manager import QdrantManager
        from src.embed import EmbeddingGenerator

        qdrant = QdrantManager()
        embedder = EmbeddingGenerator()

        # Generate query embedding
        query = "What is embodied AI?"
        print(f"Query: '{query}'")
        print(f"Generating query embedding...")

        query_vector = embedder.generate_embedding(query)
        print(f"✓ Query embedding generated ({len(query_vector)} dimensions)")

        # Search Qdrant
        print(f"Searching Qdrant for top 5 similar chunks...")
        results = qdrant.search(query_vector, limit=5)

        print(f"✓ Search completed")
        print(f"  - Results found: {len(results)}")

        if len(results) == 0:
            print("\n⚠️  NO RESULTS FOUND!")
            print("   Possible causes:")
            print("   1. No embeddings in Qdrant collection")
            print("   2. Embedding dimension mismatch")
            print("   3. Collection not initialized properly")
            return False

        # Show top results
        print(f"\nTop {min(3, len(results))} results:")
        for i, result in enumerate(results[:3]):
            print(f"\n  Result {i+1}:")
            print(f"    - Chapter: {result.payload.get('chapter', 'N/A')}")
            print(f"    - Score: {result.score:.4f}")
            print(f"    - Text preview: {result.payload.get('text', 'N/A')[:100]}...")

        return True

    except Exception as e:
        logger.error(f"Failed to search Qdrant: {e}")
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_rag_query():
    """Test complete RAG query"""
    print_section("5. Full RAG Query Test")

    try:
        from src.rag import RAGEngine

        rag = RAGEngine()
        print(f"✓ RAG engine initialized")

        # Test query
        query = "What is embodied AI?"
        print(f"\nQuerying RAG system: '{query}'")

        result = rag.query_global(query)

        print(f"✓ RAG query completed")
        print(f"\nAnswer:")
        print(f"  {result['answer'][:500]}...")
        print(f"\nCitations: {result.get('citations', [])}")
        print(f"Sources: {len(result.get('sources', []))} source chunks")

        # Check debug info
        if 'debug_info' in result:
            print(f"\nDebug Info:")
            for key, value in result['debug_info'].items():
                print(f"  - {key}: {value}")

        return True

    except Exception as e:
        logger.error(f"Failed to query RAG: {e}")
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all diagnostic tests"""
    print("""
╔════════════════════════════════════════════════════════════════════════════╗
║                   RAG CHATBOT DIAGNOSTIC TOOL                              ║
╚════════════════════════════════════════════════════════════════════════════╝
    """)

    results = {
        "API Keys": check_api_keys(),
        "Qdrant Collection": check_qdrant_collection(),
        "Embedding Generation": test_embedding_generation(),
        "Qdrant Search": test_qdrant_search(),
        "Full RAG Query": test_rag_query()
    }

    # Summary
    print_section("DIAGNOSTIC SUMMARY")

    all_passed = True
    for test_name, passed in results.items():
        status = "✓ PASS" if passed else "❌ FAIL"
        print(f"{status} - {test_name}")
        if not passed:
            all_passed = False

    print("\n" + "="*80)
    if all_passed:
        print("✓ All tests passed! RAG chatbot should be working.")
    else:
        print("❌ Some tests failed. Please review the output above and fix issues.")
        print("\nCommon fixes:")
        print("  1. Set API keys in .env file")
        print("  2. Ensure Qdrant is running (docker run -p 6333:6333 qdrant/qdrant)")
        print("  3. Embed book content using /embed or /embed-book endpoint")
        print("  4. Check embedding dimensions match Qdrant schema (1536)")

    print("="*80 + "\n")

if __name__ == "__main__":
    main()
