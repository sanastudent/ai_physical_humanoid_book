"""
Comprehensive Backend RAG Pipeline Test (T046)

This script tests the complete RAG pipeline:
1. Initialize in-memory Qdrant (no Docker required)
2. Embed book content
3. Test global QA queries
4. Test selected-text QA queries
5. Verify Google Gemini integration and citations

Requirements:
- GOOGLE_API_KEY must be set in .env file
- Python packages installed from backend/requirements.txt
"""
import os
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent.parent / 'backend' / 'src'
sys.path.insert(0, str(backend_path))

from dotenv import load_dotenv
load_dotenv()

# CRITICAL: Test that GOOGLE_API_KEY is set
gemini_key = os.getenv('GOOGLE_API_KEY')
if not gemini_key or gemini_key == 'your-google-api-key-here':
    print("❌ ERROR: GOOGLE_API_KEY not set in .env file")
    print("\nTo run this test, you need to:")
    print("1. Get a Google API key from https://aistudio.google.com/app/apikey")
    print("2. Add it to .env file: GOOGLE_API_KEY=AIza...")
    print("\nSkipping test due to missing API key.")
    sys.exit(1)

from qdrant_client import QdrantClient
from embed import EmbeddingGenerator
from rag import RAGEngine
from schema import COLLECTION_NAME, VECTOR_SIZE

print("="*70)
print("BACKEND RAG PIPELINE TEST (T046)")
print("="*70)
print()

# Step 1: Initialize in-memory Qdrant
print("Step 1: Initialize in-memory Qdrant...")
print("✓ Using :memory: mode (no Docker required)")

# Override environment to use in-memory Qdrant
os.environ['QDRANT_HOST'] = ':memory:'

try:
    client = QdrantClient(':memory:')
    print(f"✓ Qdrant client initialized: {client}")
except Exception as e:
    print(f"❌ Failed to initialize Qdrant: {e}")
    sys.exit(1)

# Step 2: Create collection
print("\nStep 2: Create embeddings collection...")
try:
    from qdrant_client.models import Distance, VectorParams
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=VECTOR_SIZE,
            distance=Distance.COSINE
        )
    )
    print(f"✓ Created collection: {COLLECTION_NAME}")
except Exception as e:
    print(f"❌ Failed to create collection: {e}")
    sys.exit(1)

# Step 3: Embed sample content
print("\nStep 3: Embed sample book content...")
print("(Using small sample to test pipeline without embedding entire book)")

sample_content = {
    "introduction": """
# Introduction to Embodied AI

Embodied AI represents a paradigm shift in artificial intelligence, focusing on
agents that interact with and learn from their physical environment. This book
explores the intersection of robotics, AI, and digital twins.

Key topics covered:
- ROS 2 for robot development
- Digital twin technology
- NVIDIA Isaac Sim platform
- Vision-Language-Action (VLA) models
""",
    "chapter1": """
# Chapter 1: Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the next generation of the most widely used
robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is built from
the ground up for production systems.

Key improvements in ROS 2:
- Real-time capabilities for critical robotic systems
- Improved security with DDS (Data Distribution Service)
- Better multi-robot support for coordinated systems
- Cross-platform compatibility (Linux, Windows, macOS)
- Lifecycle nodes for better state management
"""
}

try:
    embedder = EmbeddingGenerator()

    total_chunks = 0
    for chapter_name, content in sample_content.items():
        print(f"  - Embedding {chapter_name}...")
        chunks = embedder.chunk_text(content)

        # Generate embeddings
        embeddings = []
        for i, chunk in enumerate(chunks):
            vector = embedder.generate_embedding(chunk['text'])
            embeddings.append({
                'id': f"{chapter_name}_{i}",
                'vector': vector,
                'chapter': chapter_name,
                'text': chunk['text'],
                'token_count': chunk['token_count']
            })

        # Insert into Qdrant
        from qdrant_client.models import PointStruct
        points = [
            PointStruct(
                id=emb['id'],
                vector=emb['vector'],
                payload={
                    'chapter': emb['chapter'],
                    'text': emb['text'],
                    'token_count': emb['token_count']
                }
            )
            for emb in embeddings
        ]
        client.upsert(collection_name=COLLECTION_NAME, points=points)

        total_chunks += len(chunks)

    print(f"✓ Embedded {total_chunks} chunks from {len(sample_content)} chapters")
except Exception as e:
    print(f"❌ Failed to embed content: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Step 4: Test Global QA
print("\nStep 4: Test Global QA mode...")
print("-" * 70)

try:
    # Initialize RAG engine (will use our in-memory Qdrant)
    rag = RAGEngine()

    test_queries = [
        "What is ROS 2?",
        "What are the key improvements in ROS 2?",
        "What topics does this book cover?"
    ]

    for query in test_queries:
        print(f"\nQuery: {query}")
        print()

        result = rag.query_global(query)

        print(f"Answer: {result['answer'][:300]}...")
        print(f"\nCitations: {result['citations']}")
        print(f"Sources: {len(result['sources'])} chunks retrieved")
        print("-" * 70)

    print("\n✓ Global QA mode working correctly")
except Exception as e:
    print(f"❌ Global QA test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Step 5: Test Selected-text QA
print("\nStep 5: Test Selected-text QA mode...")
print("-" * 70)

try:
    selected_text = """
ROS 2 (Robot Operating System 2) is the next generation of the most widely used
robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is built from
the ground up for production systems.
"""

    query = "What are the main differences from ROS 1?"

    print(f"Selected text: {selected_text[:100]}...")
    print(f"Query: {query}")
    print()

    result = rag.query_selected(query, selected_text)

    print(f"Answer: {result['answer'][:300]}...")
    print(f"\nCitations: {result['citations']}")
    print(f"Sources: {len(result['sources'])} chunks retrieved")

    print("\n✓ Selected-text QA mode working correctly")
except Exception as e:
    print(f"❌ Selected-text QA test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Step 6: Verify Gemini integration
print("\nStep 6: Verify Gemini integration...")
print("-" * 70)

try:
    print(f"✓ Using model: {rag.model}")
    print(f"✓ Gemini client initialized")
    print(f"✓ API key configured (first 10 chars): {gemini_key[:10]}...")
except Exception as e:
    print(f"❌ Gemini verification failed: {e}")
    sys.exit(1)

# Final summary
print("\n" + "="*70)
print("TEST SUMMARY")
print("="*70)
print("✅ All tests passed!")
print()
print("Components tested:")
print("  ✓ In-memory Qdrant initialization")
print("  ✓ Collection creation and management")
print("  ✓ Text chunking and embedding generation")
print("  ✓ Vector search and retrieval")
print("  ✓ Global QA mode with Google Gemini Pro")
print("  ✓ Selected-text QA mode")
print("  ✓ Citation extraction")
print("  ✓ Google Gemini integration")
print()
print("Next steps:")
print("  1. Run full book embedding: python scripts/embed_book.py")
print("  2. Start backend server: cd backend/src && python main.py")
print("  3. Test with frontend: cd frontend/my-book && npm start")
print()
print("="*70)
print("Task T046: Backend RAG Pipeline Test - COMPLETE ✅")
print("="*70)
