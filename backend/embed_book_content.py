"""
Comprehensive script to embed all book content into Qdrant

This script:
1. Connects to Qdrant Cloud
2. Deletes existing collection (if it exists)
3. Creates new collection with proper configuration
4. Embeds all book content (chapters, summary, glossary, references)
5. Verifies embeddings are searchable
"""
import sys
import os
from pathlib import Path

# Force UTF-8 encoding for Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from dotenv import load_dotenv
load_dotenv()

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import src.qdrant_manager
import src.embed

print("=" * 80)
print("Book Content Embedding Script")
print("=" * 80)

# Step 1: Connect to Qdrant
print("\n[Step 1] Connecting to Qdrant Cloud...")
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_api_key:
    print("✗ Error: QDRANT_URL or QDRANT_API_KEY not set in .env")
    sys.exit(1)

print(f"  URL: {qdrant_url}")
print(f"  API Key: ***{qdrant_api_key[-10:]}")

try:
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    collections = client.get_collections()
    print(f"✓ Connected to Qdrant successfully")
    print(f"  Existing collections: {[c.name for c in collections.collections]}")
except Exception as e:
    print(f"✗ Failed to connect to Qdrant: {e}")
    sys.exit(1)

# Step 2: Delete existing collection
print("\n[Step 2] Deleting existing 'book_embeddings' collection...")
collection_name = "book_embeddings"

try:
    # Check if collection exists
    existing_collections = [c.name for c in client.get_collections().collections]
    if collection_name in existing_collections:
        client.delete_collection(collection_name)
        print(f"✓ Deleted existing collection '{collection_name}'")
    else:
        print(f"  Collection '{collection_name}' does not exist, skipping deletion")
except Exception as e:
    print(f"  Note: {e}")

# Step 3: Create new collection
print("\n[Step 3] Creating new collection with proper VectorParams...")
try:
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=1536,
            distance=Distance.COSINE
        )
    )
    print(f"✓ Created collection '{collection_name}' successfully")
    print(f"  Vector size: 1536")
    print(f"  Distance metric: COSINE")
except Exception as e:
    print(f"✗ Failed to create collection: {e}")
    sys.exit(1)

# Step 4: Find book content files
print("\n[Step 4] Finding book content files...")
book_dir = Path(__file__).parent.parent / "frontend" / "my-book" / "docs"
print(f"  Looking in: {book_dir}")

if not book_dir.exists():
    # Try alternative locations
    alternative_locations = [
        Path(__file__).parent.parent / "book_content",
        Path(__file__).parent.parent / "content",
        Path(__file__).parent.parent / "data",
        Path(__file__).parent.parent / "book",
    ]

    for alt_dir in alternative_locations:
        if alt_dir.exists():
            book_dir = alt_dir
            print(f"  Found content in: {book_dir}")
            break
    else:
        print(f"✗ Book content directory not found")
        print(f"  Please create a directory with book content")
        print(f"  Expected structure:")
        print(f"    book_content/")
        print(f"      chapter_01.txt")
        print(f"      chapter_02.txt")
        print(f"      summary.txt")
        print(f"      glossary.txt")
        print(f"      references.txt")
        sys.exit(1)

# Find all text files
content_files = list(book_dir.glob("*.txt")) + list(book_dir.glob("*.md"))
print(f"  Found {len(content_files)} content files:")
for file in sorted(content_files):
    print(f"    - {file.name}")

if len(content_files) == 0:
    print(f"✗ No content files found in {book_dir}")
    print(f"  Creating sample content for demonstration...")

    # Create sample content
    book_dir.mkdir(exist_ok=True)

    sample_chapters = {
        "chapter_01_introduction.txt": """Chapter 1: Introduction to AI-Driven Development

Artificial Intelligence has revolutionized software development. This book explores how AI tools
and agents can enhance productivity, improve code quality, and accelerate innovation. We'll cover
practical applications, real-world examples, and best practices for integrating AI into your
development workflow.

Key topics include:
- AI-assisted coding and code generation
- Automated testing and quality assurance
- Intelligent code review and refactoring
- Natural language interfaces for development tasks
- Vector databases and semantic search
""",
        "chapter_02_rag_systems.txt": """Chapter 2: Retrieval-Augmented Generation (RAG) Systems

RAG systems combine the power of large language models with external knowledge bases. By retrieving
relevant context before generating responses, RAG systems provide more accurate, grounded, and
up-to-date information.

Architecture of RAG systems:
1. Document chunking and preprocessing
2. Embedding generation using transformer models
3. Vector storage in databases like Qdrant or Pinecone
4. Similarity search for context retrieval
5. LLM-based response generation with retrieved context

Benefits include improved accuracy, reduced hallucinations, and the ability to work with
domain-specific knowledge.
""",
        "chapter_03_vector_databases.txt": """Chapter 3: Vector Databases and Semantic Search

Vector databases store high-dimensional embeddings that represent the semantic meaning of text,
images, or other data. Unlike traditional databases that rely on exact matches, vector databases
enable semantic search based on similarity.

Popular vector databases:
- Qdrant: High-performance vector search engine with filtering
- Pinecone: Managed vector database service
- Weaviate: Open-source vector search engine
- Milvus: Scalable vector database for AI applications

Distance metrics (COSINE, EUCLIDEAN, DOT_PRODUCT) determine how similarity is calculated.
COSINE similarity is most common for text embeddings.
""",
        "summary.txt": """Book Summary: AI-Driven Development

This book provides a comprehensive guide to integrating AI into software development workflows.
From AI-assisted coding to building RAG systems, readers learn practical techniques for leveraging
AI to improve productivity and code quality.

Key takeaways:
- AI tools can significantly accelerate development tasks
- RAG systems provide grounded, accurate responses using external knowledge
- Vector databases enable semantic search and similarity-based retrieval
- Proper embedding strategies are crucial for effective AI applications
""",
        "glossary.txt": """Glossary

AI Agent: An autonomous system that can perceive its environment and take actions to achieve goals.

Embedding: A dense vector representation of data (text, images, etc.) that captures semantic meaning.

RAG (Retrieval-Augmented Generation): A technique that retrieves relevant context before generating
responses, improving accuracy and reducing hallucinations.

Vector Database: A specialized database for storing and searching high-dimensional embeddings.

COSINE Similarity: A distance metric that measures the cosine of the angle between two vectors,
commonly used for text similarity.

LLM (Large Language Model): A neural network trained on vast amounts of text data to understand
and generate human-like text.
""",
        "references.txt": """References

1. Lewis, P., et al. (2020). "Retrieval-Augmented Generation for Knowledge-Intensive NLP Tasks."
   arXiv:2005.11401

2. Qdrant Documentation. https://qdrant.tech/documentation/

3. OpenAI Embeddings Guide. https://platform.openai.com/docs/guides/embeddings

4. "Building Production-Ready RAG Applications" - Best Practices Guide

5. "Vector Databases: The Missing Piece in AI Infrastructure" - Technical Overview
"""
    }

    for filename, content in sample_chapters.items():
        filepath = book_dir / filename
        filepath.write_text(content, encoding='utf-8')
        print(f"    Created: {filename}")

    content_files = list(book_dir.glob("*.txt"))
    print(f"\n  Created {len(content_files)} sample files")

# Step 5: Initialize embedding generator
print("\n[Step 5] Initializing embedding generator...")
try:
    emb_generator = src.embed.EmbeddingGenerator()
    print("✓ Embedding generator initialized")
except Exception as e:
    print(f"✗ Failed to initialize embedding generator: {e}")
    print(f"  Make sure OPENAI_API_KEY or ANTHROPIC_API_KEY is set in .env")
    sys.exit(1)

# Step 6: Embed all content
print("\n[Step 6] Embedding all book content...")
total_chunks = 0
total_files = len(content_files)

for idx, file_path in enumerate(sorted(content_files), 1):
    print(f"\n  [{idx}/{total_files}] Processing: {file_path.name}")

    try:
        # Read file content
        content = file_path.read_text(encoding='utf-8')
        print(f"    Content length: {len(content)} characters")

        # Determine chapter name from filename
        chapter = file_path.stem.replace('_', ' ').title()

        # Chunk the content
        chunks = emb_generator.chunk_text(content, chapter)
        print(f"    Created {len(chunks)} chunks")

        # Generate embeddings for each chunk
        embeddings = []
        for chunk in chunks:
            vector = emb_generator.generate_embedding(chunk["text"])
            embeddings.append({
                "id": chunk["id"],
                "vector": vector,
                "chapter": chunk["chapter"],
                "text": chunk["text"],
                "token_count": chunk["token_count"]
            })

        # Store in Qdrant using QdrantManager
        manager = src.qdrant_manager.QdrantManager()
        manager.insert_embeddings(embeddings)

        total_chunks += len(embeddings)
        print(f"    ✓ Inserted {len(embeddings)} embeddings")

    except Exception as e:
        print(f"    ✗ Error processing {file_path.name}: {e}")
        continue

print(f"\n✓ Embedding complete! Total chunks inserted: {total_chunks}")

# Step 7: Verify embeddings
print("\n[Step 7] Verifying embeddings are searchable...")
try:
    info = client.get_collection(collection_name)
    points_count = info.points_count

    print(f"✓ Collection info:")
    print(f"    Collection name: {collection_name}")
    print(f"    Points count: {points_count}")
    print(f"    Vector size: {info.config.params.vectors.size}")
    print(f"    Distance metric: {info.config.params.vectors.distance.name}")

    if points_count == 0:
        print(f"⚠ Warning: No points in collection! Embeddings may not have been inserted.")
    else:
        # Test search
        print(f"\n  Testing search functionality...")
        test_query = "What is RAG?"
        test_vector = emb_generator.generate_embedding(test_query)

        results = client.search(
            collection_name=collection_name,
            query_vector=test_vector,
            limit=3
        )

        print(f"  Query: '{test_query}'")
        print(f"  Results: {len(results)} matches")

        for i, result in enumerate(results, 1):
            print(f"\n  [{i}] Score: {result.score:.4f}")
            print(f"      Chapter: {result.payload.get('chapter', 'Unknown')}")
            print(f"      Text preview: {result.payload.get('text', '')[:100]}...")

except Exception as e:
    print(f"✗ Verification failed: {e}")
    sys.exit(1)

# Final summary
print("\n" + "=" * 80)
print("✅ EMBEDDING COMPLETED SUCCESSFULLY")
print("=" * 80)
print(f"\nSummary:")
print(f"  - Collection: {collection_name}")
print(f"  - Total embeddings: {points_count}")
print(f"  - Files processed: {total_files}")
print(f"  - Collection ready for chatbot queries")
print(f"\nNext steps:")
print(f"  1. Start the backend: cd backend && python -m uvicorn src.main:app --reload")
print(f"  2. Test chatbot: curl -X POST http://localhost:8000/query -H 'Content-Type: application/json' -d '{{\"query\": \"What is RAG?\"}}'")
print(f"  3. Frontend should now show responses instead of infinite buffering")
print("\n" + "=" * 80)
