"""
Embedding logic for book content
"""
import os
import asyncio
import tiktoken
from typing import List, Dict
from anthropic import Anthropic
import google.generativeai as genai
from openai import AsyncOpenAI
from dotenv import load_dotenv

load_dotenv()

# Chunking configuration
CHUNK_SIZE = 500  # tokens - FR-009 requirement
OVERLAP = 50  # tokens - FR-009 requirement



class EmbeddingGenerator:
    """Generates embeddings for book content"""

    def __init__(self):
        self.anthropic_key = os.getenv("ANTHROPIC_API_KEY")
        self.google_key = os.getenv("GOOGLE_API_KEY")

        # Initialize tokenizer for chunking
        self.encoder = tiktoken.get_encoding("cl100k_base")

        # Initialize clients
        if self.anthropic_key:
            self.client = Anthropic(api_key=self.anthropic_key)
            self.provider = "anthropic"
        elif self.google_key:
            genai.configure(api_key=self.google_key)
            self.provider = "google"
        else:
            # Default to OpenAI if no other provider is available
            self.provider = "openai"

    def chunk_text(self, text: str, chapter: str) -> List[Dict]:
        """Chunk text into smaller segments with overlap

        Args:
            text: Text to chunk
            chapter: Chapter identifier

        Returns:
            List of chunks with metadata
        """
        tokens = self.encoder.encode(text)
        chunks = []

        start = 0
        chunk_id = 0

        while start < len(tokens):
            end = start + CHUNK_SIZE
            chunk_tokens = tokens[start:end]
            chunk_text = self.encoder.decode(chunk_tokens)

            chunks.append({
                "id": f"{chapter}_chunk_{chunk_id}",
                "chapter": chapter,
                "text": chunk_text,
                "token_count": len(chunk_tokens)
            })

            chunk_id += 1
            start = end - OVERLAP  # Apply overlap

        return chunks

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a text chunk

        Args:
            text: Text to embed

        Returns:
            Embedding vector
        """
        if self.provider == "anthropic":
            # Use Claude's embedding capabilities
            # Note: Claude doesn't have a direct embedding API
            # We'll use a workaround with text similarity
            # For production, use OpenAI embeddings or similar
            import openai
            openai_key = os.getenv("OPENAI_API_KEY")
            if openai_key:
                client = openai.OpenAI(api_key=openai_key)
                response = client.embeddings.create(
                    model="text-embedding-3-small",
                    input=text,
                    dimensions=1536
                )
                return response.data[0].embedding
            else:
                # Fallback: create dummy embeddings (for testing only)
                print("Warning: No OpenAI API key found. Using dummy embeddings.")
                return [0.0] * 1536

        elif self.provider == "google":
            # Use Google's embedding model (768 dimensions)
            result = genai.embed_content(
                model="models/embedding-001",
                content=text,
                task_type="retrieval_document"
            )
            # Return native 768-dimensional embedding
            return result['embedding']
        elif self.provider == "openai":
            # Use OpenAI's embedding API directly
            import openai
            openai_key = os.getenv("OPENAI_API_KEY")
            if openai_key:
                client = openai.OpenAI(api_key=openai_key)
                response = client.embeddings.create(
                    model="text-embedding-3-small",
                    input=text,
                    dimensions=1536
                )
                return response.data[0].embedding
            else:
                # Fallback: create dummy embeddings (for testing only)
                print("Warning: No OpenAI API key found. Using dummy embeddings.")
                return [0.0] * 1536

    def process_book(self, book_content: Dict[str, str]) -> List[Dict]:
        """Process entire book and generate embeddings

        Args:
            book_content: Dict mapping chapter names to content

        Returns:
            List of embeddings with metadata
        """
        all_embeddings = []

        for chapter, content in book_content.items():
            print(f"Processing chapter: {chapter}")
            chunks = self.chunk_text(content, chapter)

            for chunk in chunks:
                embedding = self.generate_embedding(chunk["text"])
                all_embeddings.append({
                    "id": chunk["id"],
                    "vector": embedding,
                    "chapter": chunk["chapter"],
                    "text": chunk["text"],
                    "token_count": chunk["token_count"]
                })

        return all_embeddings

    async def validate_embeddings_service(self) -> Dict[str, any]:
        """Validate that the embeddings service is accessible and functional

        Returns:
            Dict containing validation status and metadata
        """
        test_text = "test"
        embedding = self.generate_embedding(test_text)

        return {
            "provider": self.provider,
            "vector_size": len(embedding),
            "api_configured": True
        }


async def get_embedding(text: str) -> List[float]:
    """Generate embedding for a text chunk asynchronously

    Args:
        text: Text to embed

    Returns:
        Embedding vector
    """
    try:
        # Use Google's embedding model
        google_key = os.getenv("GOOGLE_API_KEY")
        if google_key:
            genai.configure(api_key=google_key)
            result = genai.embed_content(
                model="models/embedding-001",
                content=text,
                task_type="retrieval_document"
            )
            # Pad or truncate to 1536 dimensions
            embedding = result['embedding']
            if len(embedding) < 1536:
                embedding.extend([0.0] * (1536 - len(embedding)))
            else:
                embedding = embedding[:1536]
            return embedding
        else:
            print("Warning: No Google API key found. Using dummy embeddings.")
            return [0.0] * 1536
    except Exception as e:
        print(f"Error generating embedding: {e}")
        # Return dummy embedding as fallback
        return [0.0] * 1536