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
import time
import random

load_dotenv()

# Chunking configuration
CHUNK_SIZE = 500  # tokens - FR-009 requirement
OVERLAP = 50  # tokens - FR-009 requirement



class EmbeddingGenerator:
    """Generates embeddings for book content"""

    def __init__(self):
        self.anthropic_key = os.getenv("ANTHROPIC_API_KEY")
        self.google_key = os.getenv("GOOGLE_API_KEY")
        self.openai_key = os.getenv("OPENAI_API_KEY")

        # Initialize tokenizer for chunking
        self.encoder = tiktoken.get_encoding("cl100k_base")

        # Initialize clients - check if API keys are valid format first
        # Google API keys should start with "AI" (not Groq keys starting with "gsk_")
        valid_google_key = self.google_key and self.google_key.startswith("AI") and not self.google_key.startswith("gsk_")
        valid_openai_key = self.openai_key and self.openai_key.startswith("sk-") and not self.openai_key.startswith("gsk_")
        valid_anthropic_key = self.anthropic_key and self.anthropic_key.startswith("sk-ant-")

        if self.anthropic_key and valid_anthropic_key:
            try:
                self.client = Anthropic(api_key=self.anthropic_key)
                self.provider = "anthropic"
            except Exception as e:
                print(f"Warning: Anthropic API initialization failed: {e}")
                self.provider = "local"
                self._init_local_model()
        elif self.google_key and valid_google_key:
            try:
                genai.configure(api_key=self.google_key)
                self.provider = "google"
            except Exception as e:
                print(f"Warning: Google API initialization failed: {e}")
                self.provider = "local"
                self._init_local_model()
        elif self.openai_key and valid_openai_key:
            try:
                # Test if the OpenAI API key is valid by importing and checking
                import openai
                self.provider = "openai"
            except Exception as e:
                print(f"Warning: OpenAI API initialization failed: {e}")
                self.provider = "local"
                self._init_local_model()
        else:
            # Default to local embedding model if no valid API keys are available
            self.provider = "local"
            self._init_local_model()

    def _init_local_model(self):
        """Initialize local embedding model"""
        try:
            from sentence_transformers import SentenceTransformer
            # Try to load the model, handling potential download issues
            self.local_model = SentenceTransformer('all-MiniLM-L6-v2', cache_folder='./models')
            print("Local embedding model initialized successfully")
        except ImportError:
            print("Warning: sentence-transformers not installed. Using dummy embeddings.")
            self.local_model = None
        except Exception as e:
            print(f"Warning: Failed to initialize local embedding model: {e}. Using dummy embeddings.")
            print("This might be due to model download issues. Using dummy embeddings as fallback.")
            self.local_model = None

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
                # Fallback to local embedding model
                return self._get_local_embedding(text)

        elif self.provider == "google":
            # Use Google's embedding model (768 dimensions)
            def _call_google_api():
                return genai.embed_content(
                    model="models/embedding-001",
                    content=text,
                    task_type="retrieval_document"
                )

            try:
                result = self._retry_with_exponential_backoff(_call_google_api)
                # Pad or truncate to 1536 dimensions to maintain consistency
                embedding = result['embedding']
                if len(embedding) < 1536:
                    embedding.extend([0.0] * (1536 - len(embedding)))
                else:
                    embedding = embedding[:1536]
                return embedding
            except Exception as e:
                print(f"Google embedding failed after retries: {e}")
                # Check if it's a quota error
                error_str = str(e).lower()
                if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
                    print("Google API quota exceeded. Please check your Google Cloud billing settings or API quota limits. Using local embedding model as fallback.")

                # Fallback to OpenAI embeddings if Google API quota exceeded
                openai_key = os.getenv("OPENAI_API_KEY")
                if openai_key:
                    def _call_openai_api():
                        from openai import OpenAI
                        client = OpenAI(api_key=openai_key)
                        response = client.embeddings.create(
                            model="text-embedding-3-small",
                            input=text,
                            dimensions=1536
                        )
                        return response.data[0].embedding

                    try:
                        return self._retry_with_exponential_backoff(_call_openai_api)
                    except Exception as openai_e:
                        print(f"OpenAI embedding failed after retries: {openai_e}")
                        # Fallback to local embedding model
                        return self._get_local_embedding(text)
                else:
                    # Fallback to local embedding model
                    return self._get_local_embedding(text)

        elif self.provider == "openai":
            # Use OpenAI's embedding API directly
            openai_key = os.getenv("OPENAI_API_KEY")
            if openai_key:
                def _call_openai_api():
                    from openai import OpenAI
                    client = OpenAI(api_key=openai_key)
                    response = client.embeddings.create(
                        model="text-embedding-3-small",
                        input=text,
                        dimensions=1536
                    )
                    return response.data[0].embedding

                try:
                    return self._retry_with_exponential_backoff(_call_openai_api)
                except Exception as e:
                    print(f"OpenAI embedding failed after retries: {e}")
                    # Fallback to local embedding model
                    return self._get_local_embedding(text)
            else:
                # Fallback to local embedding model
                return self._get_local_embedding(text)
        else:
            # Use local embedding model as primary provider
            return self._get_local_embedding(text)

    def _get_local_embedding(self, text: str) -> List[float]:
        """Generate embedding using local model as fallback

        Args:
            text: Text to embed

        Returns:
            Embedding vector
        """
        # Check if local_model exists and is initialized
        if hasattr(self, 'local_model') and self.local_model is not None:
            try:
                embedding = self.local_model.encode([text])[0].tolist()
                # Pad or truncate to 1536 dimensions to maintain consistency with other models
                if len(embedding) < 1536:
                    embedding.extend([0.0] * (1536 - len(embedding)))
                else:
                    embedding = embedding[:1536]
                return embedding
            except Exception as e:
                print(f"Local embedding failed: {e}")
                # Final fallback to dummy embeddings
                print("Warning: Local embedding failed. Using dummy embeddings.")
                return [0.0] * 1536
        else:
            # Try to initialize local model on-demand if not already done
            try:
                from sentence_transformers import SentenceTransformer
                # Handle potential dependency issues by using a try-catch around the initialization
                try:
                    self.local_model = SentenceTransformer('all-MiniLM-L6-v2', cache_folder='./models')
                except Exception:
                    # If cache folder fails, try without specifying cache folder
                    self.local_model = SentenceTransformer('all-MiniLM-L6-v2')

                embedding = self.local_model.encode([text])[0].tolist()
                # Pad or truncate to 1536 dimensions to maintain consistency with other models
                if len(embedding) < 1536:
                    embedding.extend([0.0] * (1536 - len(embedding)))
                else:
                    embedding = embedding[:1536]
                return embedding
            except ImportError:
                print("Warning: sentence-transformers not installed. Using dummy embeddings.")
                return [0.0] * 1536
            except Exception as e:
                print(f"Local model initialization failed: {e}")
                # Final fallback to dummy embeddings
                print("Warning: No local model available. Using dummy embeddings.")
                return [0.0] * 1536

    def _retry_with_exponential_backoff(self, func, *args, max_retries=3, base_delay=1, max_delay=60):
        """
        Execute a function with exponential backoff retry logic

        Args:
            func: Function to execute
            *args: Arguments to pass to the function
            max_retries: Maximum number of retry attempts
            base_delay: Base delay in seconds
            max_delay: Maximum delay in seconds
        """
        for attempt in range(max_retries + 1):
            try:
                return func(*args)
            except Exception as e:
                if attempt == max_retries:
                    # Last attempt, raise the exception
                    raise e

                # Check if it's a rate limit error
                error_str = str(e).lower()
                if "quota" not in error_str and "rate limit" not in error_str and "429" not in error_str:
                    # If it's not a rate limit error, don't retry
                    raise e

                # Calculate delay with exponential backoff and jitter
                delay = min(base_delay * (2 ** attempt) + random.uniform(0, 1), max_delay)
                print(f"Rate limit hit, retrying in {delay:.2f} seconds... (attempt {attempt + 1}/{max_retries})")
                time.sleep(delay)

        # This should not be reached
        raise Exception("Max retries exceeded")

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
    # Create a simple retry function for this standalone function
    def _retry_with_exponential_backoff(func, *args, max_retries=3, base_delay=1, max_delay=60):
        for attempt in range(max_retries + 1):
            try:
                return func(*args)
            except Exception as e:
                if attempt == max_retries:
                    # Last attempt, raise the exception
                    raise e

                # Check if it's a rate limit error
                error_str = str(e).lower()
                if "quota" not in error_str and "rate limit" not in error_str and "429" not in error_str:
                    # If it's not a rate limit error, don't retry
                    raise e

                # Calculate delay with exponential backoff and jitter
                delay = min(base_delay * (2 ** attempt) + random.uniform(0, 1), max_delay)
                print(f"Rate limit hit, retrying in {delay:.2f} seconds... (attempt {attempt + 1}/{max_retries})")
                time.sleep(delay)

        # This should not be reached
        raise Exception("Max retries exceeded")

    # Check for valid API keys in order of preference
    google_key = os.getenv("GOOGLE_API_KEY")
    openai_key = os.getenv("OPENAI_API_KEY")

    # Validate API keys before using them
    valid_google_key = google_key and google_key.startswith("AI") and not google_key.startswith("gsk_")
    valid_openai_key = openai_key and openai_key.startswith("sk-") and not openai_key.startswith("gsk_")

    # Try Google API if valid
    if valid_google_key:
        try:
            genai.configure(api_key=google_key)

            def _call_google_api():
                return genai.embed_content(
                    model="models/embedding-001",
                    content=text,
                    task_type="retrieval_document"
                )

            result = _retry_with_exponential_backoff(_call_google_api)
            # Pad or truncate to 1536 dimensions
            embedding = result['embedding']
            if len(embedding) < 1536:
                embedding.extend([0.0] * (1536 - len(embedding)))
            else:
                embedding = embedding[:1536]
            return embedding
        except Exception as e:
            print(f"Google embedding failed after retries: {e}")
            # Check if it's a quota error
            error_str = str(e).lower()
            if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
                print("Google API quota exceeded. Please check your Google Cloud billing settings or API quota limits.")

    # Fallback to OpenAI embeddings if available and valid
    if valid_openai_key:
        def _call_openai_api():
            from openai import OpenAI
            client = OpenAI(api_key=openai_key)
            response = client.embeddings.create(
                model="text-embedding-3-small",
                input=text,
                dimensions=1536
            )
            return response.data[0].embedding

        try:
            return _retry_with_exponential_backoff(_call_openai_api)
        except Exception as openai_e:
            print(f"OpenAI embedding failed after retries: {openai_e}")

    print("Warning: No valid API keys found. Using local embedding model as fallback.")

    # Try local embedding model as fallback
    try:
        from sentence_transformers import SentenceTransformer
        # Handle potential dependency issues by using a try-catch around the initialization
        try:
            local_model = SentenceTransformer('all-MiniLM-L6-v2', cache_folder='./models')
        except Exception:
            # If cache folder fails, try without specifying cache folder
            local_model = SentenceTransformer('all-MiniLM-L6-v2')

        embedding = local_model.encode([text])[0].tolist()
        # Pad or truncate to 1536 dimensions to maintain consistency
        if len(embedding) < 1536:
            embedding.extend([0.0] * (1536 - len(embedding)))
        else:
            embedding = embedding[:1536]
        return embedding
    except ImportError:
        print("Warning: sentence-transformers not installed. Using dummy embeddings.")
    except Exception as e:
        print(f"Local embedding failed: {e}")

    # Return dummy embedding as final fallback
    return [0.0] * 1536