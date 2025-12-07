"""
Qdrant client configuration and utilities with proper error handling

Implements error handling for Qdrant database unavailability (Edge Case #106)
"""
import os
import logging
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse
from dotenv import load_dotenv
import uuid

from .schema import QdrantSchema, COLLECTION_NAME, VECTOR_SIZE, DISTANCE_METRIC

load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantManager:
    """Manages Qdrant vector database operations with proper error handling"""

    def __init__(self):
        host = os.getenv("QDRANT_HOST", "localhost")
        port = int(os.getenv("QDRANT_PORT", "6333"))
        api_key = os.getenv("QDRANT_API_KEY")
        url = os.getenv("QDRANT_URL")

        try:
            if url:
                # Use URL if provided (for cloud instances)
                self.client = QdrantClient(url=url, api_key=api_key)
            else:
                # Use host/port for local instance
                self.client = QdrantClient(host=host, port=port, api_key=api_key)
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            raise

        self.collection_name = QdrantSchema.COLLECTION_NAME

    def create_collection(self):
        """Create the book embeddings collection if it doesn't exist with proper error handling"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception:
            try:
                # Create collection with proper schema as required by FR-008
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=QdrantSchema.VECTOR_SIZE,
                        distance=models.Distance(QdrantSchema.DISTANCE_METRIC.upper())
                    ),
                    on_disk_payload=True  # For better performance with large payloads
                )
                logger.info(f"Created collection '{self.collection_name}' with FR-008 schema")
            except UnexpectedResponse as e:
                logger.error(f"Failed to create collection: {e}")
                raise
            except Exception as e:
                logger.error(f"Unexpected error creating collection: {e}")
                raise

    def insert_embeddings(self, embeddings: List[Dict]):
        """Insert embeddings into Qdrant with proper error handling

        Args:
            embeddings: List of dicts with keys: id, vector, chapter, text, token_count, book_id
        """
        try:
            points = []
            for emb in embeddings:
                # Use the schema-defined payload fields from FR-008
                payload = {
                    "id": emb.get("id", str(uuid.uuid4())),
                    "chapter": emb["chapter"],
                    "text": emb["text"],
                    "token_count": emb["token_count"],
                    "book_id": emb.get("book_id", "default")  # Added for organization
                }

                point = PointStruct(
                    id=emb.get("id", str(uuid.uuid4())),
                    vector=emb["vector"],
                    payload=payload
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Successfully inserted {len(points)} embeddings")
        except UnexpectedResponse as e:
            logger.error(f"Failed to insert embeddings: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error inserting embeddings: {e}")
            raise

    def search(self, query_vector: List[float], limit: int = 5, book_id: Optional[str] = None):
        """Search for similar embeddings with proper error handling and book filtering

        Args:
            query_vector: Query embedding vector
            limit: Number of results to return
            book_id: Optional book identifier to filter results

        Returns:
            List of search results with scores and payloads
        """
        try:
            # Build search filter if book_id is specified
            search_filter = None
            if book_id:
                search_filter = models.Filter(
                    must=[
                        FieldCondition(
                            key="book_id",
                            match=MatchValue(value=book_id)
                        )
                    ]
                )

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                query_filter=search_filter
            )
            return results
        except UnexpectedResponse as e:
            logger.error(f"Search failed: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error during search: {e}")
            raise

    def delete_collection(self):
        """Delete the collection with proper error handling (useful for testing)"""
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Successfully deleted collection '{self.collection_name}'")
        except UnexpectedResponse as e:
            logger.error(f"Failed to delete collection: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error deleting collection: {e}")
            raise

    def health_check(self) -> bool:
        """Check if Qdrant is accessible and responding"""
        try:
            # Try to get collection list as a basic health check
            self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False

    def get_collection_info(self):
        """Get collection information with error handling"""
        try:
            return self.client.get_collection(self.collection_name)
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise