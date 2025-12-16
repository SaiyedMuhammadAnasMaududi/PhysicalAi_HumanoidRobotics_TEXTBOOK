"""
Qdrant client connection and configuration
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from src.config import settings


class VectorDBService:
    def __init__(self):
        # Get Qdrant configuration from settings
        self.qdrant_url = settings.qdrant_url
        self.qdrant_api_key = settings.qdrant_api_key

        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            # For local development, you might use:
            # host="localhost", port=6333
        )

        # Define collection name
        self.collection_name = "book_content"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """Initialize the Qdrant collection for book content"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with appropriate vector configuration
                # Assuming we're using embeddings with 1536 dimensions (like OpenAI ada-002)
                # Adjust dimensions based on your actual embedding model
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Adjust based on your embedding model
                        distance=models.Distance.COSINE
                    )
                )

                print(f"Created Qdrant collection: {self.collection_name}")
            else:
                print(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            print(f"Error initializing Qdrant collection: {e}")
            raise

    async def index_content(self, content: Any, embedding: List[float]) -> bool:
        """Index book content with its embedding"""
        try:
            # Prepare the point for Qdrant
            point = models.PointStruct(
                id=content.content_id,
                vector=embedding,
                payload={
                    "chapter": content.chapter,
                    "section": content.section,
                    "content_text": content.content_text,
                    "version": content.version,
                    "metadata": content.metadata or {}
                }
            )

            # Upload the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            return True
        except Exception as e:
            print(f"Error indexing content in Qdrant: {e}")
            return False

    async def search_content(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant content based on query embedding"""
        try:
            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            results = []
            for hit in search_results:
                results.append({
                    "content_id": hit.id,
                    "content_text": hit.payload.get("content_text", ""),
                    "chapter": hit.payload.get("chapter", ""),
                    "section": hit.payload.get("section", ""),
                    "similarity_score": hit.score,
                    "metadata": hit.payload.get("metadata", {})
                })

            return results
        except Exception as e:
            print(f"Error searching content in Qdrant: {e}")
            return []

    async def delete_content(self, content_id: str) -> bool:
        """Delete content from the vector database"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
            return True
        except Exception as e:
            print(f"Error deleting content from Qdrant: {e}")
            return False

    def health_check(self) -> bool:
        """Check if Qdrant is accessible"""
        try:
            self.client.get_collection(self.collection_name)
            return True
        except Exception:
            return False


# Helper functions for easy access
def get_qdrant_client() -> QdrantClient:
    """Get a configured Qdrant client instance"""
    return QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )


def create_collection(client: QdrantClient, collection_name: str, vector_size: int = 768):
    """Create a Qdrant collection with specified configuration"""
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection: {collection_name}")
        else:
            print(f"Collection {collection_name} already exists")
    except Exception as e:
        print(f"Error creating collection: {e}")
        raise