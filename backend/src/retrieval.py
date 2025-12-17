#!/usr/bin/env python3
"""
Retrieval & Query Validation Module for RAG Chatbot

This module provides semantic search capabilities over embedded book content,
retrieving relevant chunks from the Qdrant vector database based on natural-language queries.
"""

import os
import logging
from typing import List, Dict, Any
from dataclasses import dataclass
from datetime import datetime
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


@dataclass
class Query:
    """Represents a natural-language search query"""
    query_text: str
    timestamp: datetime
    top_k: int = 5
    similarity_threshold: float = 0.0


@dataclass
class RetrievalResult:
    """Document chunk with similarity scoring"""
    # From DocumentChunk (Spec 1)
    chunk_id: str
    source_file: str
    section_title: str
    content: str
    content_hash: str
    chunk_sequence: int
    total_chunks: int
    processing_timestamp: datetime
    token_count: int

    # New for retrieval
    similarity_score: float
    rank: int


@dataclass
class RetrievalResponse:
    """Complete response to a semantic search query"""
    query_text: str
    results: List[RetrievalResult]
    total_results: int
    execution_time_ms: float
    timestamp: datetime
    parameters: Dict[str, Any]


class QueryEmbedder:
    """Generates embeddings for natural-language queries using Cohere"""

    def __init__(self, api_key: str, model: str = "embed-multilingual-v3.0"):
        self.client = cohere.Client(api_key)
        self.model = model

    def generate_embedding(self, query_text: str) -> List[float]:
        """Generate embedding for a query using Cohere with input_type='search_query'"""
        if not query_text or not query_text.strip():
            raise ValueError("Query cannot be empty")

        try:
            response = self.client.embed(
                texts=[query_text],
                model=self.model,
                input_type="search_query"  # Optimized for queries
            )
            return response.embeddings[0]
        except Exception as e:
            logging.error(f"Cohere API error: {e}")
            raise Exception(f"Failed to generate query embedding: {e}")


class VectorRetriever:
    """Searches Qdrant collection for similar chunks"""

    def __init__(self, url: str, api_key: str, collection_name: str = "physicalai"):
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name

    def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        similarity_threshold: float = 0.0
    ) -> List[models.ScoredPoint]:
        """Search Qdrant for similar chunks using cosine similarity"""
        try:
            response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,
                with_payload=True,
                with_vectors=False  # Don't return vectors (saves bandwidth)
            )
            return response.points
        except Exception as e:
            logging.error(f"Qdrant connection error: {e}")
            raise ConnectionError(f"Failed to search Qdrant: {e}")


class ResultFormatter:
    """Converts Qdrant ScoredPoints to RetrievalResult objects"""

    @staticmethod
    def format_results(
        scored_points: List[models.ScoredPoint],
        query_text: str
    ) -> List[RetrievalResult]:
        """Convert Qdrant results to RetrievalResult objects with ranking"""
        results = []

        for rank, point in enumerate(scored_points, start=1):
            payload = point.payload

            # Parse processing_timestamp
            processing_timestamp = datetime.fromisoformat(payload.get('processing_timestamp'))

            result = RetrievalResult(
                # From DocumentChunk
                chunk_id=payload.get('chunk_id'),
                source_file=payload.get('source_file'),
                section_title=payload.get('section_title'),
                content=payload.get('content'),
                content_hash=payload.get('content_hash'),
                chunk_sequence=payload.get('chunk_sequence'),
                total_chunks=payload.get('total_chunks'),
                processing_timestamp=processing_timestamp,
                token_count=payload.get('token_count'),

                # New for retrieval
                similarity_score=point.score,  # Already normalized by Qdrant
                rank=rank
            )
            results.append(result)

        return results


class RetrievalPipeline:
    """Main pipeline for semantic retrieval and query validation"""

    def __init__(
        self,
        cohere_api_key: str = None,
        qdrant_url: str = None,
        qdrant_api_key: str = None,
        collection_name: str = "physicalai",
        embedding_model: str = "embed-multilingual-v3.0"
    ):
        """Initialize retrieval pipeline with API clients"""
        # Use environment variables if not provided
        self.cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = collection_name or os.getenv("QDRANT_COLLECTION", "physicalai")
        self.embedding_model = embedding_model or os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0")

        # Initialize components
        self.embedder = QueryEmbedder(self.cohere_api_key, self.embedding_model)
        self.retriever = VectorRetriever(self.qdrant_url, self.qdrant_api_key, self.collection_name)
        self.formatter = ResultFormatter()

    def query(
        self,
        query_text: str,
        top_k: int = 5,
        similarity_threshold: float = 0.0
    ) -> RetrievalResponse:
        """
        Execute semantic search query against the vector database.

        Args:
            query_text: Natural-language query string
            top_k: Maximum number of results to return
            similarity_threshold: Minimum similarity score (0.0-1.0)

        Returns:
            RetrievalResponse with ordered results and metadata

        Raises:
            ValueError: If query_text is empty
            ConnectionError: If Qdrant is unreachable
            Exception: If Cohere API fails
        """
        start_time = datetime.now()

        # Phase 1: Validate query
        if not query_text or not query_text.strip():
            raise ValueError("Query cannot be empty. Please provide a non-empty query string.")

        # Phase 2: Generate query embedding
        query_embedding = self.embedder.generate_embedding(query_text)

        # Phase 3: Search Qdrant for similar chunks
        scored_points = self.retriever.search(
            query_embedding=query_embedding,
            top_k=top_k,
            similarity_threshold=similarity_threshold
        )

        # Phase 4: Format results
        results = self.formatter.format_results(scored_points, query_text)

        # Calculate execution time
        end_time = datetime.now()
        execution_time_ms = (end_time - start_time).total_seconds() * 1000

        # Build response
        response = RetrievalResponse(
            query_text=query_text,
            results=results,
            total_results=len(results),
            execution_time_ms=execution_time_ms,
            timestamp=end_time,
            parameters={
                "top_k": top_k,
                "similarity_threshold": similarity_threshold,
                "collection_name": self.collection_name,
                "embedding_model": self.embedding_model
            }
        )

        return response
