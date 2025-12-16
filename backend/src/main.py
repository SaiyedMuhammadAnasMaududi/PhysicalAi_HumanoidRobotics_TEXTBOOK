#!/usr/bin/env python3
"""
Book Content Embedding & Vector Storage Pipeline for RAG Chatbot

This script processes markdown files from the /docs directory, generates semantic embeddings
using Cohere, and stores them in a Qdrant vector database collection named 'physicalai'.
"""

import os
import re
import asyncio
import hashlib
import logging
import uuid
from typing import List, Dict, Any, Optional, Tuple
from pathlib import Path
from dataclasses import dataclass
from datetime import datetime
import markdown
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import cohere
from dotenv import load_dotenv
import yaml


# Load environment variables
load_dotenv()


@dataclass
class DocumentChunk:
    """Represents a semantically coherent segment of book content"""
    chunk_id: str
    source_file: str
    section_title: str
    content: str
    content_hash: str
    chunk_sequence: int
    total_chunks: int
    processing_timestamp: datetime
    token_count: int


class DocumentIngestor:
    """Handles scanning and parsing markdown files from the /docs directory"""

    def __init__(self, docs_path: str = "./docs"):
        self.docs_path = Path(docs_path)

    def scan_directory(self) -> List[Path]:
        """Scan the docs directory recursively for markdown files"""
        if not self.docs_path.exists():
            raise FileNotFoundError(f"Docs directory does not exist: {self.docs_path}")

        markdown_files = []
        for file_path in self.docs_path.rglob("*.md"):
            if file_path.is_file():
                markdown_files.append(file_path)

        logging.info(f"Found {len(markdown_files)} markdown files in {self.docs_path}")
        return markdown_files

    def parse_markdown(self, file_path: Path) -> str:
        """Parse markdown file and extract plain text content"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Convert markdown to HTML, then extract text
            html = markdown.markdown(content)
            # Remove HTML tags to get plain text
            text = re.sub(r'<[^>]+>', '', html)
            return text
        except Exception as e:
            logging.error(f"Error parsing markdown file {file_path}: {e}")
            return ""

    def extract_section_title(self, file_path: Path) -> str:
        """Extract a section title from the file path or content"""
        # Try to get the first heading from the file content
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Look for first heading in markdown
            lines = content.split('\n')
            for line in lines:
                if line.strip().startswith('#'):
                    # Remove markdown heading markers
                    title = line.strip().lstrip('#').strip()
                    if title:
                        return title

        except Exception as e:
            logging.warning(f"Could not extract section title from {file_path}: {e}")

        # Fallback to filename
        return file_path.stem


class Chunker:
    """Handles splitting documents into semantically coherent chunks"""

    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 50, strategy: str = "semantic"):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.strategy = strategy

    def chunk_content(self, content: str, source_file: str, section_title: str) -> List[DocumentChunk]:
        """Split content into chunks based on the specified strategy"""
        if self.strategy == "semantic":
            return self._semantic_chunking(content, source_file, section_title)
        else:
            return self._fixed_size_chunking(content, source_file, section_title)

    def _semantic_chunking(self, content: str, source_file: str, section_title: str) -> List[DocumentChunk]:
        """Split content at semantic boundaries like paragraphs and sections"""
        # Split by double newlines (paragraph boundaries)
        paragraphs = content.split('\n\n')

        chunks = []
        current_chunk = ""
        chunk_sequence = 0

        for paragraph in paragraphs:
            # Check if adding this paragraph would exceed chunk size
            if len(current_chunk) + len(paragraph) > self.chunk_size and current_chunk:
                # Finalize current chunk
                chunk = self._create_chunk(current_chunk.strip(), source_file, section_title, chunk_sequence)
                chunks.append(chunk)
                chunk_sequence += 1

                # Start new chunk with overlap
                if self.chunk_overlap > 0:
                    # Add overlap from end of current chunk
                    overlap_start = max(0, len(current_chunk) - self.chunk_overlap)
                    current_chunk = current_chunk[overlap_start:] + paragraph
                else:
                    current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunk = self._create_chunk(current_chunk.strip(), source_file, section_title, chunk_sequence)
            chunks.append(chunk)

        # Update total_chunks for each chunk
        for chunk in chunks:
            chunk.total_chunks = len(chunks)

        return chunks

    def _fixed_size_chunking(self, content: str, source_file: str, section_title: str) -> List[DocumentChunk]:
        """Split content into fixed-size chunks"""
        chunks = []
        chunk_sequence = 0

        for i in range(0, len(content), self.chunk_size - self.chunk_overlap):
            chunk_text = content[i:i + self.chunk_size]

            chunk = self._create_chunk(chunk_text, source_file, section_title, chunk_sequence)
            chunks.append(chunk)
            chunk_sequence += 1

        # Update total_chunks for each chunk
        for chunk in chunks:
            chunk.total_chunks = len(chunks)

        return chunks

    def _create_chunk(self, content: str, source_file: str, section_title: str, chunk_sequence: int) -> DocumentChunk:
        """Create a DocumentChunk object with computed fields"""
        content_hash = hashlib.sha256(content.encode()).hexdigest()
        token_count = len(content.split())  # Simple tokenization

        return DocumentChunk(
            chunk_id=str(uuid.uuid4()),
            source_file=str(source_file),
            section_title=section_title,
            content=content,
            content_hash=content_hash,
            chunk_sequence=chunk_sequence,
            total_chunks=0,  # Will be updated after all chunks are created
            processing_timestamp=datetime.now(),
            token_count=token_count
        )


class EmbeddingGenerator:
    """Handles generating embeddings using Cohere API"""

    def __init__(self, api_key: str, model: str = "embed-multilingual-v3.0"):
        self.client = cohere.Client(api_key)
        self.model = model

    def generate_embeddings(self, chunks: List[DocumentChunk], batch_size: int = 10) -> List[List[float]]:
        """Generate embeddings for a list of chunks in batches"""
        embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]
            texts = [chunk.content for chunk in batch]

            try:
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document"
                )

                embeddings.extend(response.embeddings)
                logging.info(f"Generated embeddings for batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}")

            except Exception as e:
                logging.error(f"Error generating embeddings for batch {i//batch_size + 1}: {e}")
                # Return empty embeddings for failed batch, but continue
                embeddings.extend([[]] * len(batch))

        return embeddings


class VectorStorage:
    """Handles storing embeddings in Qdrant database"""

    def __init__(self, url: Optional[str] = None, api_key: Optional[str] = None,
                 host: str = "localhost", port: int = 6333, collection_name: str = "physicalai"):
        if url:
            # Use cloud instance
            self.client = QdrantClient(url=url, api_key=api_key)
        else:
            # Use local instance
            self.client = QdrantClient(host=host, port=port)

        self.collection_name = collection_name

    def create_collection(self):
        """Create the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logging.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Create collection with 1024-dimensional vectors (for Cohere embeddings)
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
            )
            logging.info(f"Created collection '{self.collection_name}' with 1024-dimensional vectors")

    def store_embeddings(self, chunks: List[DocumentChunk], embeddings: List[List[float]]):
        """Store chunks and embeddings in Qdrant"""
        if len(chunks) != len(embeddings):
            raise ValueError(f"Number of chunks ({len(chunks)}) must match number of embeddings ({len(embeddings)})")

        # Prepare points for insertion
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            if len(embedding) == 0:  # Skip empty embeddings
                continue

            point = models.PointStruct(
                id=chunk.chunk_id,
                vector=embedding,
                payload={
                    "chunk_id": chunk.chunk_id,
                    "source_file": chunk.source_file,
                    "section_title": chunk.section_title,
                    "content": chunk.content,
                    "content_hash": chunk.content_hash,
                    "chunk_sequence": chunk.chunk_sequence,
                    "total_chunks": chunk.total_chunks,
                    "processing_timestamp": chunk.processing_timestamp.isoformat(),
                    "token_count": chunk.token_count,
                    "model_version": "embed-multilingual-v3.0"
                }
            )
            points.append(point)

        # Upload points to Qdrant in batches to avoid timeout
        batch_size = 100
        total_points = len(points)
        for i in range(0, total_points, batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            logging.info(f"Uploaded batch {i//batch_size + 1}/{(total_points + batch_size - 1)//batch_size} ({len(batch)} points)")

        logging.info(f"Stored {len(points)} embeddings in collection '{self.collection_name}'")

    def verify_storage(self) -> bool:
        """Verify that the collection has data"""
        try:
            count = self.client.count(collection_name=self.collection_name)
            logging.info(f"Collection '{self.collection_name}' contains {count.count} vectors")
            return count.count > 0
        except Exception as e:
            logging.error(f"Error verifying storage: {e}")
            return False


class EmbeddingPipeline:
    """Main pipeline class that orchestrates the entire process"""

    def __init__(self):
        # Configuration from environment variables
        self.docs_path = os.getenv("DOCS_PATH", "./docs")
        self.chunk_size = int(os.getenv("CHUNK_SIZE", "512"))
        self.chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "50"))
        self.chunking_strategy = os.getenv("CHUNKING_STRATEGY", "semantic")
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.embedding_model = os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0")
        self.batch_size = int(os.getenv("BATCH_SIZE", "10"))
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.qdrant_host = os.getenv("QDRANT_HOST", "localhost")
        self.qdrant_port = int(os.getenv("QDRANT_PORT", "6333"))
        self.collection_name = os.getenv("QDRANT_COLLECTION", "physicalai")

        # Initialize components
        self.ingestor = DocumentIngestor(self.docs_path)
        self.chunker = Chunker(self.chunk_size, self.chunk_overlap, self.chunking_strategy)
        self.embedding_gen = EmbeddingGenerator(self.cohere_api_key, self.embedding_model)
        self.storage = VectorStorage(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            host=self.qdrant_host,
            port=self.qdrant_port,
            collection_name=self.collection_name
        )

    def run(self):
        """Execute the full embedding pipeline"""
        logging.info("Starting embedding pipeline...")

        # Phase 1: Content Ingestion
        logging.info("Phase 1: Content Ingestion")
        markdown_files = self.ingestor.scan_directory()
        if not markdown_files:
            logging.warning("No markdown files found in the docs directory")
            return

        all_chunks = []
        for file_path in markdown_files:
            logging.info(f"Processing file: {file_path}")

            content = self.ingestor.parse_markdown(file_path)
            if not content.strip():
                logging.warning(f"Empty content in file: {file_path}")
                continue

            section_title = self.ingestor.extract_section_title(file_path)
            chunks = self.chunker.chunk_content(content, str(file_path), section_title)
            all_chunks.extend(chunks)

        logging.info(f"Generated {len(all_chunks)} chunks from {len(markdown_files)} files")

        if not all_chunks:
            logging.warning("No content chunks generated")
            return

        # Phase 2: Embedding Generation
        logging.info("Phase 2: Embedding Generation")
        embeddings = self.embedding_gen.generate_embeddings(all_chunks, self.batch_size)

        # Filter out empty embeddings and corresponding chunks
        valid_data = [(chunk, emb) for chunk, emb in zip(all_chunks, embeddings) if len(emb) > 0]
        if not valid_data:
            logging.error("No valid embeddings generated")
            return

        valid_chunks, valid_embeddings = zip(*valid_data)
        valid_chunks = list(valid_chunks)
        valid_embeddings = list(valid_embeddings)

        logging.info(f"Generated {len(valid_embeddings)} valid embeddings")

        # Phase 3: Vector Storage
        logging.info("Phase 3: Vector Storage")
        self.storage.create_collection()
        self.storage.store_embeddings(valid_chunks, valid_embeddings)

        # Phase 4: Verification
        logging.info("Phase 4: Verification")
        success = self.storage.verify_storage()

        if success:
            logging.info("Pipeline completed successfully!")
        else:
            logging.error("Pipeline completed but verification failed")


def main():
    """Main entry point"""
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # Create and run the pipeline
    pipeline = EmbeddingPipeline()

    try:
        pipeline.run()
    except Exception as e:
        logging.error(f"Pipeline failed with error: {e}")
        raise


if __name__ == "__main__":
    main()