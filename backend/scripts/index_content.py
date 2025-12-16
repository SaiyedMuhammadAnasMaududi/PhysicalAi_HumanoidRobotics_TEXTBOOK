#!/usr/bin/env python3
"""
Content Indexing Script for RAG Chatbot
Indexes book content into Qdrant vector database
"""

import os
import sys
import json
import asyncio
from pathlib import Path
from typing import List, Dict
import hashlib

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config import settings
from src.vector_db import get_qdrant_client, create_collection
from qdrant_client.models import PointStruct, VectorParams, Distance


class ContentIndexer:
    def __init__(self):
        self.qdrant_client = None
        self.embedding_client = None
        self.collection_name = "book_content"

        # Determine which embedding provider to use
        self.embedding_provider = settings.embedding_provider
        print(f"🔧 Using {self.embedding_provider} for embeddings")

        if self.embedding_provider == "local":
            from sentence_transformers import SentenceTransformer
            self.embedding_model = settings.embedding_model  # Default: all-MiniLM-L6-v2
            print(f"📦 Loading local model: {self.embedding_model}")
            self.embedding_client = SentenceTransformer(self.embedding_model)
            self.vector_size = self.embedding_client.get_sentence_embedding_dimension()
            print(f"✅ Model loaded (dimension: {self.vector_size})")
        elif self.embedding_provider == "qwen":
            from openai import OpenAI
            self.embedding_client = OpenAI(
                api_key=settings.qwen_api_key,
                base_url=settings.qwen_base_url
            )
            self.embedding_model = settings.embedding_model  # Default: text-embedding-v3
            self.vector_size = 1024  # Qwen text-embedding-v3 dimension
        elif self.embedding_provider == "openrouter":
            from litellm import embedding
            self.embedding_fn = embedding
            self.embedding_model = settings.embedding_model
            self.vector_size = 1536  # Common size for OpenRouter models
            # Set environment variables for litellm
            os.environ["OPENROUTER_API_KEY"] = settings.openrouter_api_key
        else:  # Default to Gemini
            import google.generativeai as genai
            genai.configure(api_key=settings.gemini_api_key)
            self.genai = genai
            self.embedding_model = "models/embedding-001"
            self.vector_size = 768

    async def initialize(self):
        """Initialize Qdrant client and collection"""
        print("🔌 Connecting to Qdrant...")
        self.qdrant_client = get_qdrant_client()

        # Create collection if it doesn't exist
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            print(f"✅ Collection '{self.collection_name}' already exists")
        except Exception:
            print(f"📦 Creating collection '{self.collection_name}'...")
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Collection created")

    def get_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using configured provider"""
        if self.embedding_provider == "local":
            # Use SentenceTransformer for local embeddings
            embedding = self.embedding_client.encode(text, convert_to_tensor=False)
            return embedding.tolist()
        elif self.embedding_provider == "qwen":
            # Use OpenAI-compatible client for Qwen
            response = self.embedding_client.embeddings.create(
                model=self.embedding_model,
                input=text
            )
            return response.data[0].embedding
        elif self.embedding_provider == "openrouter":
            # OpenRouter requires api_key and api_base to be set
            response = self.embedding_fn(
                model=self.embedding_model,  # Don't add "openrouter/" prefix
                input=[text],
                api_key=settings.openrouter_api_key,
                api_base=settings.openrouter_base_url
            )
            return response['data'][0]['embedding']
        else:  # Gemini
            result = self.genai.embed_content(
                model=self.embedding_model,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']

    def find_book_content(self, docs_dir: str = "../docs") -> List[Dict]:
        """Find all markdown files in docs directory"""
        # Get the project root (parent of backend directory)
        # __file__ is backend/scripts/index_content.py
        # parent is backend/scripts, parent.parent is backend, parent.parent.parent is project root
        script_path = Path(__file__).resolve()
        project_root = script_path.parent.parent.parent
        docs_path = project_root / "docs"

        print(f"📁 Script location: {script_path}")
        print(f"📁 Project root: {project_root}")
        print(f"📁 Looking for docs at: {docs_path}")

        if not docs_path.exists():
            print(f"❌ Docs directory not found: {docs_path}")
            return []

        content_files = []
        for md_file in docs_path.rglob("*.md"):
            # Skip certain files
            if md_file.name.startswith("_") or "node_modules" in str(md_file):
                continue

            content_files.append({
                "path": str(md_file),
                "relative_path": str(md_file.relative_to(docs_path)),
                "name": md_file.stem
            })

        print(f"📚 Found {len(content_files)} content files")
        return content_files

    def chunk_content(self, text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
        """Split content into overlapping chunks"""
        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk = text[start:end]

            # Try to break at sentence boundary
            if end < len(text):
                last_period = chunk.rfind('. ')
                if last_period > chunk_size * 0.7:  # At least 70% of chunk size
                    end = start + last_period + 1
                    chunk = text[start:end]

            chunks.append(chunk.strip())
            start = end - overlap

        return chunks

    def process_markdown_file(self, file_path: str) -> List[Dict]:
        """Process a markdown file and extract content chunks"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Remove frontmatter
            if content.startswith('---'):
                parts = content.split('---', 2)
                if len(parts) >= 3:
                    content = parts[2]

            # Chunk the content
            chunks = self.chunk_content(content)

            results = []
            for i, chunk in enumerate(chunks):
                # Create unique ID for this chunk
                chunk_id = hashlib.md5(f"{file_path}_{i}".encode()).hexdigest()

                results.append({
                    "id": chunk_id,
                    "text": chunk,
                    "metadata": {
                        "source": file_path,
                        "chunk_index": i,
                        "total_chunks": len(chunks)
                    }
                })

            return results

        except Exception as e:
            print(f"❌ Error processing {file_path}: {e}")
            return []

    async def index_content(self):
        """Index all book content into Qdrant"""
        print("\n📖 Finding book content...")
        content_files = self.find_book_content()

        if not content_files:
            print("❌ No content files found")
            return

        all_points = []
        total_chunks = 0

        print("\n🔨 Processing content files...")
        for file_info in content_files:
            print(f"  Processing: {file_info['relative_path']}")
            chunks = self.process_markdown_file(file_info['path'])

            for chunk_data in chunks:
                try:
                    # Generate embedding
                    embedding = self.get_embedding(chunk_data['text'])

                    # Create point
                    point = PointStruct(
                        id=chunk_data['id'],
                        vector=embedding,
                        payload={
                            "text": chunk_data['text'],
                            "source": chunk_data['metadata']['source'],
                            "chunk_index": chunk_data['metadata']['chunk_index'],
                            "total_chunks": chunk_data['metadata']['total_chunks'],
                            "file_name": file_info['name'],
                            "relative_path": file_info['relative_path']
                        }
                    )
                    all_points.append(point)
                    total_chunks += 1

                except Exception as e:
                    print(f"    ❌ Error generating embedding: {e}")
                    continue

        if all_points:
            print(f"\n📤 Uploading {total_chunks} chunks to Qdrant in batches...")
            batch_size = 50  # Upload 50 chunks at a time
            total_uploaded = 0

            for i in range(0, len(all_points), batch_size):
                batch = all_points[i:i + batch_size]
                try:
                    self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=batch
                    )
                    total_uploaded += len(batch)
                    print(f"  ✅ Uploaded batch {i//batch_size + 1}/{(len(all_points) + batch_size - 1)//batch_size} ({total_uploaded}/{len(all_points)} chunks)")
                except Exception as e:
                    print(f"  ❌ Error uploading batch: {e}")
                    continue

            print(f"✅ Successfully indexed {total_uploaded} content chunks")

            # Verify
            try:
                collection_info = self.qdrant_client.get_collection(self.collection_name)
                print(f"📊 Collection stats: {collection_info.points_count} total points")
            except Exception as e:
                print(f"❌ Error getting collection info: {e}")
        else:
            print("❌ No content chunks to index")

    async def verify_indexing(self):
        """Verify that content is properly indexed"""
        print("\n🔍 Verifying indexing...")

        try:
            # Test query
            test_query = "What is Physical AI?"
            embedding = self.get_embedding(test_query)

            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=embedding,
                limit=3
            )

            print(f"\n✅ Retrieval test successful!")
            print(f"Query: '{test_query}'")
            print(f"Found {len(results)} relevant chunks:")

            for i, result in enumerate(results, 1):
                print(f"\n{i}. Score: {result.score:.4f}")
                print(f"   Source: {result.payload.get('relative_path', 'Unknown')}")
                print(f"   Preview: {result.payload.get('text', '')[:100]}...")

        except Exception as e:
            print(f"❌ Verification failed: {e}")


async def main():
    """Main function"""
    print("=" * 60)
    print("📚 RAG Chatbot Content Indexer")
    print("=" * 60)

    indexer = ContentIndexer()

    try:
        await indexer.initialize()
        await indexer.index_content()
        await indexer.verify_indexing()

        print("\n" + "=" * 60)
        print("🎉 Content indexing complete!")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
