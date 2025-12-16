"""
Unit tests for the embedding pipeline main.py
"""
import os
import tempfile
import pytest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
from src.main import DocumentIngestor, Chunker, EmbeddingGenerator, VectorStorage, EmbeddingPipeline, DocumentChunk


class TestDocumentIngestor:
    def test_scan_directory(self):
        """Test scanning directory for markdown files"""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create test markdown files
            test_dir = Path(temp_dir)
            (test_dir / "doc1.md").write_text("# Test Document 1\nContent here.")
            (test_dir / "doc2.md").write_text("# Test Document 2\nMore content.")
            (test_dir / "not_markdown.txt").write_text("Not a markdown file")

            ingestor = DocumentIngestor(str(test_dir))
            files = ingestor.scan_directory()

            # Should find 2 markdown files
            assert len(files) == 2
            file_names = [f.name for f in files]
            assert "doc1.md" in file_names
            assert "doc2.md" in file_names

    def test_parse_markdown(self):
        """Test parsing markdown content"""
        with tempfile.TemporaryDirectory() as temp_dir:
            test_file = Path(temp_dir) / "test.md"
            test_file.write_text("# Test Title\n\nThis is **bold** content.\n\nAnother paragraph.")

            ingestor = DocumentIngestor(temp_dir)
            content = ingestor.parse_markdown(test_file)

            # Should extract text content without markdown formatting
            assert "Test Title" in content
            assert "bold" in content
            assert "Another paragraph" in content

    def test_extract_section_title(self):
        """Test extracting section title from file"""
        with tempfile.TemporaryDirectory() as temp_dir:
            test_file = Path(temp_dir) / "test.md"
            test_file.write_text("# Main Title\n\nContent here.")

            ingestor = DocumentIngestor(temp_dir)
            title = ingestor.extract_section_title(test_file)

            assert title == "Main Title"


class TestChunker:
    def test_semantic_chunking(self):
        """Test semantic chunking strategy"""
        chunker = Chunker(chunk_size=100, chunk_overlap=10, strategy="semantic")

        content = "First paragraph with some content.\n\nSecond paragraph with more content.\n\nThird paragraph."
        chunks = chunker.chunk_content(content, "test.md", "Test Section")

        # Should create chunks based on paragraph boundaries
        assert len(chunks) >= 1
        assert all(len(chunk.content) <= 100 for chunk in chunks)

    def test_fixed_chunking(self):
        """Test fixed-size chunking strategy"""
        chunker = Chunker(chunk_size=20, chunk_overlap=5, strategy="fixed")

        content = "This is a longer piece of content that should be split into multiple chunks based on the fixed size."
        chunks = chunker.chunk_content(content, "test.md", "Test Section")

        # Should create chunks of approximately fixed size
        assert len(chunks) >= 1
        assert all(len(chunk.content) <= 25 for chunk in chunks)  # 20 + some buffer


class TestEmbeddingGenerator:
    @patch('cohere.Client')
    def test_generate_embeddings(self, mock_cohere_client):
        """Test embedding generation"""
        # Mock the Cohere client
        mock_client = Mock()
        mock_client.embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]])
        mock_cohere_client.return_value = mock_client

        generator = EmbeddingGenerator("test-api-key")
        chunks = [
            DocumentChunk(
                chunk_id="test1",
                source_file="test.md",
                section_title="Test",
                content="Test content 1",
                content_hash="hash1",
                chunk_sequence=0,
                total_chunks=2,
                processing_timestamp=None,
                token_count=3
            ),
            DocumentChunk(
                chunk_id="test2",
                source_file="test.md",
                section_title="Test",
                content="Test content 2",
                content_hash="hash2",
                chunk_sequence=1,
                total_chunks=2,
                processing_timestamp=None,
                token_count=3
            )
        ]

        embeddings = generator.generate_embeddings(chunks, batch_size=2)

        assert len(embeddings) == 2
        assert len(embeddings[0]) == 3  # 3-dimensional mock embedding


class TestVectorStorage:
    @patch('qdrant_client.QdrantClient')
    def test_store_embeddings(self, mock_qdrant_client):
        """Test storing embeddings in Qdrant"""
        mock_client = Mock()
        mock_qdrant_client.return_value = mock_client

        storage = VectorStorage(collection_name="test_collection")

        chunks = [
            DocumentChunk(
                chunk_id="test1",
                source_file="test.md",
                section_title="Test",
                content="Test content 1",
                content_hash="hash1",
                chunk_sequence=0,
                total_chunks=1,
                processing_timestamp=None,
                token_count=3
            )
        ]

        embeddings = [[0.1, 0.2, 0.3]]

        storage.store_embeddings(chunks, embeddings)

        # Verify that upsert was called
        mock_client.upsert.assert_called_once()


class TestEmbeddingPipeline:
    def test_pipeline_initialization(self):
        """Test pipeline initialization with environment variables"""
        # Test with default values
        pipeline = EmbeddingPipeline()

        assert pipeline.docs_path == "./docs"
        assert pipeline.chunk_size == 512
        assert pipeline.collection_name == "physicalai"