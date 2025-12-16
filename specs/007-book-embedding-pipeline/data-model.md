# Data Model: Book Content Embedding Pipeline

**Date**: 2025-12-16
**Feature**: 007-book-embedding-pipeline
**Model Version**: 1.0

## Core Entities

### Document Chunk
**Description**: Represents a semantically coherent segment of book content
**Fields**:
- `chunk_id` (string): Unique identifier for the chunk (UUID or path-based)
- `source_file` (string): Relative path to the source markdown file
- `section_title` (string): Title or heading of the section containing this chunk
- `content` (string): The actual text content of the chunk
- `content_hash` (string): SHA256 hash of the content for deduplication
- `chunk_sequence` (integer): Position of this chunk within the document
- `total_chunks` (integer): Total number of chunks from the original document
- `processing_timestamp` (datetime): When this chunk was processed
- `token_count` (integer): Number of tokens in the content

**Validation Rules**:
- Content must not be empty
- Source file must be a valid markdown file path
- Chunk sequence must be non-negative
- Token count must be positive

### Embedding Vector
**Description**: Represents the semantic representation of a document chunk
**Fields**:
- `vector_id` (string): Unique identifier matching the chunk_id
- `embedding` (array[float]): High-dimensional vector representation from Cohere
- `model_version` (string): Version of the embedding model used
- `dimensionality` (integer): Number of dimensions in the embedding vector
- `processing_timestamp` (datetime): When the embedding was generated

**Validation Rules**:
- Embedding vector must have consistent dimensionality
- Model version must match the configured Cohere model
- Vector ID must correspond to a valid document chunk

### Metadata Record
**Description**: Contains information about source documents and processing
**Fields**:
- `record_id` (string): Unique identifier for the metadata record
- `source_file` (string): Path to the original source file
- `document_title` (string): Title of the document
- `document_size` (integer): Size of the original document in bytes
- `total_chunks` (integer): Number of chunks created from this document
- `processing_status` (enum): Status of processing (pending, in_progress, completed, failed)
- `start_timestamp` (datetime): When processing started
- `end_timestamp` (datetime): When processing completed
- `error_message` (string, optional): Error details if processing failed

**Validation Rules**:
- Source file must exist and be accessible
- Processing status must be one of the defined enum values
- End timestamp must be after start timestamp when processing is completed

## Relationships

### Document Chunk → Embedding Vector (1:1)
- Each document chunk produces exactly one embedding vector
- Linked by matching chunk_id to vector_id

### Source File → Document Chunks (1:many)
- Each source file can be split into multiple document chunks
- Related through source_file field

### Source File → Metadata Record (1:1)
- Each source file has one metadata record tracking its processing
- Related through source_file field

## Qdrant Collection Schema

### Collection Name: `physicalai`

**Vector Configuration**:
- Vector size: 1024 (for Cohere embed-multilingual-v3.0)
- Distance metric: Cosine
- Collection name: `physicalai`

**Payload Schema**:
```json
{
  "chunk_id": "keyword",
  "source_file": "keyword",
  "section_title": "text",
  "content": "text",
  "content_hash": "keyword",
  "chunk_sequence": "integer",
  "total_chunks": "integer",
  "processing_timestamp": "datetime",
  "token_count": "integer",
  "model_version": "keyword"
}
```

**Indexed Fields**:
- `source_file`: Fast filtering by document source
- `section_title`: Fast filtering by section
- `content_hash`: Deduplication support
- `processing_timestamp`: Time-based queries