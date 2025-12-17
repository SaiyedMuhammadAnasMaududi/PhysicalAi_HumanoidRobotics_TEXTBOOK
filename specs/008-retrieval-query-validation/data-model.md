# Data Model: Retrieval & Query Validation

**Feature**: 008-retrieval-query-validation
**Date**: 2025-12-16
**Model Version**: 1.0

## Overview

This document defines the data structures for the retrieval and query validation system. The model extends the DocumentChunk entity from Spec 1 (007-book-embedding-pipeline) with query-specific entities and retrieval results.

## Core Entities

### Query

**Description**: Represents a natural-language search query submitted by a user/developer

**Fields**:
- `query_text` (string): The natural-language query string
- `timestamp` (datetime): When the query was submitted
- `top_k` (integer): Maximum number of results to return (default: 5)
- `similarity_threshold` (float): Minimum similarity score for results (default: 0.0, range: 0.0-1.0)

**Validation Rules**:
- `query_text` must not be empty or whitespace-only
- `top_k` must be positive integer (recommended: 1-20)
- `similarity_threshold` must be in range [0.0, 1.0]

**Example**:
```python
Query(
    query_text="What is ROS2?",
    timestamp=datetime(2025, 12, 16, 10, 30, 0),
    top_k=5,
    similarity_threshold=0.0
)
```

---

### RetrievalResult

**Description**: Represents a single document chunk retrieved in response to a query, with similarity scoring

**Fields**:

*Inherited from DocumentChunk (Spec 1)*:
- `chunk_id` (string): Unique UUID identifying the chunk
- `source_file` (string): Path to the original markdown file
- `section_title` (string): Title or heading of the section
- `content` (string): The actual text content of the chunk
- `content_hash` (string): SHA256 hash for deduplication
- `chunk_sequence` (integer): Position of chunk within the document
- `total_chunks` (integer): Total chunks from the source document
- `processing_timestamp` (datetime): When the chunk was originally processed
- `token_count` (integer): Number of tokens in the content

*New Fields for Retrieval*:
- `similarity_score` (float): Semantic similarity to the query (0.0-1.0, higher = more similar)
- `rank` (integer): 1-indexed position in the result list (1 = best match)

**Validation Rules**:
- All DocumentChunk validation rules apply
- `similarity_score` must be in range [0.0, 1.0]
- `rank` must be positive integer
- Results should be ordered by `similarity_score` descending (rank 1 has highest score)

**Example**:
```python
RetrievalResult(
    # From DocumentChunk
    chunk_id="550e8400-e29b-41d4-a716-446655440000",
    source_file="../docs/module-01-ros2/01-introduction.md",
    section_title="What is ROS2?",
    content="ROS2 (Robot Operating System 2) is an open-source...",
    content_hash="a1b2c3d4e5f6...",
    chunk_sequence=0,
    total_chunks=15,
    processing_timestamp=datetime(2025, 12, 16, 10, 0, 0),
    token_count=128,

    # New for retrieval
    similarity_score=0.87,
    rank=1
)
```

---

### RetrievalResponse

**Description**: The complete response to a semantic search query, containing all matching results and metadata

**Fields**:
- `query_text` (string): The original query that was submitted
- `results` (List[RetrievalResult]): List of retrieved results, ordered by similarity
- `total_results` (integer): Number of results returned
- `execution_time_ms` (float): Total time taken for query processing in milliseconds
- `timestamp` (datetime): When the query was executed
- `parameters` (dict): Query parameters used (top_k, similarity_threshold)

**Validation Rules**:
- `results` list should be ordered by `similarity_score` descending
- `total_results` must equal `len(results)`
- `execution_time_ms` must be non-negative
- All results in `results` list should have `similarity_score >= similarity_threshold`

**Example**:
```python
RetrievalResponse(
    query_text="What is ROS2?",
    results=[
        RetrievalResult(..., similarity_score=0.87, rank=1),
        RetrievalResult(..., similarity_score=0.82, rank=2),
        RetrievalResult(..., similarity_score=0.76, rank=3),
        RetrievalResult(..., similarity_score=0.71, rank=4),
        RetrievalResult(..., similarity_score=0.65, rank=5)
    ],
    total_results=5,
    execution_time_ms=1250.5,
    timestamp=datetime(2025, 12, 16, 10, 30, 0),
    parameters={
        "top_k": 5,
        "similarity_threshold": 0.0,
        "collection_name": "physicalai",
        "embedding_model": "embed-multilingual-v3.0"
    }
)
```

---

## Relationships

### Query → RetrievalResponse (1:1)
- Each query execution produces exactly one retrieval response
- The response contains all information about that specific query execution

### RetrievalResponse → RetrievalResult (1:many)
- Each response contains zero or more retrieval results
- Results are ordered by relevance (similarity score)

### RetrievalResult → DocumentChunk (1:1 reference)
- Each retrieval result references a document chunk from the Qdrant database
- The relationship is read-only (retrieval doesn't modify chunks)

---

## Data Flow

```
User Query (string)
    ↓
Query Entity
    ↓
Query Embedding (1024-dim vector via Cohere)
    ↓
Qdrant Search (semantic similarity)
    ↓
List[ScoredPoint] (from Qdrant)
    ↓
List[RetrievalResult] (formatted with metadata)
    ↓
RetrievalResponse (complete response with metrics)
    ↓
User Output (displayed or returned to caller)
```

---

## Storage Considerations

### Not Persisted (Transient)
- **Query**: Only exists during query processing, not stored
- **RetrievalResponse**: Returned to caller, not persisted (unless caller chooses to log)
- **RetrievalResult**: Created from Qdrant data, not persisted separately

### Persisted (From Spec 1)
- **DocumentChunk**: Stored in Qdrant 'physicalai' collection as payload
- **Embedding Vectors**: Stored in Qdrant as 1024-dimensional vectors

**Rationale**: This is a validation/testing phase. Query logging and analytics can be added in future specs if needed.

---

## Python Type Definitions

```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Dict, Any

@dataclass
class Query:
    """Natural-language search query"""
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
```

---

## Similarity Score Semantics

### Score Interpretation

| Score Range | Interpretation | Usage Guidance |
|-------------|----------------|----------------|
| 0.9 - 1.0   | Excellent match | Highly relevant, near-duplicate content |
| 0.7 - 0.9   | Good match      | Relevant content, topically aligned |
| 0.5 - 0.7   | Moderate match  | Somewhat relevant, broader topic |
| 0.3 - 0.5   | Weak match      | Tangentially related |
| 0.0 - 0.3   | Poor match      | Not relevant |

### Recommended Thresholds

- **Strict retrieval**: `similarity_threshold >= 0.7`
- **Balanced retrieval**: `similarity_threshold >= 0.5`
- **Exploratory retrieval**: `similarity_threshold >= 0.0` (no filtering)

**Note**: Optimal thresholds should be determined empirically during validation based on query types and content characteristics.

---

## Future Extensions (Out of Scope for Spec 2)

- **Query logging**: Store Query and RetrievalResponse for analytics
- **Query expansion**: Reformulate queries for better recall
- **Result reranking**: Post-process results with additional scoring
- **Caching**: Store query embeddings for repeated queries
- **Hybrid search**: Combine semantic and keyword-based retrieval

These will be considered in future specs based on requirements.
