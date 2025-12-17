# API Contract: Retrieval Pipeline

**Feature**: 008-retrieval-query-validation
**Version**: 1.0
**Date**: 2025-12-16

## Overview

This document defines the public API contract for the retrieval and query validation system. All interfaces must remain stable across implementations to ensure compatibility with future specs (particularly Spec 3: Agent Integration).

---

## RetrievalPipeline Class

### Constructor

```python
class RetrievalPipeline:
    """
    Orchestrates semantic retrieval of book content from Qdrant vector database.

    This class provides the main interface for querying the embedded book content.
    It handles query embedding generation via Cohere and similarity search via Qdrant.
    """

    def __init__(
        self,
        cohere_api_key: str,
        qdrant_url: str,
        qdrant_api_key: str,
        collection_name: str = "physicalai",
        embedding_model: str = "embed-multilingual-v3.0"
    ):
        """
        Initialize the retrieval pipeline.

        Args:
            cohere_api_key: API key for Cohere embeddings
            qdrant_url: URL of the Qdrant instance
            qdrant_api_key: API key for Qdrant
            collection_name: Name of the Qdrant collection (default: "physicalai")
            embedding_model: Cohere model for embeddings (default: "embed-multilingual-v3.0")

        Raises:
            ValueError: If any required parameter is empty
            ConnectionError: If cannot connect to Qdrant
        """
```

**Invariants**:
- Must use the same embedding model as Spec 1 for consistency
- Collection name must match the collection created in Spec 1
- Cohere client must be configured with `input_type="search_query"` for queries

---

### Query Method

```python
    def query(
        self,
        query_text: str,
        top_k: int = 5,
        similarity_threshold: float = 0.0
    ) -> RetrievalResponse:
        """
        Execute a semantic search query against the vector database.

        This is the primary method for retrieving relevant book content.
        It generates a query embedding, searches Qdrant for similar chunks,
        and returns formatted results with metadata.

        Args:
            query_text: Natural-language query string (required, non-empty)
            top_k: Maximum number of results to return (default: 5, range: 1-100)
            similarity_threshold: Minimum similarity score (default: 0.0, range: 0.0-1.0)
                                 Results below this threshold are filtered out

        Returns:
            RetrievalResponse containing:
                - Ordered list of matching chunks with similarity scores
                - Execution metadata (time, parameters, result count)
                - Full chunk content and source information

        Raises:
            ValueError: If query_text is empty or parameters are invalid
            ConnectionError: If Qdrant is unreachable
            APIError: If Cohere API call fails

        Behavior Guarantees:
            - Results are ordered by similarity_score descending (best match first)
            - Rank 1 always has the highest similarity_score
            - All results have similarity_score >= similarity_threshold
            - Returns empty results list if no matches found (not an error)
            - Same query with same parameters returns identical results (deterministic)
            - Execution time is tracked and included in response

        Examples:
            >>> pipeline = RetrievalPipeline(...)
            >>> response = pipeline.query("What is ROS2?", top_k=5)
            >>> print(f"Found {response.total_results} results")
            >>> for result in response.results:
            ...     print(f"  {result.rank}. {result.section_title} (score: {result.similarity_score:.2f})")
        """
```

**Preconditions**:
- `query_text` must not be empty or whitespace-only
- `top_k` must be positive integer
- `similarity_threshold` must be in range [0.0, 1.0]

**Postconditions**:
- Returns `RetrievalResponse` with results ordered by `similarity_score` descending
- `response.total_results == len(response.results)`
- All `result.similarity_score >= similarity_threshold`
- `response.execution_time_ms >= 0`

---

## Data Transfer Objects (DTOs)

### Query

```python
@dataclass
class Query:
    """
    Represents a search query submitted to the retrieval system.

    This is primarily used internally for tracking and logging.
    Most users will pass query parameters directly to the query() method.
    """

    query_text: str              # The natural-language query
    timestamp: datetime           # When the query was submitted
    top_k: int = 5               # Max results to return
    similarity_threshold: float = 0.0  # Min similarity score
```

**Contract**:
- Immutable after creation (dataclass frozen=False for simplicity, but treat as immutable)
- All fields must be provided at construction
- Validation happens at `query()` method level, not in dataclass

---

### RetrievalResult

```python
@dataclass
class RetrievalResult:
    """
    Represents a single document chunk retrieved from the vector database.

    Contains all metadata from the original DocumentChunk (Spec 1) plus
    similarity scoring information specific to the query.
    """

    # From DocumentChunk (Spec 1) - READ-ONLY
    chunk_id: str                      # UUID
    source_file: str                   # Path to original markdown file
    section_title: str                 # Heading/section name
    content: str                       # Full chunk text
    content_hash: str                  # SHA256 hash
    chunk_sequence: int                # Position in source doc
    total_chunks: int                  # Total chunks from source
    processing_timestamp: datetime      # When chunk was created
    token_count: int                   # Number of tokens in content

    # New for retrieval - QUERY-SPECIFIC
    similarity_score: float            # 0.0-1.0, higher = better match
    rank: int                          # 1-indexed position in results
```

**Contract**:
- All DocumentChunk fields are read-only (from Qdrant payload)
- `similarity_score` is normalized to [0.0, 1.0] range
- `rank` starts at 1 (not 0) for user-friendliness
- Results with same similarity_score may have arbitrary rank order

**Ordering Guarantee**:
```python
assert results[0].similarity_score >= results[1].similarity_score >= ...
assert results[0].rank == 1
assert results[1].rank == 2
```

---

### RetrievalResponse

```python
@dataclass
class RetrievalResponse:
    """
    Complete response to a semantic search query.

    Contains all matching results plus execution metadata.
    This is the primary return type for the query() method.
    """

    query_text: str                    # Original query
    results: List[RetrievalResult]     # Ordered list of matches
    total_results: int                 # len(results)
    execution_time_ms: float           # Query processing time
    timestamp: datetime                 # When query was executed
    parameters: Dict[str, Any]         # Query parameters used
```

**Contract**:
- `total_results == len(results)` (invariant)
- `results` is ordered by `similarity_score` descending
- `execution_time_ms` includes embedding generation + Qdrant search + formatting
- `parameters` dict includes: `top_k`, `similarity_threshold`, `collection_name`, `embedding_model`

**Usage Example**:
```python
response = pipeline.query("What is ROS2?", top_k=5, similarity_threshold=0.7)

# Access results
for result in response.results:
    print(f"{result.rank}. [{result.similarity_score:.2f}] {result.section_title}")
    print(f"   File: {result.source_file}")
    print(f"   Content: {result.content[:100]}...")

# Check performance
print(f"Query took {response.execution_time_ms:.0f}ms")
print(f"Found {response.total_results} results")
```

---

## Error Handling

### Exception Hierarchy

```python
# User errors (fixable by caller)
class ValidationError(ValueError):
    """Query validation failed (empty query, invalid parameters)"""
    pass

# System errors (infrastructure issues)
class QdrantConnectionError(ConnectionError):
    """Cannot connect to Qdrant instance"""
    pass

class CohereAPIError(Exception):
    """Cohere API request failed"""
    pass

# Usage
try:
    response = pipeline.query(query_text)
except ValidationError as e:
    print(f"Invalid query: {e}")
except QdrantConnectionError as e:
    print(f"Database unavailable: {e}")
except CohereAPIError as e:
    print(f"Embedding service error: {e}")
```

### Error Messages

**Must be clear and actionable**:

```python
# Good
raise ValidationError("Query cannot be empty. Please provide a non-empty query string.")

# Good
raise ValidationError(f"similarity_threshold must be between 0.0 and 1.0, got {threshold}")

# Bad (too vague)
raise ValueError("Invalid input")

# Bad (too technical)
raise Exception("Qdrant search() returned status 500")
```

---

## Performance Contracts

### Response Time Guarantees

**Target**: 95% of queries complete in < 2 seconds

**Breakdown**:
- Query embedding generation: 200-500ms (Cohere API)
- Qdrant search: 50-200ms
- Result formatting: <10ms
- **Total**: <2000ms for p95

**If exceeding targets**:
- Log warning with actual execution time
- Continue processing (don't fail)
- Include timing breakdown in logs for debugging

---

## Compatibility Guarantees

### Backward Compatibility

**These signatures MUST NOT change in future versions**:
- `RetrievalPipeline.__init__(...)` parameters
- `RetrievalPipeline.query(...)` parameters and return type
- `RetrievalResult` fields (can add new fields, but cannot remove/rename)
- `RetrievalResponse` fields (can add new fields, but cannot remove/rename)

**Safe to change**:
- Internal implementation details
- Error message text (keep same exception types)
- Performance characteristics (as long as within contract)
- Logging and debugging output

### Forward Compatibility (for Spec 3)

**Agent Integration Requirements**:
```python
# Spec 3 will use this interface:
retrieval = RetrievalPipeline(...)
response = retrieval.query(user_query, top_k=5)
context = "\n\n".join([r.content for r in response.results])
# Feed context to LLM agent
```

**Must support**:
- Synchronous query execution (no async required yet)
- Batch queries (multiple consecutive calls)
- Thread-safe operation (may be called from different threads)

---

## Testing Contracts

### Unit Test Requirements

```python
def test_query_empty_string():
    """Empty query raises ValidationError"""
    pipeline = RetrievalPipeline(...)
    with pytest.raises(ValidationError, match="cannot be empty"):
        pipeline.query("")

def test_query_results_ordered():
    """Results are ordered by similarity descending"""
    response = pipeline.query("test query", top_k=10)
    scores = [r.similarity_score for r in response.results]
    assert scores == sorted(scores, reverse=True)

def test_query_deterministic():
    """Same query returns same results"""
    response1 = pipeline.query("test query", top_k=5)
    response2 = pipeline.query("test query", top_k=5)
    assert response1.results[0].chunk_id == response2.results[0].chunk_id
    assert response1.results[0].similarity_score == response2.results[0].similarity_score
```

---

## Versioning

**Current Version**: 1.0

**Version Compatibility**:
- Major version bump (2.0): Breaking changes to public API
- Minor version bump (1.1): New features, backward compatible
- Patch version bump (1.0.1): Bug fixes only

**This contract is v1.0 and must remain stable through Spec 3 (Agent Integration).**

---

## Future Extensions (Out of Scope)

These may be added in future versions without breaking compatibility:

1. **Async Support**:
   ```python
   async def query_async(...) -> RetrievalResponse
   ```

2. **Batch Queries**:
   ```python
   def query_batch(queries: List[str], ...) -> List[RetrievalResponse]
   ```

3. **Streaming Results**:
   ```python
   def query_stream(...) -> Iterator[RetrievalResult]
   ```

4. **Query Caching**:
   ```python
   def query(..., use_cache: bool = True) -> RetrievalResponse
   ```

5. **Result Reranking**:
   ```python
   def query(..., rerank: bool = False) -> RetrievalResponse
   ```

These will be considered based on Spec 3+ requirements.
