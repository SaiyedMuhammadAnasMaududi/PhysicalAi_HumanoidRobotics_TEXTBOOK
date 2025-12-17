# Research: Retrieval & Query Validation

**Feature**: 008-retrieval-query-validation
**Date**: 2025-12-16
**Purpose**: Technical research for implementing semantic retrieval and query validation

## Research Questions & Answers

### 1. Cohere Query Embedding Parameters

**Question**: What is the optimal input_type for query embeddings vs document embeddings?

**Answer**: Cohere's embed API supports different `input_type` values optimized for different use cases:
- `input_type="search_document"`: For embedding documents/chunks to be stored (used in Spec 1)
- `input_type="search_query"`: For embedding user queries that will search against documents
- Using the correct input_type improves retrieval quality by optimizing embeddings for their purpose

**Implementation**:
```python
# For queries (Spec 2 - this feature)
response = cohere_client.embed(
    texts=[query_text],
    model="embed-multilingual-v3.0",
    input_type="search_query"  # ← Query-optimized
)

# For documents (Spec 1 - embedding pipeline)
response = cohere_client.embed(
    texts=chunk_texts,
    model="embed-multilingual-v3.0",
    input_type="search_document"  # ← Document-optimized
)
```

**Reference**: Cohere Embed API documentation - https://docs.cohere.com/reference/embed

---

### 2. Qdrant Search API

**Question**: What parameters does qdrant-client.search() accept and how to apply top_k and similarity score filtering?

**Answer**: The Qdrant Python client's `search()` method supports:
- `collection_name`: Name of the collection to search ("physicalai")
- `query_vector`: The embedding vector to search with
- `limit`: Maximum number of results (equivalent to top_k)
- `score_threshold`: Minimum score to include in results (filters low-similarity matches)
- `with_payload`: Whether to include payload metadata (default: True)
- `with_vectors`: Whether to include embedding vectors (default: False for efficiency)

**Implementation**:
```python
from qdrant_client import QdrantClient

client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

results = client.search(
    collection_name="physicalai",
    query_vector=query_embedding,  # 1024-dim vector from Cohere
    limit=top_k,  # e.g., 5
    score_threshold=similarity_threshold,  # e.g., 0.0 or 0.7
    with_payload=True,  # Include metadata
    with_vectors=False  # Don't return embedding vectors (saves bandwidth)
)

# results is List[ScoredPoint]
# Each ScoredPoint has:
# - id: chunk_id (UUID string)
# - score: similarity score (higher = more similar)
# - payload: dict with metadata (source_file, content, etc.)
```

**Reference**: Qdrant Python client documentation - https://qdrant.tech/documentation/

---

### 3. Similarity Score Normalization

**Question**: Does cosine similarity return scores between 0 and 1, or -1 and 1? How does Qdrant represent it?

**Answer**:
- **Cosine Similarity** (mathematical): Returns values from -1 (opposite) to 1 (identical)
- **Qdrant Cosine Distance**: Returns values from 0 to 2, where:
  - `distance = 0`: Identical vectors (perfect match)
  - `distance = 1`: Orthogonal vectors (no similarity)
  - `distance = 2`: Opposite vectors

- **Qdrant's `.score` field**: When using cosine distance, Qdrant returns:
  - `score = 1 - (cosine_distance / 2)`, which normalizes to [0, 1]
  - `score = 1.0`: Perfect match
  - `score = 0.5`: Orthogonal
  - `score = 0.0`: Completely opposite

**Important**: Qdrant's `search()` method already returns normalized scores in the `.score` field, so no manual conversion is needed!

**Implementation**:
```python
# Qdrant already normalizes scores for you
results = client.search(...)
for result in results:
    similarity_score = result.score  # Already in [0, 1] range, higher = better
    # No conversion needed!
```

**Reference**: Qdrant distance metrics - https://qdrant.tech/documentation/concepts/search/#metrics

---

### 4. Query Validation Best Practices

**Question**: What are reasonable query length limits? How to handle edge cases (empty, special characters, multilingual)?

**Answer**:

**Query Length**:
- Cohere embed-multilingual-v3.0 supports up to 512 tokens
- For most natural language queries, this is >2000 characters
- **Recommendation**: No hard character limit needed. Cohere will automatically truncate if needed.

**Empty Queries**:
- Should be validated and rejected before API call
- Return clear error: `ValueError("Query cannot be empty")`

**Special Characters**:
- Cohere handles special characters, emojis, and punctuation naturally
- No sanitization needed
- Examples that work: "ROS2?", "What is 🤖?", "C++ vs Python"

**Multilingual Queries**:
- embed-multilingual-v3.0 supports 100+ languages
- Works for: English, Spanish, Chinese, Arabic, etc.
- Semantically matches across languages (query in English can match document in Spanish if semantically similar)

**Edge Case Handling**:
```python
def validate_query(query_text: str) -> None:
    """Validate query before processing"""
    if not query_text or not query_text.strip():
        raise ValueError("Query cannot be empty")

    # That's it! Cohere handles everything else:
    # - Length truncation (up to 512 tokens)
    # - Special characters
    # - Multiple languages
    # - Punctuation and formatting
```

**Reference**: Cohere model documentation - https://docs.cohere.com/docs/models

---

## Key Findings Summary

### Technical Decisions Validated

1. **Use `input_type="search_query"` for queries** ✅
   - Different from document embeddings
   - Optimizes retrieval quality

2. **Qdrant search() provides clean API** ✅
   - Simple parameter: `limit` for top_k
   - Built-in filtering: `score_threshold`
   - Scores already normalized to [0, 1]

3. **Minimal query validation needed** ✅
   - Only validate: non-empty
   - Cohere handles: length, special chars, multilingual

4. **No manual score conversion required** ✅
   - Qdrant's `.score` field is already normalized
   - Higher score = better match

### Implementation Implications

- **Simple API**: Retrieval pipeline code will be clean and straightforward
- **Robust**: No edge case handling needed beyond empty string check
- **Performant**: No preprocessing or post-processing of scores
- **Maintainable**: Leverages Cohere and Qdrant built-in capabilities

### Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Cohere API rate limits | Query failures | Already handled in Spec 1, reuse error handling |
| Qdrant connection failures | No results | Add try-catch with clear error messages |
| Query-document embedding mismatch | Poor retrieval quality | Use correct input_type for each |
| Empty result sets | User confusion | Return clear message: "No results found" |

## Next Steps

1. ✅ Research complete - all questions answered
2. → Proceed to Phase 1: Design (architecture decisions)
3. → Document data model
4. → Create API contracts
5. → Write quickstart guide
