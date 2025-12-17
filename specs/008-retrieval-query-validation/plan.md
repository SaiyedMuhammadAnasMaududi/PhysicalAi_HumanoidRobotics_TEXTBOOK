# Implementation Plan: Retrieval & Query Validation for RAG Chatbot

**Branch**: `008-retrieval-query-validation` | **Date**: 2025-12-16 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/008-retrieval-query-validation/spec.md`

## Summary

Implementation of a retrieval and query validation system that accepts natural-language queries, generates embeddings using the same Cohere model from Spec 1, retrieves semantically similar chunks from the Qdrant 'physicalai' collection, and validates the end-to-end retrieval pipeline. The system will be implemented as a Python module with command-line utilities for testing and validation.

## Technical Context

**Language/Version**: Python 3.11 (same as Spec 1)
**Primary Dependencies**: cohere (>=4.0), qdrant-client (>=1.9), python-dotenv (>=1.0)
**Storage**: Existing Qdrant 'physicalai' collection (populated in Spec 1)
**Testing**: Manual validation via CLI, no pytest required per user request
**Target Platform**: Local development environment (Linux/WSL)
**Project Type**: Single module extending backend from Spec 1
**Performance Goals**: Query processing (embedding + retrieval) < 2 seconds for 95% of queries
**Constraints**: Must use same Cohere embed-multilingual-v3.0 model, cosine similarity metric, no LLM response generation
**Scale/Scope**: Validate retrieval across 550 embedded chunks from 30 book files

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Compliance with Constitution Principles:**
- ✅ Accuracy and Source Verification: Retrieved results include full metadata with source files and sections
- ✅ Traceability and Reproducibility: Consistent results across repeated queries with same parameters
- ✅ Reliability and Performance: Graceful error handling for connection failures and invalid inputs
- ✅ Extensibility and Technology Standards: Uses Cohere and Qdrant as specified, reusable for Spec 3 agent integration
- ✅ Security and Ethical Standards: No user data storage, validation-only pipeline

## Project Structure

### Documentation (this feature)

```text
specs/008-retrieval-query-validation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (backend directory)

```text
backend/
├── src/
│   ├── main.py                    # Embedding pipeline from Spec 1
│   ├── retrieval.py               # NEW: Query retrieval module
│   └── __init__.py
├── scripts/
│   ├── setup_embedding_env.sh     # From Spec 1
│   ├── run_embedding_pipeline.py  # From Spec 1
│   └── test_retrieval.py          # NEW: Retrieval validation script
├── tests/
│   └── unit/
│       ├── test_main.py           # From Spec 1
│       └── test_retrieval.py      # NEW: Retrieval tests (manual validation)
└── requirements.txt               # Same dependencies as Spec 1
```

**Structure Decision**: Extend existing backend structure from Spec 1. The retrieval.py module will contain the query processing logic, and test_retrieval.py will provide CLI utilities for validation. No new dependencies required - reuses Cohere and Qdrant clients from Spec 1.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## Phase 0: Research

### Research Questions

1. **Cohere Query Embedding Parameters**:
   - What is the optimal input_type for query embeddings vs document embeddings?
   - Answer: Use `input_type="search_query"` for queries (vs `input_type="search_document"` for documents)
   - Reference: Cohere Embed API documentation

2. **Qdrant Search API**:
   - What parameters does qdrant-client.search() accept?
   - How to apply top_k and similarity score filtering?
   - Answer: Use `client.search(collection_name, query_vector, limit=top_k, score_threshold=min_score)`
   - Reference: Qdrant Python client documentation

3. **Similarity Score Normalization**:
   - Does cosine similarity return scores between 0 and 1, or -1 and 1?
   - Answer: Cosine distance in Qdrant returns values 0-2, with lower being more similar. Convert to similarity: `similarity = 1 - (distance / 2)`
   - Reference: Qdrant distance metrics documentation

4. **Query Validation Best Practices**:
   - What are reasonable query length limits?
   - How to handle edge cases (empty, special characters, multilingual)?
   - Answer: No hard length limit needed - Cohere handles up to 512 tokens. Validate only for empty strings.

### Research Outputs

**File**: `specs/008-retrieval-query-validation/research.md`

- Cohere query embedding parameters and best practices
- Qdrant search API usage with filtering and scoring
- Similarity metric conversion and normalization
- Query validation strategies and edge case handling

## Phase 1: Design

### Architecture Decisions

**Decision 1: Retrieval Pipeline Flow**
- **Options Considered**:
  - A) Single function with inline logic
  - B) Separate classes for QueryEmbedder and ResultRetriever
  - C) Single RetrievalPipeline class orchestrating all steps
- **Decision**: Option C - Single RetrievalPipeline class
- **Rationale**: Provides clear structure, easy to test each phase independently, matches EmbeddingPipeline pattern from Spec 1
- **Document in**: ADR not required (simple architectural choice)

**Decision 2: Similarity Threshold Default**
- **Options Considered**:
  - A) threshold=0.0 (return all results)
  - B) threshold=0.5 (moderate filter)
  - C) threshold=0.7 (strict filter)
- **Decision**: Option A - threshold=0.0 (default), configurable via parameter
- **Rationale**: Allows developers to see all results and determine appropriate threshold empirically during validation
- **Document in**: quickstart.md configuration section

**Decision 3: Result Metadata Structure**
- **Options Considered**:
  - A) Return raw Qdrant ScoredPoint objects
  - B) Convert to custom RetrievalResult dataclass
  - C) Return plain dictionaries
- **Decision**: Option B - Custom RetrievalResult dataclass
- **Rationale**: Type-safe, matches DocumentChunk pattern from Spec 1, clear API for consumers
- **Document in**: data-model.md

**Decision 4: Query Embedding Caching**
- **Options Considered**:
  - A) Always regenerate embeddings for each query
  - B) Cache embeddings in-memory with LRU
  - C) Persist embeddings to disk
- **Decision**: Option A - Always regenerate (no caching)
- **Rationale**: Validation phase prioritizes correctness over performance. Caching can be added in Spec 3 if needed.
- **Document in**: Plan (this file) - out of scope for Spec 2

### Component Breakdown

**Component 1: QueryEmbedder**
- **Purpose**: Generate embeddings for natural-language queries
- **Inputs**: query_text (string), cohere_client, model_name
- **Outputs**: query_embedding (1024-dim vector)
- **Validation**: Empty query check, API error handling

**Component 2: VectorRetriever**
- **Purpose**: Search Qdrant collection for similar chunks
- **Inputs**: query_embedding, qdrant_client, collection_name, top_k, similarity_threshold
- **Outputs**: List[ScoredPoint] from Qdrant
- **Validation**: Connection error handling, result ordering verification

**Component 3: ResultFormatter**
- **Purpose**: Convert Qdrant results to RetrievalResult objects
- **Inputs**: List[ScoredPoint], query_text
- **Outputs**: RetrievalResponse with formatted results
- **Validation**: Metadata completeness check, similarity score normalization

**Component 4: RetrievalPipeline (Orchestrator)**
- **Purpose**: Coordinate end-to-end query processing
- **Inputs**: Configuration from .env, user query parameters
- **Outputs**: RetrievalResponse with results and execution metrics
- **Validation**: Full pipeline integration test

### Data Model

**File**: `specs/008-retrieval-query-validation/data-model.md`

**New Entities**:

1. **Query**
   - query_text: str
   - timestamp: datetime
   - top_k: int (default: 5)
   - similarity_threshold: float (default: 0.0)

2. **RetrievalResult**
   - All fields from DocumentChunk (Spec 1): chunk_id, source_file, section_title, content, content_hash, chunk_sequence, total_chunks, processing_timestamp, token_count
   - similarity_score: float (0.0 to 1.0, higher = more similar)
   - rank: int (1-indexed position in results)

3. **RetrievalResponse**
   - query_text: str
   - results: List[RetrievalResult]
   - total_results: int
   - execution_time_ms: float
   - timestamp: datetime
   - parameters: dict (top_k, similarity_threshold used)

### API Contracts

**File**: `specs/008-retrieval-query-validation/contracts/retrieval_api.md`

```python
class RetrievalPipeline:
    def __init__(
        self,
        cohere_api_key: str,
        qdrant_url: str,
        qdrant_api_key: str,
        collection_name: str = "physicalai",
        embedding_model: str = "embed-multilingual-v3.0"
    )

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
            APIError: If Cohere API fails
        """
```

### Quickstart Guide

**File**: `specs/008-retrieval-query-validation/quickstart.md`

**5-Minute Setup**:

1. **Prerequisites**: Spec 1 completed, .env configured, Qdrant collection populated
2. **Install dependencies**: Already installed from Spec 1
3. **Run validation script**: `python3 backend/scripts/test_retrieval.py "What is ROS2?"`
4. **Review results**: Check similarity scores, source files, content relevance
5. **Experiment with parameters**: Try different top_k and threshold values

**Common Queries for Testing**:
- "How do I set up ROS2?" → Should return module-01 content
- "Gazebo simulation" → Should return module-02 content
- "Computer vision" → Should return module-03 content
- "Behavior trees" → Should return module-04 content

## Phase 2: Task Breakdown

**Note**: Detailed tasks will be generated in `/sp.tasks` command. This section provides high-level phases.

### Phase 2.1: Core Retrieval Implementation
- Implement RetrievalResult and RetrievalResponse dataclasses
- Implement QueryEmbedder with Cohere client
- Implement VectorRetriever with Qdrant search
- Implement ResultFormatter with score normalization
- Implement RetrievalPipeline orchestrator

### Phase 2.2: Validation Script
- Create test_retrieval.py CLI script
- Add command-line argument parsing (query, top_k, threshold)
- Add formatted output display (table or JSON)
- Add execution time tracking

### Phase 2.3: Validation Checklist
- Test queries from each book module
- Verify results ordered by similarity
- Verify metadata completeness
- Test edge cases (empty query, no results, connection errors)
- Verify consistency across repeated queries
- Document optimal threshold values for different query types

## Testing Strategy

### Validation Approach

**No automated tests required** per user request. Use manual validation with test_retrieval.py script.

**Validation Checklist** (`specs/008-retrieval-query-validation/validation-checklist.md`):

1. **Basic Functionality**:
   - [ ] Query "What is ROS2?" returns relevant module-01 chunks
   - [ ] Results ordered by decreasing similarity score
   - [ ] All metadata fields present in results

2. **Parameter Configuration**:
   - [ ] top_k=3 returns exactly 3 results
   - [ ] top_k=10 returns up to 10 results
   - [ ] similarity_threshold=0.7 filters out low-score results

3. **Edge Cases**:
   - [ ] Empty query raises ValueError with clear message
   - [ ] Very long query (>1000 chars) processes successfully
   - [ ] Query with special characters returns valid results
   - [ ] Qdrant disconnected returns clear error (test by stopping Qdrant)

4. **Consistency**:
   - [ ] Same query repeated 3 times returns identical results in same order
   - [ ] Results match expected source files (ROS2 → module-01, Gazebo → module-02)

5. **Coverage**:
   - [ ] At least one successful query from each of 5 book modules
   - [ ] Verify chunks retrieved from all 30 source files (aggregate across queries)

### Sample Queries for Validation

```python
# Module-specific queries
"What is ROS2 and how does it work?"
"How do I install Gazebo simulator?"
"Explain computer vision in robotics"
"What are behavior trees for task planning?"
"How to integrate all modules for capstone project?"

# Cross-module queries
"How to connect perception to control?"
"What simulation tools are available?"

# Edge cases
""  # Empty
"🤖" * 500  # Special characters
"a" * 1000  # Very long
```

## Dependencies

- **Spec 1 (007-book-embedding-pipeline)**: REQUIRED - Qdrant collection must be populated
- **Cohere API**: REQUIRED - Same API key from Spec 1
- **Qdrant Database**: REQUIRED - Same instance and collection from Spec 1
- **.env Configuration**: REQUIRED - COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

## Implementation Notes

### Key Technical Decisions

1. **Similarity Score Conversion**: Qdrant cosine distance → similarity: `1 - (distance / 2)`
2. **Default Parameters**: top_k=5, similarity_threshold=0.0 (show all results)
3. **Error Handling**: Fail fast with clear messages, no silent failures
4. **No Caching**: Regenerate embeddings each query for validation accuracy
5. **Output Format**: Rich metadata for debugging, JSON export option for programmatic use

### Performance Expectations

- Query embedding generation: ~200-500ms
- Qdrant search: ~50-200ms
- Result formatting: <10ms
- **Total**: <2 seconds for 95% of queries (meets SC-002)

### Configuration via Environment

Reuse .env from Spec 1:
```bash
COHERE_API_KEY=<key>
QDRANT_URL=<url>
QDRANT_API_KEY=<key>
QDRANT_COLLECTION=physicalai
EMBEDDING_MODEL=embed-multilingual-v3.0
```

No new environment variables required.

## Next Steps

1. Run `/sp.tasks` to generate detailed task list
2. Implement retrieval.py module
3. Create test_retrieval.py validation script
4. Execute validation checklist
5. Document optimal threshold values in quickstart.md
6. Ready for Spec 3: Agent Integration
