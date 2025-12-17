# Tasks: Retrieval & Query Validation for RAG Chatbot

**Feature**: 008-retrieval-query-validation
**Branch**: `008-retrieval-query-validation`
**Created**: 2025-12-16

## Overview

This document breaks down the implementation of the retrieval and query validation system into concrete, executable tasks. Tasks are organized by user story to enable independent implementation and testing.

**Testing Approach**: Manual validation via CLI (no automated tests per spec requirement)

---

## Task Summary

| Phase | User Story | Task Count | Can Run Independently? |
|-------|-----------|------------|------------------------|
| Setup | N/A | 2 | Yes (after Spec 1) |
| Foundational | N/A | 3 | No (blocks all stories) |
| US1: Basic Retrieval | P1 | 7 | Yes (after Foundational) |
| US2: Configurable Parameters | P2 | 3 | Yes (after US1) |
| US3: Error Handling | P2 | 4 | Yes (after US1) |
| US4: Result Inspection | P3 | 3 | Yes (after US1) |
| Polish | N/A | 2 | No (after all stories) |
| **TOTAL** | | **24** | |

---

## Dependencies & Execution Order

```
Setup (T001-T002)
    ↓
Foundational (T003-T005)
    ↓
    ├─→ US1: Basic Retrieval (T006-T012) ← REQUIRED FIRST
    │       ↓
    │       ├─→ US2: Configurable Parameters (T013-T015) [P]
    │       ├─→ US3: Error Handling (T016-T019) [P]
    │       └─→ US4: Result Inspection (T020-T022) [P]
    │               ↓
    └───────────────→ Polish (T023-T024)
```

**Parallel Opportunities**: US2, US3, and US4 can be implemented in parallel after US1 is complete.

**MVP Scope**: Complete through US1 (T001-T012) for a working retrieval system.

---

## Phase 1: Setup

**Goal**: Verify prerequisites and prepare environment

- [X] T001 Verify Spec 1 (007-book-embedding-pipeline) is complete and Qdrant collection 'physicalai' contains 550 vectors
- [X] T002 Confirm backend/.env file exists with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION=physicalai

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Create shared data structures and base infrastructure needed by all user stories

**Independent Test**: Can instantiate dataclasses and import from retrieval module

- [X] T003 Create RetrievalResult dataclass in backend/src/retrieval.py with all DocumentChunk fields plus similarity_score and rank
- [X] T004 Create RetrievalResponse dataclass in backend/src/retrieval.py with query_text, results list, total_results, execution_time_ms, timestamp, and parameters dict
- [X] T005 Create Query dataclass in backend/src/retrieval.py with query_text, timestamp, top_k (default=5), similarity_threshold (default=0.0)

---

## Phase 3: User Story 1 - Basic Semantic Query Retrieval (P1)

**Goal**: Accept natural-language queries and return semantically relevant chunks ordered by similarity

**Why P1**: Core functionality - entire RAG system depends on this working

**Independent Test**: Submit query "What is ROS2?" and verify:
- Returns 5 results with similarity scores
- Results ordered by score (highest first)
- Top results contain ROS2-related content from module-01 files
- All metadata fields present

**Tasks**:

- [X] T006 [US1] Implement QueryEmbedder class in backend/src/retrieval.py with generate_embedding() method using Cohere client with input_type="search_query"
- [X] T007 [US1] Implement VectorRetriever class in backend/src/retrieval.py with search() method using Qdrant client.search() with cosine similarity
- [X] T008 [US1] Implement ResultFormatter class in backend/src/retrieval.py with format_results() method to convert Qdrant ScoredPoints to RetrievalResult objects
- [X] T009 [US1] Implement RetrievalPipeline class __init__() in backend/src/retrieval.py to initialize Cohere and Qdrant clients from environment variables
- [X] T010 [US1] Implement RetrievalPipeline.query() method in backend/src/retrieval.py to orchestrate: validate query → generate embedding → search Qdrant → format results → return RetrievalResponse
- [X] T011 [US1] Ensure results in RetrievalResponse are ordered by similarity_score descending with rank field set correctly (1-indexed)
- [X] T012 [US1] Create backend/scripts/test_retrieval.py CLI script with basic query execution and table output display

**Validation**: Run `python3 backend/scripts/test_retrieval.py "What is ROS2?"` and verify output shows 5 results with scores, ordered correctly

---

## Phase 4: User Story 2 - Configurable Retrieval Parameters (P2)

**Goal**: Support top_k and similarity_threshold parameters for flexible retrieval

**Why P2**: Allows optimization for different query types, but basic retrieval works without it

**Independent Test**: Run same query with different parameters:
- `--top_k 3` returns exactly 3 results
- `--threshold 0.7` filters to only high-scoring results
- `--top_k 10 --threshold 0.6` returns up to 10 results all >= 0.6

**Tasks**:

- [X] T013 [P] [US2] Add top_k parameter to RetrievalPipeline.query() method with default=5 and pass to VectorRetriever.search() as limit parameter
- [X] T014 [P] [US2] Add similarity_threshold parameter to RetrievalPipeline.query() method with default=0.0 and pass to Qdrant client.search() as score_threshold
- [X] T015 [US2] Add --top_k and --threshold command-line arguments to backend/scripts/test_retrieval.py using argparse

**Validation**: Test with `python3 backend/scripts/test_retrieval.py "robotics" --top_k 3 --threshold 0.7`

---

## Phase 5: User Story 3 - Query Validation and Error Handling (P2)

**Goal**: Handle edge cases gracefully without crashes

**Why P2**: Production readiness, but basic functionality works for valid inputs

**Independent Test**: Submit edge cases and verify:
- Empty query: Raises ValueError with clear message
- Long query (>1000 chars): Processes successfully
- Special characters: Returns valid results or empty set
- Qdrant disconnected: Returns ConnectionError with message

**Tasks**:

- [X] T016 [P] [US3] Add query validation in RetrievalPipeline.query() to raise ValueError if query_text is empty or whitespace-only with message "Query cannot be empty"
- [X] T017 [P] [US3] Add try-catch in QueryEmbedder.generate_embedding() to catch Cohere API errors and raise CohereAPIError with descriptive message
- [X] T018 [P] [US3] Add try-catch in VectorRetriever.search() to catch Qdrant connection errors and raise QdrantConnectionError with descriptive message
- [X] T019 [US3] Handle empty result sets gracefully in ResultFormatter.format_results() by returning RetrievalResponse with empty results list and appropriate message

**Validation**: Test error cases:
```bash
python3 backend/scripts/test_retrieval.py ""  # Should show error
python3 backend/scripts/test_retrieval.py "$(printf 'test%.0s' {1..300})"  # Should work
```

---

## Phase 6: User Story 4 - Retrieval Result Inspection (P3)

**Goal**: Display detailed metadata for debugging and validation

**Why P3**: Helpful for validation but not critical for basic operation

**Independent Test**: Submit query and verify all metadata fields present:
- chunk_id, source_file, section_title, content (full, not truncated)
- content_hash, chunk_sequence, total_chunks, processing_timestamp
- token_count, similarity_score, rank

**Tasks**:

- [X] T020 [P] [US4] Add --verbose flag to backend/scripts/test_retrieval.py to display full metadata for each result including all DocumentChunk fields
- [X] T021 [P] [US4] Add --format json option to backend/scripts/test_retrieval.py to output results as JSON for programmatic use
- [X] T022 [US4] Create formatted output display in backend/scripts/test_retrieval.py showing result details: rank, score, source file, section title, and content preview

**Validation**: Run with verbose flag:
```bash
python3 backend/scripts/test_retrieval.py "ROS2" --verbose
python3 backend/scripts/test_retrieval.py "ROS2" --format json > results.json
```

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final refinements and documentation

- [X] T023 Add execution time tracking to RetrievalPipeline.query() and include in RetrievalResponse.execution_time_ms field
- [X] T024 Create validation checklist document at specs/008-retrieval-query-validation/validation-checklist.md with test cases for all user stories per plan.md

---

## Implementation Strategy

### MVP (Minimum Viable Product)

**Scope**: Complete through User Story 1 (Tasks T001-T012)

**Delivers**:
- Working retrieval system
- Basic query execution via CLI
- Ordered results with similarity scores
- Full metadata in responses

**Validation**: Can submit queries and get relevant results

### Incremental Delivery

After MVP, add features independently:

1. **US2 (T013-T015)**: Add parameter configuration - 30 min
2. **US3 (T016-T019)**: Add error handling - 45 min
3. **US4 (T020-T022)**: Add detailed inspection - 30 min
4. **Polish (T023-T024)**: Final touches - 20 min

**Total estimated time**: ~3-4 hours for full implementation

### Parallel Execution Examples

**After Foundational Phase**:
```bash
# US1 must complete first
Developer A: Implements US1 (T006-T012)

# After US1 is done, these can run in parallel:
Developer A: Implements US2 (T013-T015)
Developer B: Implements US3 (T016-T019)
Developer C: Implements US4 (T020-T022)
```

---

## Validation Checklist Reference

After implementation, validate using `specs/008-retrieval-query-validation/validation-checklist.md`:

1. **US1**: Basic query "What is ROS2?" returns relevant module-01 chunks
2. **US2**: Parameters (top_k, threshold) work correctly
3. **US3**: Edge cases handled gracefully (empty query, errors)
4. **US4**: All metadata fields present and correct
5. **Cross-cutting**: Consistent results, performance <2s, all 30 files reachable

---

## File Reference

**New Files Created**:
- `backend/src/retrieval.py` - Core retrieval module (T003-T011, T013-T014, T016-T019)
- `backend/scripts/test_retrieval.py` - CLI validation script (T012, T015, T020-T022)
- `specs/008-retrieval-query-validation/validation-checklist.md` - Test cases (T024)

**Modified Files**:
- None (extends Spec 1 without modifications)

**Dependencies**:
- Reuses: `backend/.env`, `backend/requirements.txt` (from Spec 1)
- Requires: Qdrant collection 'physicalai' populated with 550 vectors

---

## Success Criteria Mapping

| Success Criteria | Validated By |
|------------------|--------------|
| SC-001: Relevant results for topics | US1 validation + manual review |
| SC-002: Query processing <2s | T023 (execution time tracking) |
| SC-003: Retrieves from all 30 files | Manual validation across queries |
| SC-004: Consistent results | Repeat query tests |
| SC-005: Graceful error handling | US3 validation |
| SC-006: 80% semantic relevance | Manual inspection via US4 |
| SC-007: Correct source files | US1 validation per module |
| SC-008: Normalized scores [0,1] | US1 validation |

---

## Notes

- **No automated tests**: Per spec requirement, validation is manual via CLI script
- **Reuses Spec 1 infrastructure**: Same .env, dependencies, Qdrant collection
- **Ready for Spec 3**: RetrievalPipeline class designed for agent integration
- **Independent stories**: US2, US3, US4 can be developed in any order after US1
