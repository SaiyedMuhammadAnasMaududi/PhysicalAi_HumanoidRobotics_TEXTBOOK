# Validation Checklist: Retrieval & Query Validation

**Feature**: 008-retrieval-query-validation
**Date**: 2025-12-17
**Validator**: _________________

## Overview

This checklist validates that the retrieval and query validation system meets all requirements from spec.md and plan.md. All tests are manual via CLI script as specified.

---

## Prerequisites

- [ ] Spec 1 (007-book-embedding-pipeline) complete with 550 vectors in Qdrant collection 'physicalai'
- [ ] Environment file backend/.env exists with all required keys
- [ ] backend/src/retrieval.py exists and is executable
- [ ] backend/scripts/test_retrieval.py exists and is executable

---

## User Story 1: Basic Semantic Query Retrieval (P1)

### Test Case US1-1: Basic Query Execution

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "What is ROS2?"
```

**Expected Results**:
- [ ] Returns exactly 5 results (default top_k)
- [ ] Results are ordered by similarity score (highest first)
- [ ] Rank 1 has the highest similarity score
- [ ] All results have similarity_score between 0.0 and 1.0
- [ ] Top result contains ROS2-related content
- [ ] Source files are from module-01-ros2 directory
- [ ] Execution time is displayed in milliseconds
- [ ] All metadata fields present: rank, score, file, section, chunk position

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US1-2: Query from Different Modules

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "computer vision"
```

**Expected Results**:
- [ ] Returns 5 results
- [ ] Top results from module-03-perception directory
- [ ] Content relates to computer vision/perception
- [ ] Results ordered by similarity

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US1-3: Control Systems Query

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "PID controller"
```

**Expected Results**:
- [ ] Returns 5 results
- [ ] Top results from module-04-control directory
- [ ] Content relates to control systems
- [ ] Execution time < 2000ms

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

## User Story 2: Configurable Retrieval Parameters (P2)

### Test Case US2-1: Custom top_k Parameter

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "robotics" --top_k 3
```

**Expected Results**:
- [ ] Returns exactly 3 results (not 5)
- [ ] Results ordered by similarity
- [ ] All other behavior consistent with US1

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US2-2: Similarity Threshold Filtering

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "sensors" --threshold 0.7
```

**Expected Results**:
- [ ] All returned results have similarity_score >= 0.7
- [ ] May return fewer than 5 results if not enough match threshold
- [ ] Results still ordered by similarity

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US2-3: Combined Parameters

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "simulation" --top_k 10 --threshold 0.6
```

**Expected Results**:
- [ ] Returns up to 10 results
- [ ] All results have similarity_score >= 0.6
- [ ] Results ordered by similarity descending

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

## User Story 3: Query Validation and Error Handling (P2)

### Test Case US3-1: Empty Query String

**Command**:
```bash
python3 backend/scripts/test_retrieval.py ""
```

**Expected Results**:
- [ ] Exits with error code 1
- [ ] Displays clear error message: "Query cannot be empty"
- [ ] Does not crash or throw stack trace

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-2: Very Long Query

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "$(printf 'test%.0s' {1..300})"
```

**Expected Results**:
- [ ] Processes without error
- [ ] Returns results
- [ ] Completes within reasonable time (<3s)

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-3: Special Characters in Query

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "What is ROS2? #robotics @AI"
```

**Expected Results**:
- [ ] Processes without error
- [ ] Returns relevant results
- [ ] Special characters handled gracefully

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-4: Empty Result Set

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "quantum physics" --threshold 0.95
```

**Expected Results**:
- [ ] Returns 0 results
- [ ] Does not crash
- [ ] Displays helpful message: "No results found. Try adjusting your query or lowering the similarity threshold."
- [ ] Execution metadata still shown

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-5: Invalid Parameter Values

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "test" --top_k 0
```

**Expected Results**:
- [ ] Exits with error code 1
- [ ] Clear error message about invalid top_k

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "test" --threshold 1.5
```

**Expected Results**:
- [ ] Exits with error code 1
- [ ] Clear error message about threshold range [0.0, 1.0]

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

## User Story 4: Retrieval Result Inspection (P3)

### Test Case US4-1: Verbose Output Mode

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "ROS2 nodes" --top_k 2 --verbose
```

**Expected Results**:
- [ ] Displays all DocumentChunk metadata fields:
  - [ ] chunk_id (UUID)
  - [ ] source_file (full path)
  - [ ] section_title
  - [ ] content (full, not truncated)
  - [ ] content_hash (SHA256)
  - [ ] chunk_sequence
  - [ ] total_chunks
  - [ ] processing_timestamp (ISO format)
  - [ ] token_count
- [ ] Displays retrieval-specific fields:
  - [ ] similarity_score (6 decimal places)
  - [ ] rank
- [ ] Shows query parameters used
- [ ] Shows execution time

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US4-2: JSON Output Format

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "perception" --top_k 3 --format json > results.json
cat results.json
```

**Expected Results**:
- [ ] Output is valid JSON (can be parsed)
- [ ] Contains all fields from verbose mode
- [ ] Structure matches specification:
  - [ ] query_text
  - [ ] timestamp (ISO format)
  - [ ] execution_time_ms
  - [ ] total_results
  - [ ] parameters object
  - [ ] results array with all result fields
- [ ] Can be imported into other tools

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US4-3: Table Output Format (Default)

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "humanoid"
```

**Expected Results**:
- [ ] Clean, readable table format
- [ ] Shows rank, score, file, section, chunk position
- [ ] Content preview (truncated to ~200 chars)
- [ ] Clear separators between results
- [ ] Execution summary at top

**Actual Results**:
```
[Record results here]
```

**Status**: ☐ PASS ☐ FAIL

---

## Success Criteria Validation

### SC-001: Semantic Relevance

**Test Queries**:
```bash
python3 backend/scripts/test_retrieval.py "What is ROS2?"
python3 backend/scripts/test_retrieval.py "computer vision"
python3 backend/scripts/test_retrieval.py "PID control"
python3 backend/scripts/test_retrieval.py "Gazebo simulation"
```

**Validation**:
- [ ] Top 3 results for each query are semantically relevant to the topic
- [ ] Results from appropriate modules (ROS2→module-01, vision→module-03, etc.)
- [ ] Manual inspection confirms 80%+ of results contain relevant content

**Notes**:
```
[Record observations]
```

**Status**: ☐ PASS ☐ FAIL

---

### SC-002: Query Performance (<2s for 95% of queries)

**Test**: Run 20 diverse queries and measure execution times

**Commands**:
```bash
for query in "ROS2" "sensors" "control" "simulation" "perception" "navigation" "manipulation" "planning" "URDF" "launch files" "topics" "services" "actions" "transforms" "cameras" "IMU" "PID" "MPC" "Gazebo" "collision"; do
  python3 backend/scripts/test_retrieval.py "$query" | grep "RESULTS"
done
```

**Validation**:
- [ ] Record execution times for all 20 queries
- [ ] At least 19/20 (95%) complete in < 2000ms
- [ ] Average execution time < 1500ms

**Execution Times**:
```
[Record times here]
```

**Status**: ☐ PASS ☐ FAIL

---

### SC-003: Coverage Across All 30 Source Files

**Test**: Query topics from each module and verify results span all source files

**Commands**:
```bash
# Module 1 - ROS2 (10 files)
python3 backend/scripts/test_retrieval.py "ROS2 introduction" --verbose
python3 backend/scripts/test_retrieval.py "publishers subscribers"
python3 backend/scripts/test_retrieval.py "services actions"
python3 backend/scripts/test_retrieval.py "parameters"
python3 backend/scripts/test_retrieval.py "launch files"

# Module 2 - Simulation (5 files)
python3 backend/scripts/test_retrieval.py "Gazebo"
python3 backend/scripts/test_retrieval.py "sensor simulation"
python3 backend/scripts/test_retrieval.py "URDF"

# Module 3 - Perception (5 files)
python3 backend/scripts/test_retrieval.py "computer vision"
python3 backend/scripts/test_retrieval.py "sensor fusion"
python3 backend/scripts/test_retrieval.py "perception integration"

# Module 4 - Control (7 files)
python3 backend/scripts/test_retrieval.py "PID control"
python3 backend/scripts/test_retrieval.py "model predictive control"
python3 backend/scripts/test_retrieval.py "safety constraints"

# Module 5 - Integration (3 files)
python3 backend/scripts/test_retrieval.py "system integration"
python3 backend/scripts/test_retrieval.py "deployment"
```

**Validation**:
- [ ] Collect unique source_file values from all query results
- [ ] Verify all 30 files appear at least once across queries
- [ ] Each module's files are reachable

**Files Found**:
```
[List unique files from results]
```

**Status**: ☐ PASS ☐ FAIL

---

### SC-004: Consistent Results

**Test**: Run same query 5 times and verify deterministic behavior

**Command**:
```bash
for i in {1..5}; do
  python3 backend/scripts/test_retrieval.py "ROS2 nodes" --format json | jq '.results[0].chunk_id'
done
```

**Validation**:
- [ ] All 5 runs return identical chunk_id for rank 1 result
- [ ] All 5 runs return identical similarity scores
- [ ] Result order is consistent

**Actual Results**:
```
[Record chunk IDs and scores]
```

**Status**: ☐ PASS ☐ FAIL

---

### SC-005: Graceful Error Handling

**Test**: Verify all error conditions handled without crashes

**Test Cases**:
- [ ] Empty query (US3-1) - exits cleanly with error message
- [ ] Invalid top_k (US3-5) - clear error, no crash
- [ ] Invalid threshold (US3-5) - clear error, no crash
- [ ] Empty results (US3-4) - displays helpful message
- [ ] Long query (US3-2) - processes successfully

**Status**: ☐ PASS ☐ FAIL

---

### SC-006: Semantic Relevance Threshold (80%)

**Test**: Manual inspection of results for 10 diverse queries

**Commands**:
```bash
python3 backend/scripts/test_retrieval.py "What is ROS2?" --top_k 5
python3 backend/scripts/test_retrieval.py "depth cameras" --top_k 5
python3 backend/scripts/test_retrieval.py "inverse kinematics" --top_k 5
python3 backend/scripts/test_retrieval.py "Gazebo plugins" --top_k 5
python3 backend/scripts/test_retrieval.py "sensor fusion algorithms" --top_k 5
python3 backend/scripts/test_retrieval.py "PID tuning" --top_k 5
python3 backend/scripts/test_retrieval.py "URDF models" --top_k 5
python3 backend/scripts/test_retrieval.py "ROS2 launch files" --top_k 5
python3 backend/scripts/test_retrieval.py "collision detection" --top_k 5
python3 backend/scripts/test_retrieval.py "humanoid balance control" --top_k 5
```

**Validation**:
- [ ] For each query, manually read top 5 results
- [ ] Count how many are semantically relevant (4/5 or 5/5 is good)
- [ ] Overall: At least 40/50 results (80%) are relevant

**Relevance Scores**:
```
Query 1: __/5 relevant
Query 2: __/5 relevant
Query 3: __/5 relevant
Query 4: __/5 relevant
Query 5: __/5 relevant
Query 6: __/5 relevant
Query 7: __/5 relevant
Query 8: __/5 relevant
Query 9: __/5 relevant
Query 10: __/5 relevant

Total: __/50 (must be >= 40)
```

**Status**: ☐ PASS ☐ FAIL

---

### SC-007: Correct Source Attribution

**Test**: Verify source_file paths are correct for sampled results

**Command**:
```bash
python3 backend/scripts/test_retrieval.py "ROS2" --top_k 5 --verbose
```

**Validation**:
- [ ] source_file paths match actual files in docs/ directory
- [ ] section_title matches actual section in source file
- [ ] chunk_sequence values are sensible (1-indexed, <= total_chunks)
- [ ] All metadata fields are populated (no null/empty values)

**Status**: ☐ PASS ☐ FAIL

---

### SC-008: Normalized Similarity Scores

**Test**: Verify all scores are in [0.0, 1.0] range

**Commands**:
```bash
python3 backend/scripts/test_retrieval.py "robotics" --format json | jq '.results[].similarity_score'
python3 backend/scripts/test_retrieval.py "test" --top_k 10 --format json | jq '.results[].similarity_score'
```

**Validation**:
- [ ] All scores >= 0.0
- [ ] All scores <= 1.0
- [ ] Scores are descending (first >= second >= third ...)

**Actual Scores**:
```
[Record scores from multiple queries]
```

**Status**: ☐ PASS ☐ FAIL

---

## Final Validation Summary

**Date Validated**: _________________
**Validated By**: _________________

### Test Results Summary

| Category | Total Tests | Passed | Failed |
|----------|-------------|--------|--------|
| User Story 1 | 3 | | |
| User Story 2 | 3 | | |
| User Story 3 | 6 | | |
| User Story 4 | 3 | | |
| Success Criteria | 8 | | |
| **TOTAL** | **23** | | |

**Overall Status**: ☐ PASS ☐ FAIL

### Issues Identified

```
[List any issues or edge cases discovered during validation]
```

### Recommendations

```
[Any recommendations for improvements or follow-up work]
```

---

## Sign-off

**Implementation Complete**: ☐ YES ☐ NO
**Ready for Integration (Spec 3)**: ☐ YES ☐ NO

**Signatures**:

Developer: _________________ Date: _______
Reviewer: _________________ Date: _______
Product Owner: _____________ Date: _______

---

## Notes

- All tests are manual via CLI script as specified in requirements
- Tests should be run on a clean environment with fresh .env configuration
- Performance tests should be run on similar hardware to production environment
- Semantic relevance is subjective but should align with module content
- Any FAIL status requires investigation and resolution before sign-off
