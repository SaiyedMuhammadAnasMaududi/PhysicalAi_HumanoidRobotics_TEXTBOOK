# Validation Checklist: Agent-Based RAG Backend with OpenAI Agent SDK

**Feature**: 001-agent-rag-backend
**Date**: 2025-12-17
**Validator**: _________________
**Environment**: _________________

## Overview

This checklist validates the implementation of the agent-based RAG backend using OpenAI Agent SDK with LiteLLM models. All validation is performed manually via API calls as specified in the feature requirements.

---

## Prerequisites

- [ ] Spec 2 (008-retrieval-query-validation) is complete and Qdrant collection 'physicalai' contains 550 vectors
- [ ] backend/.env file exists with GEMINI_API_KEY, LITELLM_MODEL, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
- [ ] FastAPI server is running on http://localhost:8000

---

## User Story 1: Basic Agent Chat Interface (P1)

### Test Case US1-1: Basic Query Execution

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'
```

**Expected Results**:
- [ ] Returns response with meaningful content (not empty)
- [ ] Response contains information related to ROS2
- [ ] Response completes within reasonable time (<30 seconds)
- [ ] Response includes metadata fields: response, sources (empty array OK), execution_time_ms, timestamp, query_id

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US1-2: Query with Context

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "Explain computer vision in robotics", "user_context": {"technical_level": "beginner"}}'
```

**Expected Results**:
- [ ] Returns response with meaningful content
- [ ] Response contains information related to computer vision
- [ ] Response acknowledges or uses user context if relevant
- [ ] All metadata fields present and properly formatted

**Status**: ☐ PASS ☐ FAIL

---

## User Story 2: Agent Integration with Retrieval Tool (P1)

### Test Case US2-1: Retrieval Tool Invocation

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'
```

**Expected Results**:
- [ ] Agent internally calls retrieval function_tool
- [ ] Response contains information from retrieved book chunks
- [ ] Sources array contains at least one source reference
- [ ] Source references include source_file, section_title, chunk_id, similarity_score

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US2-2: Grounded Responses

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "How do I set up a ROS2 workspace?"}'
```

**Expected Results**:
- [ ] Response contains specific information from book content (not generic)
- [ ] Response cites specific source files (should mention module-01-ros2 files)
- [ ] Information matches what's in the book content
- [ ] Similarity scores are between 0.0 and 1.0

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US2-3: Cross-Module Queries

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "How does perception connect to control?"}'
```

**Expected Results**:
- [ ] Response contains information from both perception and control modules
- [ ] Sources reference files from different modules (e.g., module-03-perception and module-04-control)
- [ ] Information is coherent and connects the concepts properly
- [ ] Response is grounded in actual book content

**Status**: ☐ PASS ☐ FAIL

---

## User Story 3: FastAPI Chat Endpoint (P2)

### Test Case US3-1: Request Validation

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": ""}'
```

**Expected Results**:
- [ ] Returns 400 status code
- [ ] Returns error message about empty query
- [ ] Response follows error format with timestamp

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-2: Long Query Validation

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "'$(printf "%.0sA" {1..1001})'"}'
```

**Expected Results**:
- [ ] Returns 400 status code
- [ ] Returns error message about query length
- [ ] Response follows error format with timestamp

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-3: Valid Response Format

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is AI?"}'
```

**Expected Results**:
- [ ] Returns 200 status code
- [ ] Response follows ChatResponse schema
- [ ] All required fields present: response, sources, execution_time_ms, timestamp, query_id
- [ ] Execution time is a positive number
- [ ] Timestamp is in ISO format
- [ ] Query ID is a string starting with "req-"

**Status**: ☐ PASS ☐ FAIL

---

### Test Case US3-4: Concurrent Requests

**Command**:
```bash
for i in {1..5}; do
  curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}' &
done
wait
```

**Expected Results**:
- [ ] All requests return successfully (no 500 errors)
- [ ] All responses are properly formatted
- [ ] No crashes or hangs in the server
- [ ] Response times remain reasonable (under 5 seconds each)

**Status**: ☐ PASS ☐ FAIL

---

## User Story 4: Session Management (P2)

*Note: This requires SQLiteSession integration which may not be fully implemented in basic version*

### Test Case US4-1: Multiple Sequential Requests

**Command**:
```bash
# First request
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'
# Second request with follow-up
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "Tell me more about its architecture?"}'
```

**Expected Results**:
- [ ] Both requests return successfully
- [ ] Second request can reference context from first (if implemented)
- [ ] Responses remain grounded in book content
- [ ] No session data leakage between requests

**Status**: ☐ PASS ☐ FAIL

---

## Performance Validation

### Test Case P-1: Response Time Performance

**Command**:
```bash
# Run multiple queries and measure response times
for i in {1..10}; do
  time curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}' -s -o /dev/null
done
```

**Expected Results**:
- [ ] 95% of queries complete in under 3 seconds
- [ ] Average response time is reasonable (<2 seconds)
- [ ] No significant degradation over multiple requests

**Status**: ☐ PASS ☐ FAIL

---

## Error Handling Validation

### Test Case E-1: Invalid JSON

**Command**:
```bash
curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{invalid json}'
```

**Expected Results**:
- [ ] Returns appropriate error status (400 or 500)
- [ ] Returns meaningful error message
- [ ] Server does not crash

**Status**: ☐ PASS ☐ FAIL

---

### Test Case E-2: Server Error Conditions

*Requires simulating backend failures - may need to temporarily disable services*

**Expected Results**:
- [ ] When retrieval pipeline fails, returns 500 with clear error message
- [ ] When agent service fails, returns 500 with clear error message
- [ ] Server remains stable after error conditions

**Status**: ☐ PASS ☐ FAIL

---

## Success Criteria Validation

### SC-001: Response Time <3s for 95% Requests

**Test**: Run 20 queries and measure execution_time_ms in responses
**Expected**: At least 19/20 (95%) have execution_time_ms < 3000
- [ ] Count of queries under 3s: ___/20
- [ ] Passes if ≥19 queries under 3s threshold

**Status**: ☐ PASS ☐ FAIL

### SC-002: Responses Grounded in Book Content

**Test**: Submit 10 different queries and manually verify responses
**Expected**: 9/10 (90%) responses contain information from book content with proper citations
- [ ] Queries tested:
  1.
  2.
  3.
  4.
  5.
  6.
  7.
  8.
  9.
  10.
- [ ] Count of grounded responses: ___/10
- [ ] Passes if ≥9 responses are properly grounded

**Status**: ☐ PASS ☐ FAIL

### SC-003: Handle 10 Concurrent Requests

**Test**: Send 10 concurrent requests using curl
**Expected**: All requests return successfully without degradation
- [ ] All 10 requests succeeded: ☐ YES ☐ NO
- [ ] Response times remained reasonable: ☐ YES ☐ NO

**Status**: ☐ PASS ☐ FAIL

### SC-004: 95% of Queries Address Question with Book Content

**Test**: Submit 20 different queries and verify they address the question using book content
**Expected**: 19/20 (95%) responses directly address the question using book content
- [ ] Count of queries addressed with book content: ___/20
- [ ] Passes if ≥19 queries properly addressed

**Status**: ☐ PASS ☐ FAIL

### SC-006: Agent Uses Retrieval Tool

**Test**: Submit queries and verify sources are returned
**Expected**: 19/20 (95%) of relevant queries trigger retrieval and return sources
- [ ] Count of queries with sources returned: ___/20
- [ ] Passes if ≥19 queries return sources

**Status**: ☐ PASS ☐ FAIL

---

## Final Validation Summary

### Test Results Summary

| Category | Total Tests | Passed | Failed | Status |
|----------|-------------|--------|--------|--------|
| User Story 1 | 2 | | | |
| User Story 2 | 3 | | | |
| User Story 3 | 4 | | | |
| User Story 4 | 1 | | | |
| Performance | 1 | | | |
| Error Handling | 2 | | | |
| Success Criteria | 5 | | | |
| **TOTAL** | **18** | | | |

### Overall Status
- [ ] All user stories validated successfully
- [ ] All success criteria met
- [ ] No critical failures observed
- [ ] Ready for production deployment: ☐ YES ☐ NO

### Issues Identified
```
[Document any issues found during validation]
```

### Recommendations
```
[Document any recommendations for improvements or fixes]
```

---

## Sign-off

**Implemented By**: _________________ **Date**: _______

**Reviewed By**: _________________ **Date**: _______

**Approved For Production**: ☐ YES ☐ NO **Date**: _______