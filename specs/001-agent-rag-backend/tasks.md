# Tasks: Agent-Based RAG Backend using OpenAI Agent SDK with LiteLLM Models

**Feature**: 001-agent-rag-backend
**Branch**: `001-agent-rag-backend`
**Created**: 2025-12-17

## Overview

This document breaks down the implementation of the agent-based RAG backend using OpenAI Agent SDK with LiteLLM models into concrete, executable tasks. Tasks are organized by user story to enable independent implementation and testing.

**Testing Approach**: Manual validation via API calls (no automated tests per spec requirement)

---

## Task Summary

| Phase | User Story | Task Count | Can Run Independently? |
|-------|-----------|------------|------------------------|
| Setup | N/A | 3 | Yes |
| Foundational | N/A | 4 | No (blocks all stories) |
| US1: Basic Agent Chat Interface | P1 | 8 | Yes (after Foundational) |
| US2: Agent Integration with Retrieval Tool | P1 | 6 | Yes (after Foundational) |
| US3: FastAPI Chat Endpoint | P2 | 7 | Yes (after Foundational) |
| US4: Session Management | P2 | 4 | Yes (after Foundational) |
| Polish | N/A | 3 | No (after all stories) |
| **TOTAL** | | **35** | |

---

## Dependencies & Execution Order

```
Setup (T001-T003)
    ↓
Foundational (T004-T007)
    ↓
    ├─→ US1: Basic Chat Interface (T008-T015) ← REQUIRED FIRST
    │       ↓
    │       ├─→ US2: Agent Integration (T016-T021) [P]
    │       ├─→ US3: FastAPI Endpoint (T022-T028) [P]
    │       └─→ US4: Session Management (T029-T032) [P]
    │               ↓
    └───────────────→ Polish (T033-T035)
```

**Parallel Opportunities**: US2, US3, and US4 can be implemented in parallel after US1 is complete.

**MVP Scope**: Complete through US1 (T001-T015) for a working agent chat system.

---

## Phase 1: Setup

**Goal**: Verify prerequisites and prepare environment

- [X] T001 Verify Specs 1-2 (007-book-embedding-pipeline, 008-retrieval-query-validation) are complete and Qdrant collection 'physicalai' contains 550 vectors
- [X] T002 Confirm backend/.env file exists with GEMINI_API_KEY, LITELLM_MODEL, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
- [X] T003 Update backend/requirements.txt to include agents>=0.1.0, litellm>=1.0.0, fastapi>=0.100.0, uvicorn>=0.20.0

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Create shared data structures and base infrastructure needed by all user stories

**Independent Test**: Can instantiate dataclasses and import from agent module

- [X] T004 Create ChatRequest model in backend/api/models/chat.py with query (required), user_context (optional), and timestamp fields per data-model.md
- [X] T005 Create ChatResponse model in backend/api/models/chat.py with response, sources, execution_time_ms, timestamp, and query_id fields per data-model.md
- [X] T006 Create SourceReference model in backend/api/models/chat.py with source_file, section_title, chunk_id, similarity_score, content_preview fields per data-model.md
- [X] T007 Create AgentService orchestrator class in backend/src/agent_service.py with configuration loading and basic structure

---

## Phase 3: User Story 1 - Basic Agent Chat Interface (P1)

**Goal**: Accept natural-language queries and return agent responses with basic functionality using OpenAI Agent SDK

**Why P1**: Core functionality - entire agent system depends on this working

**Independent Test**: Submit query "What is ROS2?" and verify:
- Returns response with some content
- Response contains information related to ROS2
- Response completes within reasonable time (<30 seconds)

**Tasks**:

- [X] T008 [US1] Implement basic RAGAgent class in backend/src/agent.py with agents.Agent and LitellmModel initialization from environment variables
- [X] T009 [US1] Add basic query method to RAGAgent that returns simple responses without retrieval context
- [X] T010 [US1] [P] Create basic agent service in backend/src/basic_agent_service.py that orchestrates agent responses
- [X] T011 [US1] [P] Implement simple query method in BasicAgentService that calls agent without retrieval
- [X] T012 [US1] Add query ID generation to BasicAgentService for tracking purposes
- [X] T013 [US1] Add execution time tracking to BasicAgentService query method
- [X] T014 [US1] [P] Create basic chat service interface in backend/api/services/chat.py
- [X] T015 [US1] Integrate basic agent response with chat service and test end-to-end functionality

**Validation**: Run basic query through system and verify response generation works

---

## Phase 4: User Story 2 - Agent Integration with Retrieval Tool (P1)

**Goal**: Integrate the agent with the existing retrieval pipeline via function_tool so responses are grounded in book content

**Why P1**: Critical for RAG functionality - responses must be grounded in actual book content

**Independent Test**: Submit query "What is ROS2?" and verify:
- Agent calls retrieval function_tool internally
- Response contains information from retrieved book chunks
- Sources are properly cited in response

**Tasks**:

- [X] T016 [P] [US2] Create RetrievalTool function_tool in backend/src/retrieval_tool.py that wraps Specs 1-2 retrieval pipeline
- [X] T017 [P] [US2] Implement retrieve_context method in RetrievalTool that calls RetrievalPipeline.query()
- [X] T018 [US2] Integrate RetrievalTool with RAGAgent using function_tool decorator for retrieval integration
- [X] T019 [US2] Update RAGAgent to call retrieval tool when processing queries via Runner.run
- [X] T020 [US2] Format retrieved context for proper injection into agent responses
- [X] T021 [US2] Validate that agent responses now contain information from retrieved book content

**Validation**: Query "What is ROS2?" returns response with content from module-01-ros2 files

---

## Phase 5: User Story 3 - FastAPI Chat Endpoint (P2)

**Goal**: Expose agent functionality through a stable FastAPI chat endpoint

**Why P2**: Required for external integration, but basic agent functionality works without it

**Independent Test**: Make HTTP request to /chat endpoint and verify:
- Accepts JSON query and returns JSON response
- Response follows API contract schema
- Error conditions handled properly

**Tasks**:

- [X] T022 [P] [US3] Create FastAPI application in backend/api/main.py with proper configuration
- [X] T023 [P] [US3] Implement POST /chat endpoint in backend/api/routes/chat.py with request/response validation
- [X] T024 [US3] Add request validation to ensure query is non-empty and within length limits
- [X] T025 [US3] Add response formatting to match API contract schema per contracts/chat_api.md
- [X] T026 [US3] Implement proper error handling with appropriate HTTP status codes
- [X] T027 [P] [US3] Add source attribution to responses with proper citation format
- [X] T028 [US3] Test complete API flow with various query types and validate response format

**Validation**: `curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'` returns properly formatted response

---

## Phase 6: User Story 4 - Session Management (P2)

**Goal**: Implement session management using SQLiteSession for conversation continuity

**Why P2**: Enables multi-turn conversations and context persistence, but basic functionality works without it

**Independent Test**: Make multiple requests in sequence and verify:
- Session context is maintained between requests
- Different sessions remain isolated
- Session data persists properly

**Tasks**:

- [X] T029 [P] [US4] Integrate SQLiteSession for session management in backend/src/basic_agent_service.py
- [X] T030 [P] [US4] Add session context management to BasicAgentService for conversation history
- [X] T031 [US4] Update RAGAgent to use Runner.run with proper session handling
- [X] T032 [US4] Test session persistence and isolation across multiple concurrent sessions

**Validation**: Multiple requests in sequence maintain conversation context while different sessions remain isolated

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final refinements and documentation

- [X] T033 Add comprehensive error handling throughout the system with proper logging
- [X] T034 Create validation checklist document at specs/001-agent-rag-backend/validation-checklist.md with test cases for all user stories
- [X] T035 Update documentation and create usage examples for the complete system

---

## Implementation Strategy

### MVP (Minimum Viable Product)

**Scope**: Complete through User Story 1 (Tasks T001-T015)

**Delivers**:
- Working agent system with basic responses using OpenAI Agent SDK
- End-to-end API functionality
- Query ID and execution time tracking
- Basic service orchestration

**Validation**: Can submit queries and get basic agent responses

### Incremental Delivery

After MVP, add features independently:

1. **US2 (T016-T021)**: Add retrieval integration with function_tool - 2-3 hours
2. **US3 (T022-T028)**: Add API endpoints - 2 hours
3. **US4 (T029-T032)**: Add session management - 1 hour
4. **Polish (T033-T035)**: Final touches - 1 hour

**Total estimated time**: ~7-8 hours for full implementation

### Parallel Execution Examples

**After Foundational Phase**:
```bash
# US2, US3, and US4 can run in parallel after foundational phase
Developer A: Implements US2 (T016-T021) - Agent integration
Developer B: Implements US3 (T022-T028) - API endpoints
Developer C: Implements US4 (T029-T032) - Session management
```

---

## Validation Checklist Reference

After implementation, validate using `specs/001-agent-rag-backend/validation-checklist.md`:

1. **US1**: Basic query "What is ROS2?" returns agent response
2. **US2**: Agent uses retrieval function_tool and cites sources
3. **US3**: API endpoint follows contract and handles errors
4. **US4**: Sessions maintain context and remain isolated
5. **Cross-cutting**: Performance <3s, proper error handling, all features integrated

---

## File Reference

**New Files Created**:
- `backend/src/agent.py` - Core agent implementation with agents.Agent and LitellmModel (T008-T009, T018-T020)
- `backend/src/retrieval_tool.py` - Retrieval integration with function_tool (T016-T017)
- `backend/src/agent_service.py` - Service orchestrator with SQLiteSession (T007, T010-T011, T013, T029-T031)
- `backend/api/models/chat.py` - API data models (T004-T006)
- `backend/api/main.py` - FastAPI application (T022)
- `backend/api/routes/chat.py` - API routes (T023, T024, T026)
- `backend/api/services/chat.py` - Chat service (T014-T015)
- `specs/001-agent-rag-backend/validation-checklist.md` - Test cases (T034)

**Modified Files**:
- `backend/requirements.txt` - Add new dependencies (T003)

**Dependencies**:
- Reuses: `backend/.env`, `backend/src/retrieval.py` (from Specs 1-2)
- Requires: Gemini API access, Qdrant database with 'physicalai' collection, SQLite for session management

---

## Success Criteria Mapping

| Success Criteria | Validated By |
|------------------|--------------|
| SC-001: Response time <3s | T013 (execution time tracking), US3 validation |
| SC-002: Grounded responses | US2 validation with source citations |
| SC-003: Concurrent requests | US3 validation with load testing |
| SC-004: Book content responses | US2 validation with content verification |
| SC-005: Availability | US3 validation with uptime testing |
| SC-006: Function tool usage | US2 validation with retrieval tool calls |
| SC-007: Session management | US4 validation with context persistence |

---

## Notes

- **No automated tests**: Per spec requirement, validation is manual via API calls
- **Reuses Specs 1-2 infrastructure**: Same retrieval pipeline, Qdrant collection, environment
- **Ready for deployment**: FastAPI structure supports scaling and deployment
- **Independent stories**: US2, US3, US4 can be developed in any order after foundational phase