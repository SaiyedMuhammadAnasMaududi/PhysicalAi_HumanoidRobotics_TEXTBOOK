# Tasks: Frontend–Backend Integration for Streaming RAG Chatbot

**Input**: Design documents from `/specs/001-frontend-backend-integration/`
**Prerequisites**: plan.md, spec.md, contracts/streaming-api.md

**Tests**: Manual validation tasks included (no automated tests requested in spec)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Repository uses web application structure with frontend at repository root:
- Frontend: `src/components/`, `src/services/`, `src/utils/`, `src/theme/`
- Config: `.env.example`, `docusaurus.config.ts`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Environment configuration and documentation

- [x] T001 Create `.env.example` with BACKEND_URL, STREAMING_ENDPOINT, TIMEOUT_MS, RETRY_ATTEMPTS
- [x] T002 [P] Update `docusaurus.config.ts` to expose environment variables to client-side code
- [x] T003 [P] Document backend configuration in `specs/001-frontend-backend-integration/contracts/streaming-api.md` (already exists, verify completeness)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core services infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Enhance `src/utils/errorHandler.js` to add backend-specific error types (VALIDATION_ERROR, BACKEND_ERROR, SERVICE_UNAVAILABLE, TIMEOUT)
- [x] T005 [P] Modify `src/services/chatService.js` to read BACKEND_URL from environment variables and construct streaming endpoint URL
- [x] T006 [P] Add URL validation function to `src/services/chatService.js` to verify backend URL format

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Connect to Local Backend (Priority: P1) 🎯 MVP

**Goal**: Enable chatbot UI to connect to local backend (http://localhost:8000) with streaming support

**Independent Test**:
1. Start local backend at http://localhost:8000
2. Configure BACKEND_URL=http://localhost:8000 in `.env.local`
3. Send query "What is ROS2?" in chat interface
4. Verify streaming response renders word-by-word without errors

### Implementation for User Story 1

- [x] T007 [P] [US1] Implement EventSource connection in `src/services/streamingHandler.js` for SSE streaming
- [x] T008 [P] [US1] Add chunk parsing logic in `src/services/streamingHandler.js` to handle JSON SSE messages (type: chunk/complete/error)
- [x] T009 [US1] Add timeout handling in `src/services/streamingHandler.js` (30-second timeout, reset on each chunk)
- [x] T010 [US1] Add retry logic in `src/services/streamingHandler.js` (3 attempts, exponential backoff: 1s, 2s, 4s)
- [x] T011 [US1] Modify `src/components/ChatInterface/ChatContainer.jsx` to accept streaming chunks and update message incrementally
- [x] T012 [US1] Update `src/components/ChatInterface/ChatContainer.jsx` to show typing indicator during streaming
- [x] T013 [US1] Add error display logic in `src/components/ChatInterface/ChatContainer.jsx` for connection failures

### Validation for User Story 1

- [x] T014 [US1] Manual test: Verify local backend connection with streaming query renders correctly
- [x] T015 [US1] Manual test: Verify timeout error appears after 30s with slow/unresponsive local backend
- [x] T016 [US1] Manual test: Verify retry logic triggers on network interruption (disconnect/reconnect local backend)

**Checkpoint**: User Story 1 complete - local backend streaming fully functional

---

## Phase 4: User Story 2 - Connect to Deployed Backend (Priority: P2)

**Goal**: Enable chatbot UI to connect to deployed backend (https://syedmuhammadanasmaududi-rag-chabot.hf.space) with HTTPS and CORS support

**Independent Test**:
1. Configure BACKEND_URL=https://syedmuhammadanasmaududi-rag-chabot.hf.space in `.env.production`
2. Build production bundle
3. Send query "Explain digital twins" in chat interface
4. Verify streaming response renders word-by-word over HTTPS without CORS errors

### Implementation for User Story 2

- [x] T017 [P] [US2] Add HTTPS support validation in `src/services/chatService.js` (ensure deployed URL uses HTTPS)
- [x] T018 [P] [US2] Add CORS error detection in `src/utils/errorHandler.js` to provide user-friendly message for CORS failures
- [x] T019 [US2] Update `src/services/streamingHandler.js` to handle HTTPS-specific errors (certificate validation, secure connection failures)
- [x] T020 [US2] Test and verify SSE connection works with deployed backend HTTPS endpoint

### Validation for User Story 2

- [x] T021 [US2] Manual test: Verify deployed backend connection with streaming query renders correctly over HTTPS
- [x] T022 [US2] Manual test: Verify CORS error handling displays clear message if backend missing CORS headers
- [x] T023 [US2] Manual test: Verify SSL certificate validation works without browser warnings

**Checkpoint**: User Story 2 complete - deployed backend streaming fully functional

---

## Phase 5: User Story 3 - Switchable Backend Configuration (Priority: P3)

**Goal**: Enable switching between local and deployed backends via environment configuration without code changes

**Independent Test**:
1. Build with BACKEND_URL=http://localhost:8000
2. Verify chat connects to local backend
3. Rebuild with BACKEND_URL=https://syedmuhammadanasmaududi-rag-chabot.hf.space
4. Verify chat connects to deployed backend
5. Both environments should work without any code modifications

### Implementation for User Story 3

- [x] T024 [P] [US3] Update `src/theme/Root.js` to inject backend configuration from environment variables at runtime
- [x] T025 [P] [US3] Export config utilities from `src/components/ChatInterface/index.jsx` for external consumption
- [x] T026 [US3] Add configuration validation in `src/services/chatService.js` to warn if BACKEND_URL is missing or invalid
- [x] T027 [US3] Create `.env.local` example configuration for local development
- [x] T028 [US3] Create `.env.production` example configuration for deployed backend

### Validation for User Story 3

- [x] T029 [US3] Manual test: Build with local backend URL and verify connection to http://localhost:8000
- [x] T030 [US3] Manual test: Rebuild with deployed backend URL and verify connection to HTTPS deployed endpoint
- [x] T031 [US3] Manual test: Verify zero code changes required between builds (only env file changes)

**Checkpoint**: User Story 3 complete - environment switching works seamlessly

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, edge cases, and validation across all user stories

- [x] T032 [P] Add malformed response handling in `src/services/streamingHandler.js` (detect invalid JSON, display parse error)
- [x] T033 [P] Add concurrent streaming cancellation in `src/services/streamingHandler.js` (close previous EventSource when new query sent)
- [x] T034 [P] Add partial response preservation in `src/services/streamingHandler.js` (save incomplete responses when stream interrupted)
- [x] T035 [P] Update error messages in `src/utils/errorHandler.js` to be user-friendly for all error scenarios
- [x] T036 Add logging for backend connection events in `src/services/chatService.js` (connection, timeout, retry, error)
- [x] T037 [P] Update documentation in `.env.example` with detailed comments for all configuration options
- [x] T038 Verify all 7 test cases from plan.md work correctly (local connection, deployed connection, environment switching, errors, timeouts, malformed responses, concurrent streaming)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories (independent)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates US1 and US2 functionality but should work independently

### Within Each User Story

- Implementation tasks before validation tasks
- Core streaming logic before error handling
- Service layer before component layer
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks (T001-T003) marked [P] can run in parallel
- All Foundational tasks (T004-T006) marked [P] can run in parallel within Phase 2
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Tasks within each story marked [P] can run in parallel (different files)

---

## Parallel Example: User Story 1

```bash
# Launch parallel tasks for User Story 1:
Task T007: "Implement EventSource connection in src/services/streamingHandler.js"
Task T008: "Add chunk parsing logic in src/services/streamingHandler.js"

# Then sequential tasks:
Task T009: "Add timeout handling" (depends on T007, T008)
Task T010: "Add retry logic" (depends on T007)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently with local backend
5. Demo local backend streaming functionality

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Demo local backend (MVP!)
3. Add User Story 2 → Test independently → Demo deployed backend
4. Add User Story 3 → Test independently → Demo environment switching
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Local backend)
   - Developer B: User Story 2 (Deployed backend)
   - Developer C: User Story 3 (Configuration switching)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Frontend-only changes - no backend modifications required
- Manual validation sufficient (no automated tests requested in spec)
