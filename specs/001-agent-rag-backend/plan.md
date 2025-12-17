# Implementation Plan: Agent-Based RAG Backend using OpenAI Agent SDK with LiteLLM Models

**Branch**: `001-agent-rag-backend` | **Date**: 2025-12-17 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-agent-rag-backend/spec.md`

## Summary

Implementation of an AI agent-based RAG (Retrieval-Augmented Generation) system that integrates with the existing retrieval pipeline from Specs 1-2. The system uses OpenAI Agent SDK with LiteLLM models (via LitellmModel) to process natural language queries about technical book content. The agent uses function_tool to retrieve relevant context from Qdrant and executes via Runner.run. Functionality is exposed through a FastAPI chat endpoint with session management via SQLiteSession. The agent generates responses grounded in the retrieved book content rather than hallucinated information.

## Technical Context

**Language/Version**: Python 3.11 (same as previous specs)
**Primary Dependencies**: agents, litellm, fastapi, uvicorn, python-dotenv, sqlite3
**Storage**: Existing Qdrant collection 'physicalai' (from Spec 2) + SQLite for sessions
**Testing**: Manual validation via CLI, no pytest required per user request
**Target Platform**: Local development environment (Linux/WSL)
**Project Type**: Backend API service extending existing backend structure
**Performance Goals**: Query processing (retrieval + agent response) < 3 seconds for 95% of queries
**Constraints**: Must use existing retrieval pipeline from Specs 1-2, LitellmModel for model integration, agents.Agent for agent functionality, SQLiteSession for session management, local execution only
**Scale/Scope**: Support up to 10 concurrent chat requests, handle all 30 book modules content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Compliance with Constitution Principles:**
- ✅ Accuracy and Source Verification: Agent responses will include citations to specific book sections from retrieved context
- ✅ Traceability and Reproducibility: All retrieved context and agent responses will be logged with source attribution and session tracking
- ✅ Reliability and Performance: Graceful error handling for connection failures and invalid inputs, 3-second response time goal
- ✅ Extensibility and Technology Standards: Uses modular design separating agent, retrieval, session, and API layers for future extensibility
- ✅ Security and Ethical Standards: No user data storage beyond session management, validation-only pipeline, proper error handling

## Project Structure

### Documentation (this feature)

```text
specs/001-agent-rag-backend/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (backend directory)

```text
backend/
├── src/
│   ├── main.py                    # Embedding pipeline from Spec 1
│   ├── retrieval.py               # Query retrieval module from Spec 2
│   ├── agent.py                   # NEW: OpenAI Agent SDK implementation with LitellmModel
│   └── __init__.py
├── api/
│   ├── main.py                    # NEW: FastAPI application entry point
│   ├── routes/
│   │   └── chat.py                # NEW: Chat endpoint implementation
│   └── models/
│       └── chat.py                # NEW: Chat request/response models
├── scripts/
│   ├── setup_embedding_env.sh     # From Spec 1
│   ├── run_embedding_pipeline.py  # From Spec 1
│   └── test_retrieval.py          # From Spec 2
├── tests/
│   └── unit/
│       ├── test_main.py           # From Spec 1
│       └── test_retrieval.py      # From Spec 2
└── requirements.txt               # Updated with new dependencies
```

**Structure Decision**: Extend existing backend structure from previous specs. The agent.py module will contain the agents.Agent implementation with LitellmModel integration, function_tool for retrieval, and Runner.run execution. The chat.py will provide the FastAPI endpoint, and the system will reuse the retrieval.py module from Specs 1-2. This maintains consistency with the established architecture while adding the agent functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## Phase 0: Research

### Research Questions

1. **OpenAI Agent SDK Integration**:
   - How to properly initialize and configure an agents.Agent with LitellmModel?
   - Answer: Use agents.Agent with LitellmModel for model configuration and function_tool for retrieval integration
   - Reference: OpenAI Agent SDK documentation

2. **LiteLLM Model Integration**:
   - How to configure LitellmModel for proper model integration with agents.Agent?
   - Answer: Use LitellmModel(model=model_name, api_key=api_key) for model configuration
   - Reference: LiteLLM documentation

3. **Function Tool Strategy**:
   - What's the optimal way to expose retrieval as a function_tool for the agent?
   - Answer: Create a function_tool that wraps the existing retrieval pipeline from Specs 1-2
   - Reference: Agent SDK tool integration patterns

4. **Runner Execution**:
   - How to properly execute agent via Runner.run with proper session management?
   - Answer: Use Runner.run(agent, input, context, session) with SQLiteSession
   - Reference: Agent SDK execution documentation

### Research Outputs

**File**: `specs/001-agent-rag-backend/research.md`

- OpenAI Agent SDK initialization and configuration patterns with LitellmModel
- LiteLLM integration and model configuration strategies
- Function tool patterns for RAG systems
- Runner execution and session management approaches

## Phase 1: Design

### Architecture Decisions

**Decision 1: Agent-Tool Architecture**
- **Options Considered**:
  - A) agents.Agent with function_tool for retrieval
  - B) Custom agent wrapper approach
  - C) Hybrid approach with both patterns available
- **Decision**: Option A - agents.Agent with function_tool for retrieval
- **Rationale**: Provides cleaner integration with OpenAI Agent SDK, follows standard patterns, allows agent to decide when to retrieve
- **Document in**: ADR not required (simple architectural choice)

**Decision 2: Model Integration via LitellmModel**
- **Options Considered**:
  - A) Direct OpenAI API calls
  - B) LitellmModel with configurable parameters
  - C) Multiple model support with fallbacks
- **Decision**: Option B - LitellmModel with configurable parameters
- **Rationale**: Provides flexibility for model experimentation, cost optimization, and proper agent SDK integration
- **Document in**: quickstart.md configuration section

**Decision 3: Function Tool Integration Method**
- **Options Considered**:
  - A) System message injection with retrieved context
  - B) function_tool injection during agent execution
  - C) Pre-processed context as part of user message
- **Decision**: Option B - function_tool injection during agent execution
- **Rationale**: Maintains proper separation of concerns, allows agent to reason about retrieved context naturally
- **Document in**: data-model.md

**Decision 4: Session Management Strategy**
- **Options Considered**:
  - A) In-memory session storage
  - B) SQLiteSession for persistent sessions
  - C) Hybrid approach with configurable storage
- **Decision**: Option B - SQLiteSession for persistent sessions
- **Rationale**: Provides reliable session persistence and isolation between users
- **Document in**: Plan (this file) - session management approach

### Component Breakdown

**Component 1: RAGAgent**
- **Purpose**: AI agent that processes user queries and integrates with retrieval pipeline via function_tool
- **Inputs**: user query, LitellmModel configuration, retrieval function_tool
- **Outputs**: agent response with grounded information from book content
- **Validation**: Response grounding verification, retrieval invocation confirmation

**Component 2: RetrievalTool**
- **Purpose**: function_tool that connects the agent to the existing retrieval pipeline from Specs 1-2
- **Inputs**: query from agent, retrieval parameters
- **Outputs**: relevant book content chunks from Qdrant
- **Validation**: Proper context retrieval, source attribution, similarity scoring

**Component 3: ChatEndpoint (FastAPI)**
- **Purpose**: RESTful API endpoint that accepts user queries and returns agent responses
- **Inputs**: user query via HTTP request
- **Outputs**: agent response via JSON response
- **Validation**: Proper request/response handling, error propagation, response timing

**Component 4: AgentService (Orchestrator)**
- **Purpose**: Coordinate end-to-end query processing between API, agent (via Runner.run), and session management (via SQLiteSession)
- **Inputs**: Configuration from .env, user query parameters, session context
- **Outputs**: Coordinated agent response with proper formatting and session tracking
- **Validation**: Full pipeline integration test

### Data Model

**File**: `specs/001-agent-rag-backend/data-model.md`

**New Entities**:

1. **ChatRequest**
   - query: str (user's natural language question)
   - user_context: dict (optional user-specific context)

2. **ChatResponse**
   - response: str (agent-generated answer)
   - sources: List[dict] (citations to book sections used)
   - execution_time_ms: float (time taken to process query)
   - timestamp: datetime (when response was generated)
   - query_id: str (unique identifier for tracking)

3. **RetrievedChunk** (from Specs 1-2, reused)
   - All fields from DocumentChunk: chunk_id, source_file, section_title, content, content_hash, chunk_sequence, total_chunks, processing_timestamp, token_count
   - similarity_score: float (0.0 to 1.0, higher = more similar)
   - rank: int (1-indexed position in results)

4. **SessionData** (for context management)
   - session_id: str (unique identifier for the session)
   - conversation_history: list of previous interactions
   - user_context: dict (context maintained across the session)

### API Contracts

**File**: `specs/001-agent-rag-backend/contracts/chat_api.md`

```python
# POST /chat
# Accepts user query and returns agent response

Request:
{
  "query": "What is ROS2 and how does it work?",
  "user_context": {}  # Optional additional context
}

Response (Success - 200):
{
  "response": "ROS2 (Robot Operating System 2) is an open-source framework...",
  "sources": [
    {
      "source_file": "../docs/module-01-ros2/01-introduction.md",
      "section_title": "Introduction to ROS 2",
      "chunk_id": "uuid-here",
      "similarity_score": 0.7123
    }
  ],
  "execution_time_ms": 1577,
  "timestamp": "2025-12-17T02:05:52.449099",
  "query_id": "req-uuid-here"
}

Response (Error - 400/500):
{
  "error": "Error message describing the issue",
  "timestamp": "2025-12-17T02:05:52.449099"
}
```

### Quickstart Guide

**File**: `specs/001-agent-rag-backend/quickstart.md`

**5-Minute Setup**:

1. **Prerequisites**: Specs 1 and 2 completed, .env configured with API keys, Qdrant collection populated
2. **Install dependencies**: `pip install -r backend/requirements.txt` (includes new agent dependencies)
3. **Start the service**: `python -m backend.api.main`
4. **Test the endpoint**: `curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'`
5. **Review response**: Check that response contains grounded information with proper source citations

**Configuration**:
- GEMINI_API_KEY: API key for Gemini model access
- LITELLM_MODEL: Model to use (default: gemini/gemini-2.0-flash)
- COHERE_API_KEY: API key for retrieval (from Specs 1-2)
- QDRANT_URL: URL for Qdrant database (from Specs 1-2)
- QDRANT_API_KEY: API key for Qdrant (from Specs 1-2)

## Phase 2: Task Breakdown

**Note**: Detailed tasks will be generated in `/sp.tasks` command. This section provides high-level phases.

### Phase 2.1: Core Agent Implementation
- Implement RAGAgent with agents.Agent and LitellmModel
- Create RetrievalTool as function_tool to connect to Specs 1-2 pipeline
- Implement session management with SQLiteSession
- Add proper error handling and logging

### Phase 2.2: FastAPI Integration
- Create chat endpoint at /chat
- Implement request/response validation
- Add execution time tracking
- Implement proper error responses

### Phase 2.3: Validation and Testing
- Test agent responses using retrieved context via function_tool
- Verify retrieval invoked on each query via Runner.run
- Validate FastAPI endpoint returns proper responses
- Confirm responses remain grounded in book content

## Testing Strategy

### Validation Approach

**No automated tests required** per user request. Use manual validation with direct API calls.

**Validation Checklist** (`specs/001-agent-rag-backend/validation-checklist.md`):

1. **Basic Functionality**:
   - [ ] Query "What is ROS2?" returns grounded response with book citations
   - [ ] Responses include proper source attribution
   - [ ] All metadata fields present in responses

2. **Agent Integration**:
   - [ ] Agent successfully calls retrieval function_tool
   - [ ] Retrieved context is properly injected into responses
   - [ ] Agent uses LitellmModel for model integration

3. **API Endpoints**:
   - [ ] POST /chat accepts queries and returns responses
   - [ ] Response includes execution time and sources
   - [ ] Error conditions handled gracefully

4. **Session Management**:
   - [ ] Multiple queries in sequence maintain context
   - [ ] Different sessions remain isolated
   - [ ] Session data persists properly

5. **Consistency**:
   - [ ] Same query repeated 3 times returns consistent grounded responses
   - [ ] Results match expected source files (ROS2 → module-01, Gazebo → module-02)

6. **Performance**:
   - [ ] 95% of queries return within 3 seconds
   - [ ] Concurrent requests handled without degradation

### Sample Queries for Validation

```bash
# Basic functionality
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'

# Cross-module queries
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "How do I connect perception to control?"}'

# Edge cases
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": ""}'  # Empty query
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "a"}'  # Single character
```

## Dependencies

- **Specs 1-2 (007-book-embedding-pipeline, 008-retrieval-query-validation)**: REQUIRED - Retrieval pipeline must be available
- **OpenAI Agent SDK**: REQUIRED - For agents.Agent implementation
- **LiteLLM**: REQUIRED - For LitellmModel integration
- **FastAPI**: REQUIRED - For web API framework
- **SQLite**: REQUIRED - For session management
- **.env Configuration**: REQUIRED - API keys and configuration settings

## Implementation Notes

### Key Technical Decisions

1. **Agent-Tool Integration**: Use agents.Agent with function_tool for retrieval integration
2. **Model Integration**: Use LitellmModel for proper model configuration
3. **Execution**: Use Runner.run for agent execution management
4. **Session Management**: Use SQLiteSession for persistent conversation context
5. **Error Handling**: Fail fast with clear messages to prevent hallucination
6. **Response Formatting**: Include source citations in structured format
7. **Performance**: Target <3 second response time for 95% of queries

### Performance Expectations

- Retrieval pipeline: <1.5 seconds (from Specs 1-2)
- Agent processing: <1.5 seconds
- **Total**: <3 seconds for 95% of queries (meets SC-001)

### Configuration via Environment

Reuse .env from previous specs with additions:
```bash
# From previous specs
COHERE_API_KEY=<key>
QDRANT_URL=<url>
QDRANT_API_KEY=<key>
QDRANT_COLLECTION=physicalai
EMBEDDING_MODEL=embed-multilingual-v3.0

# New for agent
GEMINI_API_KEY=<key>
LITELLM_MODEL=gemini/gemini-2.0-flash
AGENT_TEMPERATURE=0.1  # Low temperature for consistency
```

## Next Steps

1. Run `/sp.tasks` to generate detailed task list
2. Implement agent.py module with agents.Agent and LitellmModel integration
3. Create retrieval function_tool that connects to existing pipeline
4. Create FastAPI endpoints in api/ directory
5. Execute validation checklist
6. Document optimal configurations in quickstart.md
7. Ready for deployment and integration with frontend (future spec)