# Data Model: Agent-Based RAG Backend using OpenAI Agent SDK with LiteLLM Models

**Feature**: 001-agent-rag-backend
**Date**: 2025-12-17
**Input**: Implementation plan from `plan.md`

## Overview

This document defines the data entities and their relationships for the agent-driven RAG backend system. It builds upon the existing retrieval data model from Specs 1-2 while adding new entities for the agent, session, and API layers using the OpenAI Agent SDK with LiteLLM Models.

## Entity Definitions

### 1. ChatRequest
**Purpose**: Represents a user's query to the chat system

**Attributes**:
- `query`: string (required) - The natural language question from the user
- `user_context`: object (optional) - Additional context provided by the user
- `timestamp`: datetime - When the request was received

**Constraints**:
- query must be non-empty
- query length should be reasonable (e.g., < 1000 characters)

**Relationships**:
- One-to-many with ChatResponse (one request generates one response)

### 2. ChatResponse
**Purpose**: Represents the agent's response to a user's query

**Attributes**:
- `response`: string (required) - The agent-generated answer
- `sources`: array of SourceReference - Citations to book sections used in response
- `execution_time_ms`: number - Time taken to process the query in milliseconds
- `timestamp`: datetime - When the response was generated
- `query_id`: string - Unique identifier for tracking

**Constraints**:
- response must be non-empty when successful
- sources array can be empty if no relevant context found
- execution_time_ms must be non-negative

**Relationships**:
- Many-to-one with ChatRequest (generated from one request)

### 3. SourceReference
**Purpose**: Represents a citation to a specific book section used in the response

**Attributes**:
- `source_file`: string (required) - Path to the source document
- `section_title`: string (required) - Title of the section containing the information
- `chunk_id`: string (required) - Unique identifier of the content chunk
- `similarity_score`: number (required) - Relevance score (0.0 to 1.0)
- `content_preview`: string (optional) - Short preview of the cited content

**Constraints**:
- similarity_score must be between 0.0 and 1.0
- chunk_id must be a valid UUID format
- All fields except content_preview are required

**Relationships**:
- Part of ChatResponse (embedded within response)

### 4. RetrievedChunk (from Specs 1-2 - reused)
**Purpose**: Represents a chunk of content retrieved from the vector database

**Attributes**:
- `chunk_id`: string (required) - Unique identifier for the chunk
- `source_file`: string (required) - Path to the original document
- `section_title`: string (required) - Title of the section
- `content`: string (required) - The actual content of the chunk
- `content_hash`: string (required) - SHA256 hash of the content for integrity
- `chunk_sequence`: number (required) - Position of this chunk in the document (1-indexed)
- `total_chunks`: number (required) - Total number of chunks in the document
- `processing_timestamp`: datetime (required) - When the chunk was processed
- `token_count`: number (required) - Number of tokens in the content
- `similarity_score`: number (required) - How similar this chunk is to the query (0.0 to 1.0)
- `rank`: number (required) - Position in results list (1-indexed)

**Constraints**:
- All fields are required
- similarity_score between 0.0 and 1.0
- chunk_sequence <= total_chunks
- token_count must be non-negative

**Relationships**:
- Used to create SourceReference objects
- Originates from Qdrant vector database

### 5. SessionData
**Purpose**: Represents a conversation session with context management

**Attributes**:
- `session_id`: string (required) - Unique identifier for the session
- `conversation_history`: array of Interaction - History of interactions in the session
- `user_context`: object (optional) - Context maintained across the session
- `created_at`: datetime - When the session was created
- `last_activity`: datetime - When the session was last used

**Constraints**:
- session_id must be unique
- created_at must be before or equal to last_activity
- conversation_history should be limited in size to prevent memory issues

**Relationships**:
- One-to-many with Interaction (contains multiple interactions)

### 6. Interaction
**Purpose**: Represents a single interaction within a session

**Attributes**:
- `interaction_id`: string (required) - Unique identifier for the interaction
- `session_id`: string (required) - Session this interaction belongs to
- `query`: string (required) - The user's query
- `response`: string (required) - The agent's response
- `timestamp`: datetime - When the interaction occurred
- `metadata`: object (optional) - Additional metadata about the interaction

**Constraints**:
- query and response must be non-empty
- timestamp must be reasonable (not in the future)

**Relationships**:
- Many-to-one with SessionData (belongs to one session)

## Data Flow

### Request Processing Flow
1. **Input**: User sends ChatRequest with a query
2. **Session**: System retrieves session context from SQLiteSession
3. **Processing**: Agent calls retrieval function_tool to get relevant RetrievedChunk objects from Qdrant
4. **Agent**: Agent processes query with retrieved context via Runner.run
5. **Output**: System creates ChatResponse with sources and updates session

### Data Transformation
```
ChatRequest.query
→ Runner.run(agent, query, context, session)
→ function_tool(retrieve_context(query))
→ List[RetrievedChunk]
→ Agent processing with context
→ ChatResponse with List[SourceReference]
→ SessionData updated with new Interaction
```

## Validation Rules

### Input Validation
- ChatRequest.query: Required, non-empty, reasonable length
- Query length: Should be < 1000 characters to prevent abuse
- Query content: Should contain actual question, not just special characters

### Response Validation
- ChatResponse.response: Required when successful
- ChatResponse.sources: Should match the RetrievedChunk objects used
- Execution time: Should be reasonable (< 30 seconds for 99% of requests)

### Source Attribution Validation
- All sources in response should correspond to actually retrieved chunks
- Similarity scores should match those from the retrieval system
- Source file paths should be valid and point to existing documents

### Session Validation
- Session data should be properly isolated between different users
- Session history should be maintained across multiple interactions
- Session data should be cleaned up periodically to prevent memory issues

## API Data Contracts

### Request Schema
```json
{
  "type": "object",
  "properties": {
    "query": {"type": "string", "minLength": 1, "maxLength": 1000},
    "user_context": {"type": "object", "nullable": true}
  },
  "required": ["query"]
}
```

### Response Schema
```json
{
  "type": "object",
  "properties": {
    "response": {"type": "string", "minLength": 1},
    "sources": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "source_file": {"type": "string"},
          "section_title": {"type": "string"},
          "chunk_id": {"type": "string"},
          "similarity_score": {"type": "number", "minimum": 0.0, "maximum": 1.0}
        },
        "required": ["source_file", "section_title", "chunk_id", "similarity_score"]
      }
    },
    "execution_time_ms": {"type": "number", "minimum": 0},
    "timestamp": {"type": "string", "format": "date-time"},
    "query_id": {"type": "string"}
  },
  "required": ["response", "sources", "execution_time_ms", "timestamp", "query_id"]
}
```

## Performance Considerations

### Data Size Limits
- Individual content chunks: Should be < 1MB to avoid memory issues
- Response size: Should be < 10MB to prevent bandwidth issues
- Number of sources: Should be < 100 to maintain readability
- Session history: Should be < 100 interactions to prevent memory issues

### Caching Opportunities
- Recently retrieved chunks could be cached in memory
- Agent responses for identical queries could be cached (with appropriate TTL)
- Session data could be cached to avoid frequent database access

## Evolution Considerations

### Future Extensions
- User context could be expanded to include preferences or history
- Source references could include page numbers or specific line numbers
- Conversation history could be enhanced with semantic clustering
- User authentication data could be included for personalized experiences

### Backward Compatibility
- New optional fields can be added without breaking existing clients
- Required fields should not be removed or changed without versioning
- Field types should remain consistent (string stays string, number stays number)
- Session data schema should evolve with migration strategies