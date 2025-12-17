# Data Model: Intermediate Chatbot UI with Streaming

**Feature**: 002-chatbot-ui-streaming
**Date**: 2025-12-17
**Input**: Implementation plan from `plan.md`

## Overview

This document defines the data entities and their relationships for the streaming chatbot UI system. The model focuses on frontend representation of messages, streaming chunks, and application state for the chat interface that integrates with the RAG backend and Docusaurus textbook site.

## Entity Definitions

### 1. Message
**Purpose**: Represents a single conversational message in the chat interface

**Attributes**:
- `id`: string (required) - Unique identifier for the message
- `role`: string (required) - Sender role ("user" or "assistant")
- `content`: string (required) - Message text content
- `timestamp`: datetime (required) - When message was created/received
- `status`: string (required) - Message state ("sending", "streaming", "complete", "error")

**Constraints**:
- id must be unique within the conversation session
- role must be either "user" or "assistant"
- content must not be empty when status is "complete"
- timestamp must be reasonable (not in the future)

**Relationships**:
- Part of ChatState.messages array
- One-to-many with StreamChunk during streaming (temporary association)

### 2. StreamChunk
**Purpose**: Represents a fragment of streaming response data received from the backend

**Attributes**:
- `content`: string (required) - Text content of this chunk
- `done`: boolean (required) - Indicates if stream is complete
- `error`: string (optional) - Error message if chunk contains error information
- `timestamp`: datetime (required) - When chunk was received

**Constraints**:
- content can be empty for heartbeat chunks
- done and error are mutually exclusive (only one should be truthy)
- timestamp must be reasonable (not in the future)

**Relationships**:
- Aggregated into Message.content during streaming
- Associated with ChatState during active streaming

### 3. ChatState
**Purpose**: Represents the overall application state for the chat interface

**Attributes**:
- `messages`: array of Message (required) - Complete message history in chronological order
- `streamingStatus`: boolean (required) - Indicates if response is currently streaming
- `connectionStatus`: string (required) - Current connection state ("connected", "connecting", "disconnected", "error")
- `errorState`: object (optional) - Current error information with message and timestamp
- `scrollPosition`: object (optional) - Current scroll tracking with isAtBottom and position properties

**Constraints**:
- messages array should have reasonable size limits (e.g., max 1000 items)
- streamingStatus and connectionStatus should be consistent
- errorState should be null when connectionStatus is "connected"

**Relationships**:
- Contains multiple Message objects
- Manages active StreamChunk processing
- Controls UI behavior based on state

### 4. ChatConfig
**Purpose**: Configuration settings for the chat interface

**Attributes**:
- `backendUrl`: string (required) - API endpoint for backend connection
- `streamingEndpoint`: string (required) - Streaming API path
- `messageLimit`: number (required) - Maximum messages to keep in history
- `autoScrollEnabled`: boolean (required) - Whether auto-scroll behavior is active
- `theme`: object (optional) - Theme configuration for styling

**Constraints**:
- backendUrl must be a valid URL
- messageLimit must be positive integer
- streamingEndpoint must start with "/"

**Relationships**:
- Used by all components for configuration
- Influences ChatState behavior

## Data Flow

### Message Processing Flow
1. **Input**: User sends query via InputArea component
2. **State Update**: ChatState adds user Message with "sending" status
3. **API Call**: StreamingHandler connects to backend via SSE
4. **Streaming**: StreamChunk objects received incrementally
5. **UI Update**: Message content updated incrementally during streaming
6. **Completion**: Message status changes to "complete" when done

### Data Transformation
```
User Input → ChatState.messages.push() → SSE Connection → StreamChunk → Message.content += → UI Update
```

## Validation Rules

### Input Validation
- Message.content: Required, non-empty when complete, reasonable length (<10,000 chars)
- Message.role: Must be "user" or "assistant"
- ChatConfig.backendUrl: Must be valid URL format

### State Validation
- ChatState.messages: Should maintain chronological order
- Streaming state: Only one message should be in "streaming" state at a time
- Connection state: Should be consistent with actual connection status

### UI State Validation
- Scroll position should be maintained during user navigation
- Auto-scroll should only activate when user is at bottom
- Error states should be clearly displayed

## API Data Contracts

### Frontend to Backend Request
```json
{
  "type": "object",
  "properties": {
    "query": {"type": "string", "minLength": 1, "maxLength": 1000}
  },
  "required": ["query"]
}
```

### Backend to Frontend Streaming Response (SSE Format)
```json
{
  "type": "object",
  "properties": {
    "content": {"type": "string"},
    "done": {"type": "boolean"},
    "error": {"type": "string", "nullable": true}
  },
  "required": ["content", "done"]
}
```

### Internal Message Format
```json
{
  "type": "object",
  "properties": {
    "id": {"type": "string"},
    "role": {"type": "string", "enum": ["user", "assistant"]},
    "content": {"type": "string", "minLength": 1},
    "timestamp": {"type": "string", "format": "date-time"},
    "status": {"type": "string", "enum": ["sending", "streaming", "complete", "error"]}
  },
  "required": ["id", "role", "content", "timestamp", "status"]
}
```

## Performance Considerations

### Data Size Limits
- Individual message content: Should be <1MB to avoid memory issues
- Total conversation history: Should be limited to prevent memory bloat
- Streaming chunks: Should be optimized for network efficiency

### State Management
- Use immutable updates to prevent unexpected re-renders
- Implement proper cleanup for unmounted components
- Consider virtualization for very long conversation histories

## Evolution Considerations

### Future Extensions
- User context could be added to Message objects
- Rich content (images, code blocks) could extend content format
- Multi-user conversation could extend role field
- Message metadata could include source citations

### Backward Compatibility
- New optional fields can be added without breaking existing clients
- Required fields should not be removed or changed without versioning
- Field types should remain consistent (string stays string, number stays number)
- Message format should evolve with migration strategies

## Integration Notes

### With RAG Backend
- Message objects represent responses from RAG system
- StreamChunk format must match backend SSE output
- Error handling aligns with backend error responses

### With Docusaurus Integration
- ChatConfig uses Docusaurus theme variables
- Message styling follows Docusaurus design system
- Component structure integrates with Docusaurus page layout