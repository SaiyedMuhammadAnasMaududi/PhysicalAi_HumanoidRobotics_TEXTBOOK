# API Contract: Chat Streaming Interface

**Feature**: 002-chatbot-ui-streaming
**Date**: 2025-12-17
**Input**: Implementation plan and data model

## Overview

This contract defines the interface between the streaming chatbot UI and the backend RAG system. It specifies the request/response formats, streaming protocol, and error handling for real-time message delivery.

## Streaming Protocol

The chat interface uses Server-Sent Events (SSE) for real-time streaming responses from the backend.

### Connection Establishment
- **Method**: POST or GET (depending on implementation)
- **Content-Type**: application/json for initial request
- **Response-Type**: text/event-stream for streaming response
- **Protocol**: Server-Sent Events (EventSource API compatible)

## Request Format

### Chat Message Request
```
POST /api/chat/stream
Content-Type: application/json
```

**Request Body**:
```json
{
  "query": "What is ROS2 and how does it work?",
  "user_context": {}
}
```

**Schema**:
```json
{
  "type": "object",
  "properties": {
    "query": {
      "type": "string",
      "minLength": 1,
      "maxLength": 1000,
      "description": "The user's natural language question"
    },
    "user_context": {
      "type": "object",
      "nullable": true,
      "description": "Optional additional context provided by the user"
    }
  },
  "required": ["query"]
}
```

## Response Format

### Streaming Response (Server-Sent Events)

The backend responds with a stream of Server-Sent Events in the following format:

```
event: message
data: {"content": "ROS2", "done": false, "error": null}

event: message
data: {"content": " (Robot Operating System 2) is", "done": false, "error": null}

event: message
data: {"content": " an open-source framework for robot software development.", "done": true, "error": null}
```

**Event Data Schema**:
```json
{
  "type": "object",
  "properties": {
    "content": {
      "type": "string",
      "description": "The text content of this stream chunk"
    },
    "done": {
      "type": "boolean",
      "description": "True if this is the final chunk in the response"
    },
    "error": {
      "type": ["string", "null"],
      "description": "Error message if this chunk represents an error, otherwise null"
    }
  },
  "required": ["content", "done", "error"]
}
```

## Error Responses

### Connection Errors
When the initial connection fails:
- **Status Code**: 400, 401, 403, 429, 500, 503 (as appropriate)
- **Content-Type**: application/json
- **Response**:
```json
{
  "error": "Error message describing the issue",
  "timestamp": "2025-12-17T02:05:52.449099",
  "retry_after": 30
}
```

### Streaming Errors
When an error occurs during streaming:
```
event: error
data: {"error": "Connection lost", "retry_after": 5}
```

**Error Event Schema**:
```json
{
  "type": "object",
  "properties": {
    "error": {
      "type": "string",
      "description": "Error message describing the issue"
    },
    "retry_after": {
      "type": "number",
      "description": "Suggested delay before retrying (in seconds)"
    }
  },
  "required": ["error", "retry_after"]
}
```

## Backend Integration Points

### Required Endpoints

1. **Streaming Chat Endpoint**
   - **Path**: `/api/chat/stream`
   - **Method**: POST
   - **Purpose**: Accept user queries and return streaming responses
   - **Response**: Server-Sent Events stream

2. **Health Check Endpoint** (optional but recommended)
   - **Path**: `/health`
   - **Method**: GET
   - **Purpose**: Check backend availability
   - **Response**:
   ```json
   {
     "status": "healthy",
     "timestamp": "2025-12-17T02:05:52.449099"
   }
   ```

## Frontend Implementation Requirements

### Client-Side Handling

1. **Connection Management**
   - Use EventSource API for SSE connections
   - Implement automatic reconnection with exponential backoff
   - Handle connection timeouts gracefully

2. **Message Processing**
   - Accumulate content from multiple chunks
   - Update UI incrementally as chunks arrive
   - Complete message when `done: true`

3. **Error Handling**
   - Display user-friendly error messages
   - Preserve partial responses when possible
   - Provide retry functionality

### Example Client Implementation

```javascript
// Establish SSE connection
const eventSource = new EventSourcePolyfill(`/api/chat/stream`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ query: userQuery })
});

// Handle message chunks
eventSource.addEventListener('message', (event) => {
  const data = JSON.parse(event.data);

  if (data.error) {
    // Handle error
    displayError(data.error);
  } else {
    // Update message with new content
    updateStreamingMessage(data.content, data.done);
  }
});

// Handle connection errors
eventSource.addEventListener('error', (error) => {
  handleConnectionError(error);
});
```

## Performance Requirements

### Response Time
- **Connection establishment**: <1 second
- **First content chunk**: <2 seconds (backend dependent)
- **Subsequent chunks**: As they become available from backend
- **Error response**: <500ms

### Concurrency
- Should support multiple concurrent streaming connections per client
- Backend should handle multiple concurrent users (≥10 as per constitution)

## Security Considerations

### Input Validation
- Query length limited to prevent abuse
- Input sanitization to prevent injection attacks
- Rate limiting to prevent abuse

### Output Validation
- Content should be properly escaped for HTML display
- No sensitive information in error messages
- Proper CORS headers for cross-origin requests

## Testing Considerations

### Contract Testing
- Verify SSE format compliance
- Test error condition handling
- Validate schema compliance for all message types
- Test connection timeout scenarios

### Integration Testing
- End-to-end streaming functionality
- Error recovery scenarios
- Multiple concurrent connections
- Large response handling