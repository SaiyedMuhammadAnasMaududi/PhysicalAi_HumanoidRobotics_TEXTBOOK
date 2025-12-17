# Streaming API Contract

**Version**: 1.0.0
**Updated**: 2025-12-17
**Backends**: Local (http://localhost:8000), Deployed (https://syedmuhammadanasmaududi-rag-chabot.hf.space)

## Endpoint

```
POST /api/chat/stream
Content-Type: application/json
Accept: text/event-stream
```

## Request Schema

```json
{
  "query": "string (required, max 500 characters)",
  "session_id": "string (optional, UUID v4)",
  "timestamp": "string (optional, ISO8601)"
}
```

**Validation Rules**:
- `query`: Non-empty, max 500 characters, sanitized for injection attacks
- `session_id`: Optional UUID v4 format
- `timestamp`: Optional ISO8601 timestamp

## Response Schema (SSE Stream)

### Event: `message`

```json
{
  "type": "chunk",
  "content": "string (partial response text)",
  "metadata": {
    "timestamp": "ISO8601",
    "source": "local | deployed"
  }
}
```

### Event: `complete`

```json
{
  "type": "complete",
  "content": "string (full response text)",
  "metadata": {
    "timestamp": "ISO8601",
    "source": "local | deployed",
    "duration_ms": 1234
  }
}
```

### Event: `error`

```json
{
  "type": "error",
  "content": "string (error message)",
  "metadata": {
    "timestamp": "ISO8601",
    "error_code": "VALIDATION_ERROR | BACKEND_ERROR | TIMEOUT"
  }
}
```

## Error Codes

| HTTP Status | Error Code | Description | User Message |
|-------------|------------|-------------|--------------|
| 400 | VALIDATION_ERROR | Invalid request format | "Invalid query - please check your input" |
| 500 | BACKEND_ERROR | Backend processing failed | "Unable to process request - please try again" |
| 503 | SERVICE_UNAVAILABLE | Backend unreachable | "Service temporarily unavailable" |
| 504 | TIMEOUT | Request timeout (>30s) | "Request timed out - please try again" |

## CORS Requirements

**Required Headers** (backend must set):
```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: POST, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

## Example Request/Response

### Request
```bash
curl -X POST http://localhost:8000/api/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?", "session_id": "550e8400-e29b-41d4-a716-446655440000"}'
```

### Response (SSE Stream)
```
event: message
data: {"type":"chunk","content":"ROS2 is ","metadata":{"timestamp":"2025-12-17T12:00:00Z","source":"local"}}

event: message
data: {"type":"chunk","content":"the Robot Operating System ","metadata":{"timestamp":"2025-12-17T12:00:01Z","source":"local"}}

event: complete
data: {"type":"complete","content":"ROS2 is the Robot Operating System version 2...","metadata":{"timestamp":"2025-12-17T12:00:05Z","source":"local","duration_ms":5000}}
```

## Timeout Behavior

- Frontend timeout: 30 seconds
- Backend should respond within 25 seconds or send partial response
- On timeout, frontend closes connection and displays timeout error

## Retry Strategy

- Max 3 retries on network errors
- Exponential backoff: 1s, 2s, 4s
- No retry on validation errors (4xx)
