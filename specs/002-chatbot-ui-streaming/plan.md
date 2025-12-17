# Implementation Plan: Intermediate Chatbot UI with Streaming

**Branch**: `002-chatbot-ui-streaming` | **Date**: 2025-12-17 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-chatbot-ui-streaming/spec.md`

## Summary

Implementation of an intermediate-level chatbot UI that provides real-time streaming responses from the RAG backend. The system includes message display with chat bubbles, streaming response rendering, auto-scroll behavior, loading states, and error handling. The UI will be synchronized with the existing Docusaurus-based Physical AI & Humanoid Robotics textbook site, maintaining consistent styling and navigation context.

## Technical Context

**Language/Version**: JavaScript/ES6+ (browser-compatible), HTML5, CSS3
**Primary Dependencies**: Vanilla JavaScript (no framework required), EventSource API for SSE, Fetch API for HTTP requests
**Storage**: N/A (frontend-only, no persistent storage per spec constraints)
**Testing**: Manual validation via browser testing (no automated tests per spec requirement)
**Target Platform**: Modern web browsers (Chrome 60+, Firefox 60+, Safari 12+, Edge 79+)
**Project Type**: Web frontend (integration with existing Docusaurus site)
**Performance Goals**: <50ms input response, 30+ FPS during streaming, <2s first token display (backend-dependent), <1s UI load time
**Constraints**: No authentication, no persistence, Docusaurus styling synchronization, lightweight implementation
**Scale/Scope**: Single conversation session per page load, supports 1000+ message conversations, optimized for localhost development

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Compliance with Constitution Principles:**
- ✅ Accuracy and Source Verification: UI will display source citations from backend responses per spec requirements
- ✅ Clarity and Accessibility: Chat interface provides clear, accessible messaging with proper visual indicators
- ✅ Traceability and Reproducibility: UI connects to existing RAG pipeline with provenance information
- ✅ Reliability and Performance: Auto-scroll and streaming optimizations ensure smooth user experience
- ✅ Extensibility and Technology Standards: Uses standard web technologies (SSE, Fetch API) for extensibility
- ✅ Security and Ethical Standards: No user data stored, input sanitized via browser security

## Project Structure

### Documentation (this feature)

```text
specs/002-chatbot-ui-streaming/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (integrated with Docusaurus)

```text
src/
├── components/
│   └── ChatInterface/
│       ├── ChatContainer.jsx          # Main chat container component
│       ├── MessageList.jsx            # Message display component
│       ├── MessageBubble.jsx          # Individual message bubble component
│       ├── InputArea.jsx              # Input field and send button
│       ├── TypingIndicator.jsx        # Animated typing indicator
│       └── ChatStyles.css             # Docusaurus-compatible styling
├── services/
│   ├── chatService.js                 # Backend communication logic
│   ├── streamingHandler.js            # Streaming response processing
│   └── scrollManager.js               # Auto-scroll behavior implementation
└── utils/
    ├── messageFormatter.js            # Message formatting utilities
    └── errorHandler.js                # Error handling utilities
```

**Structure Decision**: Web frontend component integrated within existing Docusaurus structure to maintain consistent styling and navigation with the Physical AI & Humanoid Robotics textbook. The chat UI will be embedded as a React component that can be included in Docusaurus pages, using the existing theme and styling system.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

## Phase 0: Research

### Research Questions

1. **Streaming Protocol Selection**:
   - Question: Should we use Server-Sent Events (SSE) or chunked HTTP for streaming responses?
   - Answer: SSE provides better browser support and simpler implementation for one-way streaming
   - Reference: Browser compatibility and backend API contract

2. **Message State Management**:
   - Question: What's the optimal approach for managing incremental message updates during streaming?
   - Answer: Use efficient DOM updates with virtual scrolling for large conversations
   - Reference: Performance optimization patterns

3. **Auto-Scroll Behavior**:
   - Question: How to implement smart auto-scroll that respects user scrolling intent?
   - Answer: Track scroll position and only auto-scroll when user is already at bottom
   - Reference: Chat UI best practices

4. **Docusaurus Integration**:
   - Question: How to integrate with existing Docusaurus theme without breaking existing styles?
   - Answer: Use CSS variables and class names that align with Docusaurus theme
   - Reference: Docusaurus theme customization documentation

### Research Outputs

**File**: `specs/002-chatbot-ui-streaming/research.md`

- SSE vs chunked HTTP comparison and selection
- Auto-scroll implementation patterns
- Message rendering optimization strategies
- Docusaurus styling integration approaches

## Phase 1: Design

### Architecture Decisions

**Decision 1: Streaming Implementation**
- **Options Considered**:
  - A) Server-Sent Events (SSE) for real-time streaming
  - B) Chunked HTTP responses with fetch API
  - C) WebSocket connection
- **Decision**: Option A - Server-Sent Events
- **Rationale**: Better browser support, simpler implementation for one-way streaming, native browser support
- **Document in**: research.md

**Decision 2: Message Rendering Strategy**
- **Options Considered**:
  - A) Virtual scrolling for large conversation history
  - B) Simple append approach with DOM cleanup
  - C) Hybrid approach with pagination
- **Decision**: Option B - Simple append with cleanup
- **Rationale**: Appropriate for single-session chat, simpler implementation, sufficient for spec requirements
- **Document in**: research.md

**Decision 3: State Management**
- **Options Considered**:
  - A) React hooks with local component state
  - B) Global state management (Redux/Context)
  - C) Browser native state (localStorage/sessionStorage)
- **Decision**: Option A - React hooks with local state
- **Rationale**: Single component scope, lightweight implementation, no persistence needed per spec
- **Document in**: research.md

**Decision 4: Docusaurus Styling Integration**
- **Options Considered**:
  - A) CSS variables matching Docusaurus theme
  - B) Direct CSS with Docusaurus class names
  - C) Custom theme plugin
- **Decision**: Option A - CSS variables approach
- **Rationale**: Maintains theme consistency, allows customization, follows Docusaurus best practices
- **Document in**: quickstart.md

### Component Breakdown

**Component 1: ChatContainer**
- **Purpose**: Main chat interface container that manages overall state and layout
- **Inputs**: Backend API configuration, styling preferences
- **Outputs**: Complete chat UI with message history and input area
- **Validation**: Proper state management during streaming, correct layout rendering

**Component 2: MessageList**
- **Purpose**: Displays ordered list of chat messages with proper scrolling behavior
- **Inputs**: Array of message objects, scroll position state
- **Outputs**: Scrollable message container with auto-scroll functionality
- **Validation**: Correct message ordering, smooth scrolling, proper auto-scroll behavior

**Component 3: MessageBubble**
- **Purpose**: Individual message display with role-based styling
- **Inputs**: Message object (role, content, timestamp), styling configuration
- **Outputs**: Styled message bubble with proper alignment and formatting
- **Validation**: Correct styling based on role (user/assistant), proper content rendering

**Component 4: InputArea**
- **Purpose**: User input field with send functionality
- **Inputs**: Send callback function, input validation state
- **Outputs**: Functional input area with send button and keyboard shortcuts
- **Validation**: Proper input validation, keyboard shortcut support, disabled state during streaming

**Component 5: StreamingHandler**
- **Purpose**: Manages SSE connection and incremental message updates
- **Inputs**: Backend API endpoint, message update callback
- **Outputs**: Real-time message content updates during streaming
- **Validation**: Proper connection management, error handling, incremental updates

### Data Model

**File**: `specs/002-chatbot-ui-streaming/data-model.md`

**New Entities**:

1. **Message**
   - id: string (unique identifier for the message)
   - role: string (user/assistant role)
   - content: string (message text content)
   - timestamp: datetime (when message was created)
   - status: string (sending/streaming/complete/error state)

2. **StreamChunk**
   - content: string (text content of this chunk)
   - done: boolean (indicates if stream is complete)
   - error: string (optional error message if chunk contains error)

3. **ChatState**
   - messages: array of Message (complete message history)
   - streamingStatus: boolean (indicates if response is currently streaming)
   - connectionStatus: string (current connection state)
   - errorState: object (current error information)
   - scrollPosition: object (current scroll tracking information)

### API Contracts

**File**: `specs/002-chatbot-ui-streaming/contracts/chat_api.md`

```javascript
// POST /api/chat (or streaming equivalent)
// Accepts user query and returns streaming response

Request:
{
  "query": "What is ROS2 and how does it work?"
}

// Streaming Response (Server-Sent Events format)
// Event: message
// Data: {"content": "ROS2", "done": false}

// Event: message
// Data: {"content": " (Robot Operating System 2) is", "done": false}

// Event: message
// Data: {"content": " an open-source framework...", "done": true}
```

### Quickstart Guide

**File**: `specs/002-chatbot-ui-streaming/quickstart.md`

**5-Minute Setup**:

1. **Prerequisites**: Docusaurus site running, backend API available at localhost:8000
2. **Install components**: Copy chat components to src/components/ChatInterface/
3. **Integrate into Docusaurus**: Add chat component to desired page/layout
4. **Configure backend**: Set API endpoint in chatService.js
5. **Test functionality**: Open page and verify streaming responses work

**Configuration**:
- BACKEND_URL: API endpoint for chat (default: http://localhost:8000)
- STREAMING_ENDPOINT: Streaming API path (default: /api/chat)
- MESSAGE_LIMIT: Max messages to display (default: 100, clears oldest when exceeded)

## Phase 2: Task Breakdown

**Note**: Detailed tasks will be generated in `/sp.tasks` command.

### Phase 2.1: Core UI Implementation
- Create ChatContainer with state management
- Implement MessageList with auto-scroll behavior
- Create MessageBubble components with role-based styling
- Add InputArea with validation and keyboard shortcuts

### Phase 2.2: Streaming Integration
- Implement SSE connection handling
- Create streaming response processor
- Add incremental message updates
- Implement connection error handling

### Phase 2.3: Styling and Integration
- Apply Docusaurus-compatible styling
- Ensure responsive design
- Integrate with existing navigation
- Add accessibility features

### Phase 2.4: Validation and Testing
- Test streaming functionality with backend
- Validate auto-scroll behavior
- Test error handling scenarios
- Verify Docusaurus integration

## Testing Strategy

### Validation Approach

**No automated tests required** per user request. Use manual validation with direct UI interaction.

**Validation Checklist** (`specs/002-chatbot-ui-streaming/validation-checklist.md`):

1. **Basic Functionality**:
   - [ ] User message appears immediately in chat bubble
   - [ ] Typing indicator shows during response generation
   - [ ] Streaming response displays incrementally
   - [ ] Input field clears after successful send

2. **Streaming Behavior**:
   - [ ] Text appears word-by-word during streaming
   - [ ] Auto-scroll tracks streaming content when at bottom
   - [ ] Manual scroll up pauses auto-scroll correctly
   - [ ] Auto-scroll resumes when returning to bottom

3. **Error Handling**:
   - [ ] Network errors display user-friendly messages
   - [ ] Partial responses remain visible after errors
   - [ ] Retry functionality works after errors
   - [ ] Empty input validation prevents submission

4. **UI/UX**:
   - [ ] Styling matches Docusaurus theme
   - [ ] Chat bubbles clearly distinguish user/assistant
   - [ ] Typing indicator animates smoothly
   - [ ] Input field responds within 50ms

5. **Performance**:
   - [ ] UI remains responsive during long streams
   - [ ] No flicker or layout shifts during streaming
   - [ ] DOM updates are efficient
   - [ ] Memory usage remains stable

### Sample Tests for Validation

```bash
# Basic functionality
1. Open chat interface
2. Type "What is ROS2?" and send
3. Verify user message appears immediately
4. Verify typing indicator shows
5. Verify response streams incrementally

# Auto-scroll behavior
1. Send multiple messages to create scrollable content
2. Scroll up while response is streaming
3. Verify auto-scroll pauses
4. Scroll to bottom
5. Verify auto-scroll resumes

# Error handling
1. Disconnect backend temporarily
2. Send message and verify error appears
3. Reconnect backend
4. Send message and verify recovery works
```

## Dependencies

- **Spec 001-agent-rag-backend**: REQUIRED - Backend streaming API at `/api/chat/stream`
- **Docusaurus**: REQUIRED - Existing documentation site for styling integration
- **Modern Web Browser**: REQUIRED - SSE and Fetch API support
- **Node.js/npm**: REQUIRED - For development server and build tools

## Implementation Notes

### Key Technical Decisions

1. **Streaming Protocol**: Server-Sent Events for reliable one-way streaming
2. **State Management**: React hooks for component-level state management
3. **Scroll Behavior**: Smart auto-scroll with manual override detection
4. **Error Handling**: Graceful degradation with user-friendly messages
5. **Performance**: Efficient DOM updates to maintain 30+ FPS during streaming
6. **Styling**: CSS variables to match Docusaurus theme consistency

### Performance Expectations

- Input response time: <50ms (instant feel)
- Streaming rendering: 30+ FPS (smooth experience)
- UI load time: <1 second (fast startup)
- Memory usage: Stable during long conversations

### Configuration via Environment

Reuse Docusaurus configuration with additions:
```javascript
// In chatService.js
const BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
const STREAMING_ENDPOINT = process.env.STREAMING_ENDPOINT || '/api/chat/stream';
const MESSAGE_LIMIT = process.env.MESSAGE_LIMIT || 100; // Max messages before cleanup
```

## Next Steps

1. Run `/sp.tasks` to generate detailed task list
2. Implement ChatContainer component with state management
3. Create streaming handler with SSE connection
4. Build message components with Docusaurus styling
5. Execute validation checklist
6. Document integration with Docusaurus site
7. Ready for integration with frontend (future spec)
