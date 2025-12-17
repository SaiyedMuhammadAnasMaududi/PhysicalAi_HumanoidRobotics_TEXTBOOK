# Research: Intermediate Chatbot UI with Streaming

**Feature**: 002-chatbot-ui-streaming
**Date**: 2025-12-17
**Input**: Implementation plan from `plan.md`

## Research Focus

This research addresses the key technical questions identified in the implementation plan for creating a streaming chatbot UI that integrates with the existing RAG backend and Docusaurus textbook site.

## Research Questions & Findings

### 1. Streaming Protocol Selection: SSE vs Chunked HTTP

**Question**: Should we use Server-Sent Events (SSE) or chunked HTTP for streaming responses?

**Research**:
- Server-Sent Events (SSE) provides native browser API via EventSource
- SSE offers automatic reconnection, built-in error handling, and simpler implementation
- Chunked HTTP requires manual parsing of transfer-encoding responses
- SSE has good browser support (Chrome 6+, Firefox 6+, Safari 5+, Edge 12+)
- Backend already supports SSE format per spec requirements

**Decision**: Server-Sent Events (SSE) for real-time streaming

**Rationale**: Provides better browser support, simpler implementation for one-way streaming, native browser support via EventSource API, and automatic reconnection handling.

**Implementation Pattern**:
```javascript
// Using EventSource for SSE connection
const eventSource = new EventSource('/api/chat/stream', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({ query: userQuery })
});

eventSource.onmessage = (event) => {
  const data = JSON.parse(event.data);
  updateMessageContent(data.content, data.done);
};

eventSource.onerror = (error) => {
  handleStreamingError(error);
};
```

**Reference**: Browser compatibility and backend API contract

### 2. Message State Management

**Question**: What's the optimal approach for managing incremental message updates during streaming?

**Research**:
- Virtual DOM updates with React/Preact for efficient rendering
- Incremental DOM updates to avoid full re-renders
- Text node manipulation for streaming content
- Performance considerations for large conversation histories

**Decision**: Use efficient DOM updates with React hooks for state management

**Rationale**: Appropriate for single-session chat, provides good performance, familiar to developers, sufficient for spec requirements.

**Implementation Pattern**:
```javascript
// Using React state for message management
const [messages, setMessages] = useState([]);
const [currentStreamingMessage, setCurrentStreamingMessage] = useState('');

const updateMessageContent = (content, done) => {
  if (done) {
    // Complete the streaming message
    setMessages(prev => [...prev.slice(0, -1), {
      ...prev[prev.length - 1],
      content: currentStreamingMessage + content,
      status: 'complete'
    }]);
    setCurrentStreamingMessage('');
  } else {
    // Update the streaming content incrementally
    setCurrentStreamingMessage(prev => prev + content);
  }
};
```

**Reference**: Performance optimization patterns

### 3. Auto-Scroll Behavior

**Question**: How to implement smart auto-scroll that respects user scrolling intent?

**Research**:
- Detect when user is at bottom of chat container
- Track scroll position to determine if auto-scroll should be active
- Pause auto-scroll when user manually scrolls up
- Resume auto-scroll when user returns to bottom
- Smooth scrolling behavior for better UX

**Decision**: Track scroll position and only auto-scroll when user is already at bottom

**Rationale**: Provides intuitive behavior that respects user navigation choices while maintaining good default experience.

**Implementation Pattern**:
```javascript
// Auto-scroll management
const shouldAutoScroll = useRef(true);
const chatContainerRef = useRef(null);

const handleScroll = () => {
  const container = chatContainerRef.current;
  const atBottom = container.scrollHeight - container.scrollTop === container.clientHeight;

  shouldAutoScroll.current = atBottom;
};

const scrollToBottom = () => {
  if (shouldAutoScroll.current) {
    chatContainerRef.current.scrollTop = chatContainerRef.current.scrollHeight;
  }
};
```

**Reference**: Chat UI best practices

### 4. Docusaurus Styling Integration

**Question**: How to integrate with existing Docusaurus theme without breaking existing styles?

**Research**:
- Docusaurus uses CSS variables for theming
- Custom CSS can be injected via theme configuration
- React components can use CSS Modules or styled-components
- Class names should follow Docusaurus conventions

**Decision**: Use CSS variables and class names that align with Docusaurus theme

**Rationale**: Maintains theme consistency, allows customization, follows Docusaurus best practices, and ensures seamless integration.

**Implementation Pattern**:
```css
/* Using Docusaurus CSS variables */
.chat-container {
  background-color: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-global-radius);
}

.message-bubble.user {
  background-color: var(--ifm-color-primary);
  color: var(--ifm-button-color);
}

.message-bubble.assistant {
  background-color: var(--ifm-color-emphasis-100);
  color: var(--ifm-font-color-base);
}
```

**Reference**: Docusaurus theme customization documentation

## Architecture Pattern: Streaming Chat UI

Based on research, the optimal architecture combines:

1. **Server-Sent Events**: For reliable one-way streaming from backend
2. **React Hooks**: For component state management and lifecycle
3. **CSS Variables**: For Docusaurus theme integration
4. **Smart Scrolling**: For intuitive user experience
5. **Error Boundaries**: For graceful error handling

This pattern ensures good performance, maintainability, and seamless integration with the existing textbook site.

## Integration Points

### With Spec 001 RAG Backend
- Connect to streaming endpoint: `/api/chat/stream`
- Handle SSE format responses from backend
- Process response chunks and metadata (sources, timing)

### With Docusaurus Site
- Use CSS variables from Docusaurus theme
- Integrate as React component in Docusaurus pages
- Follow navigation and layout patterns

## Risk Assessment

### Potential Issues
1. **Browser Compatibility**: Older browsers may not support EventSource
2. **Performance**: Large conversations may impact rendering performance
3. **Connection Stability**: Network interruptions during streaming
4. **Memory Usage**: Long-running conversations may accumulate state

### Mitigation Strategies
1. Provide fallback for non-SSE browsers
2. Implement message history limits and cleanup
3. Add connection retry logic with exponential backoff
4. Use efficient state updates and cleanup patterns

## Conclusion

The research confirms that the planned architecture is feasible and follows industry best practices. The combination of SSE for streaming, React for UI management, and CSS variables for theming provides the best integration with the existing system while maintaining performance and user experience goals.