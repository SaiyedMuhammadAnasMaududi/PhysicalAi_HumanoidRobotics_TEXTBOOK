# Quickstart Guide: Intermediate Chatbot UI with Streaming

**Feature**: 002-chatbot-ui-streaming
**Date**: 2025-12-17
**Input**: Implementation plan and API contracts

## Overview

This guide provides step-by-step instructions to quickly set up and integrate the streaming chatbot UI with the existing Docusaurus-based Physical AI & Humanoid Robotics textbook site. The system provides real-time streaming responses from the RAG backend through an interactive chat interface.

## Prerequisites

Before starting, ensure you have:

- Node.js 14+ and npm/yarn installed
- Docusaurus site running locally (from existing textbook setup)
- Backend API from Spec 001 running at `http://localhost:8000`
- Modern web browser (Chrome 60+, Firefox 60+, Safari 12+, Edge 79+)

## Setup Instructions

### 1. Clone or Navigate to Repository

```bash
cd /path/to/PhysicalAi_HumanoidRobotics_TEXTBOOK
```

### 2. Install Chat Components

Create the component structure in your Docusaurus project:

```bash
mkdir -p src/components/ChatInterface
```

### 3. Verify Backend Availability

Before integrating the UI, verify that the RAG backend is running:

```bash
curl http://localhost:8000/health
```

You should see a healthy response from the backend.

## Integration with Docusaurus

### 1. Component Installation

Copy the chat components to the appropriate location:

```text
src/
└── components/
    └── ChatInterface/
        ├── ChatContainer.jsx          # Main chat container component
        ├── MessageList.jsx            # Message display component
        ├── MessageBubble.jsx          # Individual message bubble component
        ├── InputArea.jsx              # Input field and send button
        ├── TypingIndicator.jsx        # Animated typing indicator
        └── ChatStyles.css             # Docusaurus-compatible styling
```

### 2. Basic Integration

Add the chat component to a Docusaurus page or layout:

```jsx
// Example: Adding to a page
import ChatContainer from '@site/src/components/ChatInterface/ChatContainer';

function MyPage() {
  return (
    <div>
      <h1>Physical AI & Humanoid Robotics</h1>
      <ChatContainer />
    </div>
  );
}
```

### 3. Configuration

Set up the backend configuration in your chat service:

```javascript
// In chatService.js
const CHAT_CONFIG = {
  BACKEND_URL: process.env.BACKEND_URL || 'http://localhost:8000',
  STREAMING_ENDPOINT: process.env.STREAMING_ENDPOINT || '/api/chat/stream',
  MESSAGE_LIMIT: process.env.MESSAGE_LIMIT || 100, // Max messages before cleanup
  AUTO_SCROLL_ENABLED: process.env.AUTO_SCROLL_ENABLED !== 'false'
};
```

## Using the Chat Interface

### 1. Basic Chat Functionality

Once integrated, the chat interface will appear on your Docusaurus page:

```
┌─────────────────────────────────┐
│  Physical AI & Humanoid Robotics│
├─────────────────────────────────┤
│                                 │
│  [User Message]                 │
│  Hello!                         │
│                                 │
│  [Assistant Message]            │
│  Hi there! How can I help you   │
│  with Physical AI and Humanoid  │
│  Robotics today?                │
│                                 │
│  [Typing indicators...]         │
│                                 │
│  ┌─────────────────────────────┐ │
│  │ What is ROS2?         [↑]   │ │
│  └─────────────────────────────┘ │
└─────────────────────────────────┘
```

### 2. Streaming Response Demonstration

Send a query to test streaming functionality:

```bash
1. Type "What is ROS2?" in the input field
2. Press Enter or click Send
3. Observe the user message appears immediately
4. See the typing indicator appear
5. Watch the response stream word-by-word in real-time
6. Verify the response completes and input field clears
```

### 3. Auto-Scroll Behavior

Test the smart scrolling feature:

```bash
1. Send multiple messages to create a scrollable chat history
2. Scroll up to read earlier messages
3. Notice auto-scroll pauses when you scroll up
4. Send a new message while scrolled up
5. Scroll back to bottom to see auto-scroll resume
```

## Configuration Options

### Environment Variables

Configure the chat interface behavior using environment variables:

```bash
# Backend configuration
BACKEND_URL=http://localhost:8000
STREAMING_ENDPOINT=/api/chat/stream

# UI behavior
MESSAGE_LIMIT=100
AUTO_SCROLL_ENABLED=true
INPUT_DEBOUNCE_MS=300
```

### Styling Integration

The chat interface uses Docusaurus CSS variables for seamless theme integration:

```css
/* Example of Docusaurus-compatible styling */
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

## Testing Different Queries

### Technical Queries
```text
User: "How does perception connect to control in robotics?"
```

### Book-Specific Queries
```text
User: "Explain the architecture of ROS2"
```

### General Robotics Queries
```text
User: "What are the key components of a humanoid robot?"
```

## Troubleshooting

### Common Issues

#### 1. Streaming Not Working
- Verify backend is running at configured URL
- Check browser console for CORS errors
- Confirm backend supports SSE format

#### 2. Styling Issues
- Verify Docusaurus CSS variables are loaded
- Check that chat stylesheets are properly imported
- Ensure component is placed within Docusaurus layout

#### 3. Connection Errors
- Check network connectivity to backend
- Verify backend streaming endpoint exists
- Confirm firewall/VPN isn't blocking the connection

### Error Response Examples

#### Network Error Response
```json
{
  "error": "Unable to connect to server. Please check your connection and try again.",
  "timestamp": "2025-12-17T02:05:52.449099"
}
```

#### Backend Error Response
```json
{
  "error": "Backend service temporarily unavailable",
  "retry_after": 30,
  "timestamp": "2025-12-17T02:05:52.449099"
}
```

## Performance Testing

### Load Testing
```bash
# Test multiple rapid queries
for i in {1..5}; do
  echo "Query $i: What is robotics concept $i?"
  sleep 2
done
```

### Response Time Monitoring
- First token latency: <2 seconds (backend dependent)
- Input response: <50ms
- Streaming smoothness: 30+ FPS

## Next Steps

1. **Customization**: Modify styling to match specific Docusaurus theme
2. **Advanced Features**: Add message history persistence (beyond session)
3. **Analytics**: Integrate usage tracking for chat interactions
4. **Documentation**: Generate API documentation for chat interface
5. **Deployment**: Configure for production deployment with your Docusaurus site

## API Documentation

Once the Docusaurus site is running, the chat interface will be available as part of your textbook site. The streaming functionality works automatically with the backend RAG system, providing real-time responses synchronized with the Physical AI & Humanoid Robotics content.

The chat interface is now ready for use and fully integrated with your Docusaurus textbook site!