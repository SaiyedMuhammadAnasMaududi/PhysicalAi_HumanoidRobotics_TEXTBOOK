/**
 * Main chat container component that manages overall state and layout
 */

import { initializeChatState, addMessage, updateStreamingStatus, setErrorState } from '../../utils/messageFormatter.js';
import { formatMessage } from '../../utils/messageFormatter.js';
import { connectToStreamingEndpoint } from '../../services/streamingHandler.js';
import { initializeScrollManager } from '../../services/scrollManager.js';
import './ChatStyles.css'; // Import chat styles

/**
 * ChatContainer component
 * @param {Object} props - Component properties
 * @param {string} [props.backendUrl] - Backend URL to override default
 * @param {string} [props.streamingEndpoint] - Streaming endpoint to override default
 * @param {number} [props.messageLimit] - Maximum number of messages to display
 * @param {Function} [props.onStateChange] - Callback when chat state changes
 * @returns {HTMLElement} Chat container element
 */
export function ChatContainer({
  backendUrl,
  streamingEndpoint,
  messageLimit = 100,
  onStateChange
} = {}) {
  // Initialize state
  let state = initializeChatState();
  let currentStreamConnection = null;
  let scrollManager = null;

  // Create the main container element
  const container = document.createElement('div');
  container.className = 'chat-container';
  container.setAttribute('role', 'main');
  container.setAttribute('aria-label', 'Chat interface');
  container.innerHTML = `
    <div class="chat-messages-container" aria-live="polite" aria-atomic="false">
      <div class="chat-messages" id="chat-messages" role="log" aria-label="Chat messages"></div>
    </div>
    <div class="chat-input-container">
      <div class="chat-input-area" id="chat-input-area" role="form" aria-label="Message input area"></div>
    </div>
    <div class="chat-typing-indicator" id="chat-typing-indicator" style="display: none;" aria-label="Assistant is typing" aria-hidden="true"></div>
  `;

  // Get references to child elements
  const messagesContainer = container.querySelector('#chat-messages');
  const inputAreaContainer = container.querySelector('#chat-input-area');
  const typingIndicatorContainer = container.querySelector('#chat-typing-indicator');

  // Initialize scroll manager
  scrollManager = initializeScrollManager(messagesContainer, (scrollState) => {
    // Update state when scroll changes
    state.scrollPosition = scrollState;
  });

  // Create and add child components
  const messageList = createMessageList(state.messages);
  messagesContainer.appendChild(messageList);

  const inputArea = createInputArea(handleSendMessage);
  inputAreaContainer.appendChild(inputArea);

  // Initially disable input if not connected
  updateInputState(state.streamingStatus);

  const typingIndicator = createTypingIndicator();
  typingIndicatorContainer.appendChild(typingIndicator);

  // Update state and notify when needed
  function updateState(newState) {
    state = { ...state, ...newState };

    // Update DOM based on state changes
    updateDOMFromState();

    // Notify parent if callback provided
    if (onStateChange) {
      onStateChange(state);
    }
  }

  // Update DOM elements based on current state
  function updateDOMFromState() {
    // Update message list
    const existingMessageList = messagesContainer.querySelector('.message-list');
    if (existingMessageList) {
      existingMessageList.remove();
    }

    const newMessageList = createMessageList(state.messages);
    messagesContainer.appendChild(newMessageList);

    // Update typing indicator visibility
    typingIndicatorContainer.style.display = state.streamingStatus ? 'block' : 'none';
    typingIndicatorContainer.setAttribute('aria-hidden', !state.streamingStatus);

    // Update input area based on streaming status
    updateInputState(state.streamingStatus);

    // Update scroll position
    if (state.streamingStatus || state.messages.length > (state.messages.length - 10)) {
      // Scroll to bottom when new messages arrive or during streaming
      scrollManager.updateScrollPosition();
    }
  }

  // Update input area state (enabled/disabled)
  function updateInputState(isStreaming) {
    const inputElement = inputAreaContainer.querySelector('input');
    if (inputElement) {
      inputElement.disabled = isStreaming;
    }

    const sendButton = inputAreaContainer.querySelector('button');
    if (sendButton) {
      sendButton.disabled = isStreaming;
    }
  }

  // Handle sending a message
  async function handleSendMessage(query) {
    // Validate query
    if (!query || query.trim().length === 0) {
      return;
    }

    // Create and add user message immediately
    const userMessage = formatMessage('user', query, 'complete');
    const updatedState = addMessage(userMessage, state, messageLimit);
    updateState(updatedState);

    // Update state to show we're streaming
    updateState({
      ...state,
      streamingStatus: true,
      connectionStatus: 'connected'
    });

    // Connect to streaming endpoint
    currentStreamConnection = connectToStreamingEndpoint(
      query,
      handleStreamUpdate,  // Update message during streaming
      handleStatusChange,  // Handle status changes
      handleError        // Handle errors
    );
  }

  // Handle incremental updates during streaming
  function handleStreamUpdate(chunk) {
    if (chunk.done) {
      // Complete the assistant message
      const lastMessage = state.messages[state.messages.length - 1];
      if (lastMessage && lastMessage.role === 'assistant' && lastMessage.status === 'streaming') {
        // Update the existing streaming message
        const updatedMessages = [...state.messages];
        updatedMessages[updatedMessages.length - 1] = {
          ...lastMessage,
          content: lastMessage.content + chunk.content,
          status: 'complete'
        };

        updateState({
          ...state,
          messages: updatedMessages,
          streamingStatus: false
        });
      } else {
        // If no streaming message exists, create a new one
        const assistantMessage = formatMessage('assistant', chunk.content, 'complete');
        const updatedState = addMessage(assistantMessage, state, messageLimit);
        updateState({
          ...updatedState,
          streamingStatus: false
        });
      }
    } else {
      // Update existing streaming message or create a new one
      const lastMessage = state.messages[state.messages.length - 1];
      if (lastMessage && lastMessage.role === 'assistant' && lastMessage.status === 'streaming') {
        // Update the existing streaming message
        const updatedMessages = [...state.messages];
        updatedMessages[updatedMessages.length - 1] = {
          ...lastMessage,
          content: lastMessage.content + chunk.content
        };

        updateState({
          ...state,
          messages: updatedMessages
        });
      } else {
        // Create a new streaming message
        const assistantMessage = formatMessage('assistant', chunk.content, 'streaming');
        const updatedState = addMessage(assistantMessage, state, messageLimit);
        updateState(updatedState);
      }
    }
  }

  // Handle status changes from the streaming service
  function handleStatusChange(status) {
    if (status === 'complete') {
      updateState({
        ...state,
        streamingStatus: false
      });
    } else {
      updateState({
        ...state,
        connectionStatus: status
      });
    }
  }

  // Handle errors from the streaming service
  function handleError(error) {
    const errorMessage = {
      id: Date.now().toString(),
      role: 'assistant',
      content: error.message || 'An error occurred while processing your request',
      timestamp: new Date().toISOString(),
      status: 'error'
    };

    const updatedState = setErrorState(error, addMessage(errorMessage, {
      ...state,
      streamingStatus: false,
      connectionStatus: 'error'
    }, messageLimit));

    updateState(updatedState);
  }

  // Cleanup function
  container.cleanup = function() {
    if (currentStreamConnection && typeof currentStreamConnection.close === 'function') {
      currentStreamConnection.close();
    }

    if (scrollManager && typeof scrollManager.destroy === 'function') {
      scrollManager.destroy();
    }
  };

  // Return the container element
  return container;
}

// Helper function to create message list component
function createMessageList(messages) {
  const messageList = document.createElement('div');
  messageList.className = 'message-list';

  messages.forEach(message => {
    const messageElement = createMessageBubble(message);
    messageList.appendChild(messageElement);
  });

  return messageList;
}

// Helper function to create message bubble component
function createMessageBubble(message) {
  const bubble = document.createElement('div');
  bubble.className = `message-bubble message-bubble--${message.role} message-bubble--${message.status}`;
  bubble.setAttribute('data-message-id', message.id);

  // Add accessibility attributes
  const roleLabel = message.role === 'user' ? 'User message' : 'Assistant message';
  bubble.setAttribute('role', 'logitem');
  bubble.setAttribute('aria-label', `${roleLabel}: ${message.content.substring(0, 50)}${message.content.length > 50 ? '...' : ''}`);

  // Format timestamp
  const timestamp = new Date(message.timestamp).toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit'
  });

  bubble.innerHTML = `
    <div class="message-content" aria-label="${roleLabel} content">${escapeHtml(message.content)}</div>
    <div class="message-timestamp" aria-label="Message timestamp">${timestamp}</div>
  `;

  return bubble;
}

// Helper function to create input area component
function createInputArea(onSendMessage) {
  const inputArea = document.createElement('div');
  inputArea.className = 'input-area';

  inputArea.innerHTML = `
    <input
      type="text"
      class="message-input"
      placeholder="Type your message here..."
      id="message-input"
    />
    <button class="send-button" id="send-button">Send</button>
  `;

  const inputElement = inputArea.querySelector('#message-input');
  const sendButton = inputArea.querySelector('#send-button');

  // Handle send button click
  sendButton.addEventListener('click', () => {
    const query = inputElement.value.trim();
    if (query) {
      onSendMessage(query);
      inputElement.value = ''; // Clear input after sending
    }
  });

  // Handle Enter key press
  inputElement.addEventListener('keypress', (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      const query = inputElement.value.trim();
      if (query) {
        onSendMessage(query);
        inputElement.value = ''; // Clear input after sending
      }
    }
  });

  return inputArea;
}

// Helper function to create typing indicator
function createTypingIndicator() {
  const indicator = document.createElement('div');
  indicator.className = 'typing-indicator';
  indicator.innerHTML = `
    <div class="typing-dot"></div>
    <div class="typing-dot"></div>
    <div class="typing-dot"></div>
  `;

  return indicator;
}

// Helper function to escape HTML to prevent XSS
function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

// Export for use in other modules
export default ChatContainer;