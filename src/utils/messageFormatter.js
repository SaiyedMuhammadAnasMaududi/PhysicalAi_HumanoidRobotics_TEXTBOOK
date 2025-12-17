/**
 * Message data model interface
 * @typedef {Object} Message
 * @property {string} id - Unique identifier for the message
 * @property {'user'|'assistant'} role - Sender role (user or assistant)
 * @property {string} content - Message text content
 * @property {string} timestamp - When message was created (ISO string)
 * @property {'sending'|'streaming'|'complete'|'error'} status - Message state
 */

/**
 * StreamChunk data model interface
 * @typedef {Object} StreamChunk
 * @property {string} content - Text content of this chunk
 * @property {boolean} done - Indicates if stream is complete
 * @property {string|null} error - Error message if chunk contains error information
 * @property {string} timestamp - When chunk was received (ISO string)
 */

/**
 * ChatState data model interface
 * @typedef {Object} ChatState
 * @property {Message[]} messages - Complete message history in chronological order
 * @property {boolean} streamingStatus - Indicates if response is currently streaming
 * @property {'connected'|'connecting'|'disconnected'|'error'} connectionStatus - Current connection state
 * @property {Object|null} errorState - Current error information with message and timestamp
 * @property {Object} scrollPosition - Current scroll tracking with isAtBottom and position properties
 * @property {boolean} scrollPosition.isAtBottom - Whether user is at bottom of chat
 * @property {number} scrollPosition.position - Current scroll position
 */

/**
 * Format a new message object
 * @param {'user'|'assistant'} role - The role of the message sender
 * @param {string} content - The content of the message
 * @param {'sending'|'streaming'|'complete'|'error'} [status='complete'] - The status of the message
 * @returns {Message} Formatted message object
 */
export function formatMessage(role, content, status = 'complete') {
  return {
    id: generateId(),
    role,
    content,
    timestamp: new Date().toISOString(),
    status
  };
}

/**
 * Format a stream chunk object
 * @param {string} content - The content of this chunk
 * @param {boolean} done - Whether this is the final chunk
 * @param {string|null} error - Error message if applicable
 * @returns {StreamChunk} Formatted stream chunk object
 */
export function formatStreamChunk(content, done, error = null) {
  return {
    content,
    done,
    error,
    timestamp: new Date().toISOString()
  };
}

/**
 * Initialize default chat state
 * @returns {ChatState} Initial chat state
 */
export function initializeChatState() {
  return {
    messages: [],
    streamingStatus: false,
    connectionStatus: 'connected',
    errorState: null,
    scrollPosition: {
      isAtBottom: true,
      position: 0
    }
  };
}

/**
 * Generate a unique ID for messages
 * @returns {string} Unique identifier
 */
function generateId() {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
}

/**
 * Update message content during streaming
 * @param {string} messageId - ID of the message to update
 * @param {string} newContent - New content to append
 * @param {ChatState} state - Current chat state
 * @returns {ChatState} Updated chat state
 */
export function updateMessageContent(messageId, newContent, state) {
  const updatedMessages = state.messages.map(message => {
    if (message.id === messageId) {
      return {
        ...message,
        content: message.content + newContent
      };
    }
    return message;
  });

  return {
    ...state,
    messages: updatedMessages
  };
}

/**
 * Add a new message to the chat state with history limiting
 * @param {Message} message - Message to add
 * @param {ChatState} state - Current chat state
 * @param {number} [limit=100] - Maximum number of messages to keep
 * @returns {ChatState} Updated chat state
 */
export function addMessage(message, state, limit = 100) {
  let updatedMessages = [...state.messages, message];

  // Limit message history if needed
  if (updatedMessages.length > limit) {
    updatedMessages = updatedMessages.slice(-limit);
  }

  return {
    ...state,
    messages: updatedMessages
  };
}

/**
 * Update the streaming status
 * @param {boolean} status - New streaming status
 * @param {ChatState} state - Current chat state
 * @returns {ChatState} Updated chat state
 */
export function updateStreamingStatus(status, state) {
  return {
    ...state,
    streamingStatus: status
  };
}

/**
 * Set error state
 * @param {Object|null} error - Error object or null to clear
 * @param {ChatState} state - Current chat state
 * @returns {ChatState} Updated chat state
 */
export function setErrorState(error, state) {
  return {
    ...state,
    errorState: error,
    connectionStatus: error ? 'error' : 'connected'
  };
}