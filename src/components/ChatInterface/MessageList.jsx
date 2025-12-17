/**
 * Message display component that shows ordered list of chat messages with proper scrolling behavior
 */

/**
 * Create MessageList component
 * @param {Array} messages - Array of message objects to display
 * @param {Object} scrollPosition - Current scroll position state
 * @param {Function} onScroll - Callback when scroll position changes
 * @returns {HTMLElement} Message list element
 */
export function MessageList(messages = [], scrollPosition = null, onScroll = null) {
  const messageList = document.createElement('div');
  messageList.className = 'message-list';

  // Set up scroll handling if needed
  if (onScroll) {
    messageList.addEventListener('scroll', (e) => {
      const currentScrollTop = messageList.scrollTop;
      const isAtBottom = isScrolledToBottom(messageList);

      onScroll({
        isAtBottom,
        position: currentScrollTop
      });
    });
  }

  // Add messages to the list
  messages.forEach(message => {
    const messageElement = createMessageBubble(message);
    messageList.appendChild(messageElement);
  });

  // Add a method to update messages
  messageList.updateMessages = (newMessages) => {
    // Clear existing messages
    while (messageList.firstChild) {
      messageList.removeChild(messageList.firstChild);
    }

    // Add new messages
    newMessages.forEach(message => {
      const messageElement = createMessageBubble(message);
      messageList.appendChild(messageElement);
    });

    // If auto-scroll is enabled or we're at bottom, scroll to bottom
    if (!scrollPosition || scrollPosition.isAtBottom) {
      messageList.scrollTop = messageList.scrollHeight;
    }
  };

  // Add a method to add a single message
  messageList.addMessage = (message) => {
    const messageElement = createMessageBubble(message);
    messageList.appendChild(messageElement);

    // Auto-scroll to bottom if at bottom or auto-scroll enabled
    if (!scrollPosition || scrollPosition.isAtBottom) {
      messageList.scrollTop = messageList.scrollHeight;
    }
  };

  // Add a method to update a specific message (useful for streaming)
  messageList.updateMessage = (messageId, newContent) => {
    const messageElement = messageList.querySelector(`[data-message-id="${messageId}"]`);
    if (messageElement) {
      const contentElement = messageElement.querySelector('.message-content');
      if (contentElement) {
        contentElement.innerHTML = escapeHtml(newContent);
      }
    }
  };

  return messageList;
}

/**
 * Create a single message bubble element
 * @param {Object} message - Message object to render
 * @returns {HTMLElement} Message bubble element
 */
function createMessageBubble(message) {
  const bubble = document.createElement('div');
  bubble.className = `message-bubble message-bubble--${message.role} message-bubble--${message.status}`;
  bubble.setAttribute('data-message-id', message.id);

  // Format timestamp
  const timestamp = new Date(message.timestamp).toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit'
  });

  // Create the bubble content
  bubble.innerHTML = `
    <div class="message-content">${escapeHtml(message.content)}</div>
    <div class="message-timestamp">${timestamp}</div>
  `;

  return bubble;
}

/**
 * Check if the container is scrolled to the bottom
 * @param {HTMLElement} container - The container element to check
 * @returns {boolean} Whether the container is at the bottom
 */
function isScrolledToBottom(container) {
  // Calculate if we're within 5px of the bottom (tolerance for rounding)
  const tolerance = 5;
  const bottomPosition = container.scrollHeight - container.clientHeight;
  const currentPosition = container.scrollTop;

  return Math.abs(bottomPosition - currentPosition) <= tolerance;
}

/**
 * Escape HTML to prevent XSS
 * @param {string} text - Text to escape
 * @returns {string} Escaped HTML
 */
function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

// Export for use in other modules
export default MessageList;