/**
 * Individual message display component with role-based styling
 */

/**
 * Create a single message bubble element
 * @param {Object} message - Message object to render
 * @param {Object} [options] - Additional options for rendering
 * @param {boolean} [options.showTimestamp=true] - Whether to show timestamp
 * @param {Function} [options.onContentUpdate] - Callback when content is updated
 * @returns {HTMLElement} Message bubble element
 */
export function MessageBubble(message, options = {}) {
  const { showTimestamp = true, onContentUpdate } = options;

  if (!message || typeof message !== 'object') {
    throw new Error('Message object is required');
  }

  if (!message.id || !message.role || message.content === undefined) {
    throw new Error('Message must have id, role, and content properties');
  }

  const bubble = document.createElement('div');
  bubble.className = `message-bubble message-bubble--${message.role} message-bubble--${message.status}`;
  bubble.setAttribute('data-message-id', message.id);

  // Format timestamp if needed
  let timestampHtml = '';
  if (showTimestamp && message.timestamp) {
    const timestamp = new Date(message.timestamp).toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit'
    });
    timestampHtml = `<div class="message-timestamp">${timestamp}</div>`;
  }

  // Create the bubble content
  bubble.innerHTML = `
    <div class="message-content">${escapeHtml(message.content)}</div>
    ${timestampHtml}
  `;

  // Add error styling if message has error status
  if (message.status === 'error') {
    bubble.classList.add('message-bubble--error');
  }

  // Add streaming styling if message is streaming
  if (message.status === 'streaming') {
    bubble.classList.add('message-bubble--streaming');
  }

  // Add a method to update content
  bubble.updateContent = (newContent) => {
    const contentElement = bubble.querySelector('.message-content');
    if (contentElement) {
      contentElement.innerHTML = escapeHtml(newContent);

      // Call update callback if provided
      if (onContentUpdate) {
        onContentUpdate(newContent);
      }
    }
  };

  // Add a method to update status
  bubble.updateStatus = (newStatus) => {
    // Remove old status classes
    bubble.classList.remove(
      'message-bubble--sending',
      'message-bubble--streaming',
      'message-bubble--complete',
      'message-bubble--error'
    );

    // Add new status class
    bubble.classList.add(`message-bubble--${newStatus}`);

    // Add error class if needed
    if (newStatus === 'error') {
      bubble.classList.add('message-bubble--error');
    }
  };

  return bubble;
}

/**
 * Create a message bubble element from scratch with role-based styling
 * @param {string} role - Role of the message ('user' or 'assistant')
 * @param {string} content - Content of the message
 * @param {string} status - Status of the message ('sending', 'streaming', 'complete', 'error')
 * @param {string} [id] - Optional ID for the message
 * @param {string} [timestamp] - Optional timestamp
 * @returns {HTMLElement} Message bubble element
 */
export function createMessageBubble(role, content, status, id, timestamp) {
  const message = {
    id: id || Date.now().toString(36) + Math.random().toString(36).substr(2),
    role,
    content,
    status,
    timestamp: timestamp || new Date().toISOString()
  };

  return MessageBubble(message);
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
export default MessageBubble;