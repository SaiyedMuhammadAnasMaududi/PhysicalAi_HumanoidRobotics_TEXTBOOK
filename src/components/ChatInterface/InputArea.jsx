/**
 * User input field with send functionality
 */

/**
 * Create InputArea component
 * @param {Function} onSendMessage - Callback when user sends a message
 * @param {Object} [options] - Additional options for the input area
 * @param {boolean} [options.disabled=false] - Whether the input is disabled
 * @param {string} [options.placeholder='Type your message here...'] - Placeholder text
 * @param {boolean} [options.multiline=false] - Whether to allow multiline input
 * @returns {HTMLElement} Input area element
 */
export function InputArea(onSendMessage, options = {}) {
  const {
    disabled = false,
    placeholder = 'Type your message here...',
    multiline = false
  } = options;

  if (typeof onSendMessage !== 'function') {
    throw new Error('onSendMessage callback function is required');
  }

  const inputArea = document.createElement('div');
  inputArea.className = 'input-area';

  // Create the input element based on multiline option
  let inputElement;
  if (multiline) {
    inputElement = document.createElement('textarea');
    inputElement.className = 'message-input message-input--multiline';
    inputElement.rows = 1; // Start with 1 row, will expand as needed
  } else {
    inputElement = document.createElement('input');
    inputElement.type = 'text';
    inputElement.className = 'message-input';
  }

  inputElement.placeholder = placeholder;
  inputElement.disabled = disabled;

  // Create the send button
  const sendButton = document.createElement('button');
  sendButton.className = 'send-button';
  sendButton.textContent = 'Send';
  sendButton.disabled = disabled;

  // Add accessibility attributes
  sendButton.setAttribute('aria-label', 'Send message');
  inputElement.setAttribute('aria-label', 'Message input');
  inputElement.setAttribute('aria-describedby', 'chat-input-help');

  // Add helper text for screen readers
  const helpText = document.createElement('div');
  helpText.id = 'chat-input-help';
  helpText.className = 'sr-only';
  helpText.textContent = 'Enter your message and press Enter or click Send to submit';
  helpText.setAttribute('aria-live', 'polite');

  inputArea.appendChild(helpText);

  // Add elements to the container
  inputArea.appendChild(inputElement);
  inputArea.appendChild(sendButton);

  // Handle send button click
  sendButton.addEventListener('click', () => {
    if (disabled) return;

    const query = getInputValue(inputElement);
    if (query.trim()) {
      onSendMessage(query);
      setInputValue(inputElement, '');
    }
  });

  // Handle Enter key press
  inputElement.addEventListener('keydown', (e) => {
    if (disabled) return;

    if (e.key === 'Enter') {
      if (multiline && e.shiftKey) {
        // Allow new line when Shift+Enter in multiline mode
        return;
      }

      e.preventDefault(); // Prevent default Enter behavior
      const query = getInputValue(inputElement);
      if (query.trim()) {
        onSendMessage(query);
        setInputValue(inputElement, '');
      }
    }
  });

  // Handle input changes for validation
  inputElement.addEventListener('input', () => {
    // Enable/disable send button based on input
    const hasContent = getInputValue(inputElement).trim().length > 0;
    sendButton.disabled = disabled || !hasContent;
  });

  // Add method to update disabled state
  inputArea.setDisabled = (isDisabled) => {
    inputElement.disabled = isDisabled;
    sendButton.disabled = isDisabled;
  };

  // Add method to get current value
  inputArea.getValue = () => {
    return getInputValue(inputElement);
  };

  // Add method to set value
  inputArea.setValue = (value) => {
    setInputValue(inputElement, value);
  };

  // Add method to focus the input
  inputArea.focus = () => {
    inputElement.focus();
  };

  // Add method to validate input
  inputArea.validate = () => {
    const value = getInputValue(inputElement);
    return {
      isValid: value.trim().length > 0,
      value: value
    };
  };

  return inputArea;
}

/**
 * Get the value from an input element (works for both input and textarea)
 * @param {HTMLElement} inputElement - The input element
 * @returns {string} The input value
 */
function getInputValue(inputElement) {
  return inputElement.value || '';
}

/**
 * Set the value on an input element (works for both input and textarea)
 * @param {HTMLElement} inputElement - The input element
 * @param {string} value - The value to set
 */
function setInputValue(inputElement, value) {
  inputElement.value = value || '';

  // If it's a textarea, adjust rows based on content
  if (inputElement.tagName === 'TEXTAREA') {
    adjustTextareaRows(inputElement);
  }
}

/**
 * Adjust textarea rows based on content
 * @param {HTMLTextAreaElement} textarea - The textarea element
 */
function adjustTextareaRows(textarea) {
  textarea.style.height = 'auto';
  textarea.style.height = Math.min(textarea.scrollHeight, 150) + 'px'; // Max height of 150px
}

/**
 * Create an input area with validation
 * @param {Function} onSendMessage - Callback when user sends a message
 * @param {Object} [options] - Additional options
 * @returns {HTMLElement} Input area element with validation
 */
export function createValidatedInputArea(onSendMessage, options = {}) {
  const inputArea = InputArea(onSendMessage, options);

  // Add real-time validation
  const inputElement = inputArea.querySelector('.message-input');
  inputElement.addEventListener('input', () => {
    const value = inputElement.value;

    // Basic validation: check for minimum length
    if (value.length > 0 && value.trim().length === 0) {
      // Input contains only whitespace
      inputElement.setCustomValidity('Message cannot contain only whitespace');
    } else if (value.length > 1000) {
      // Input too long
      inputElement.setCustomValidity('Message exceeds maximum length of 1000 characters');
    } else {
      inputElement.setCustomValidity('');
    }
  });

  return inputArea;
}

// Export for use in other modules
export default InputArea;