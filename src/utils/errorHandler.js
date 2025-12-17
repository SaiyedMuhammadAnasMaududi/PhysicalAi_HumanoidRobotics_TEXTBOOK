/**
 * Error handling utilities for the chat interface
 */

// Backend-specific error types
export const ErrorTypes = {
  VALIDATION_ERROR: 'VALIDATION_ERROR',
  BACKEND_ERROR: 'BACKEND_ERROR',
  SERVICE_UNAVAILABLE: 'SERVICE_UNAVAILABLE',
  TIMEOUT: 'TIMEOUT',
  NETWORK_ERROR: 'NETWORK_ERROR',
  CORS_ERROR: 'CORS_ERROR',
  MALFORMED_RESPONSE: 'MALFORMED_RESPONSE',
  STREAMING_ERROR: 'STREAMING_ERROR',
  UNKNOWN: 'UNKNOWN'
};

/**
 * Format a user-friendly error message based on error type
 * @param {Error|string|Object} error - The error to format
 * @param {string} [context=''] - Context where the error occurred
 * @returns {Object} Formatted error object with user-friendly message
 */
export function formatError(error, context = '') {
  let errorMessage = 'An unknown error occurred';
  let errorType = ErrorTypes.UNKNOWN;
  let userFriendlyMessage = 'An error occurred while processing your request. Please try again.';

  if (typeof error === 'string') {
    errorMessage = error;
  } else if (error instanceof Error) {
    errorMessage = error.message;
    errorType = error.name;
  } else if (error && typeof error === 'object') {
    if (error.error) {
      errorMessage = error.error;
    } else if (error.message) {
      errorMessage = error.message;
    } else {
      errorMessage = JSON.stringify(error);
    }
  }

  // Format error based on type
  if (errorMessage.includes('NetworkError') || errorMessage.includes('Failed to fetch') || errorMessage.includes('network') || errorMessage.includes('ECONNREFUSED')) {
    userFriendlyMessage = 'Unable to connect to backend. Please check your connection and make sure the backend is running.';
    errorType = ErrorTypes.NETWORK_ERROR;
  } else if (errorMessage.includes('timeout') || errorMessage.includes('Timeout')) {
    userFriendlyMessage = 'Request timed out. The backend may be experiencing high load. Please try again.';
    errorType = ErrorTypes.TIMEOUT;
  } else if (errorMessage.includes('CORS') || errorMessage.includes('blocked by CORS policy')) {
    userFriendlyMessage = 'Backend configuration error (CORS). Please check backend CORS settings.';
    errorType = ErrorTypes.CORS_ERROR;
  } else if (errorMessage.includes('400') || errorMessage.toLowerCase().includes('validation')) {
    userFriendlyMessage = 'Invalid query - please check your input.';
    errorType = ErrorTypes.VALIDATION_ERROR;
  } else if (errorMessage.includes('429')) {
    userFriendlyMessage = 'Too many requests. Please wait a moment before trying again.';
    errorType = 'rate_limit';
  } else if (errorMessage.includes('401') || errorMessage.includes('403')) {
    userFriendlyMessage = 'Access denied. Please check your permissions.';
    errorType = 'authorization';
  } else if (errorMessage.includes('500') || errorMessage.toLowerCase().includes('backend error')) {
    userFriendlyMessage = 'Backend processing error - please try again.';
    errorType = ErrorTypes.BACKEND_ERROR;
  } else if (errorMessage.includes('503') || errorMessage.includes('502') || errorMessage.toLowerCase().includes('unavailable') || errorMessage.toLowerCase().includes('bad gateway')) {
    userFriendlyMessage = 'Service temporarily unavailable - please try again later.';
    errorType = ErrorTypes.SERVICE_UNAVAILABLE;
  } else if (errorMessage.toLowerCase().includes('malformed') || errorMessage.toLowerCase().includes('invalid response') || errorMessage.toLowerCase().includes('parse')) {
    userFriendlyMessage = 'Received invalid response format from backend.';
    errorType = ErrorTypes.MALFORMED_RESPONSE;
  }

  // Use user-friendly message as the main message for display
  return {
    message: userFriendlyMessage,
    type: errorType,
    originalError: error,
    context,
    timestamp: new Date().toISOString(),
    technicalMessage: errorMessage  // Keep original for debugging
  };
}

/**
 * Create an error message for the chat display
 * @param {Error|string|Object} error - The error to format
 * @param {string} [context=''] - Context where the error occurred
 * @returns {Object} Formatted message object for chat display
 */
export function createErrorMessage(error, context = '') {
  const formattedError = formatError(error, context);

  return {
    id: generateId(),
    role: 'assistant',
    content: formattedError.message,
    timestamp: new Date().toISOString(),
    status: 'error'
  };
}

/**
 * Log error with context for debugging
 * @param {Error|string|Object} error - The error to log
 * @param {string} [context=''] - Context where the error occurred
 * @param {Object} [additionalData={}] - Additional data to log
 */
export function logError(error, context = '', additionalData = {}) {
  const formattedError = formatError(error, context);

  console.error('Chat Interface Error:', {
    message: formattedError.message,
    type: formattedError.type,
    context: formattedError.context,
    timestamp: formattedError.timestamp,
    additionalData,
    originalError: formattedError.originalError
  });
}

/**
 * Check if an error is recoverable
 * @param {Object} formattedError - Formatted error object
 * @returns {boolean} Whether the error is recoverable
 */
export function isRecoverableError(formattedError) {
  // Network errors, timeouts, and service unavailable errors are typically recoverable
  return ['network', 'timeout', 'service_unavailable'].includes(formattedError.type);
}

/**
 * Generate a unique ID for error tracking
 * @returns {string} Unique identifier
 */
function generateId() {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
}

/**
 * Handle streaming error from SSE
 * @param {Event} event - The error event from EventSource
 * @returns {Object} Formatted error object
 */
export function handleStreamingError(event) {
  let errorMessage = 'Streaming connection error';

  if (event && event.type === 'error') {
    errorMessage = 'Connection to streaming service lost. Retrying...';
  } else if (typeof event === 'string') {
    errorMessage = event;
  } else if (event && typeof event === 'object') {
    if (event.error) {
      errorMessage = event.error;
    }
  }

  return {
    message: errorMessage,
    type: 'streaming',
    timestamp: new Date().toISOString()
  };
}

/**
 * Handle malformed response error
 * @param {string} responseText - The malformed response text
 * @returns {Object} Formatted error object
 */
export function handleMalformedResponse(responseText) {
  return {
    message: 'Received invalid response format from server. Please try again.',
    type: 'malformed_response',
    responseText,
    timestamp: new Date().toISOString()
  };
}