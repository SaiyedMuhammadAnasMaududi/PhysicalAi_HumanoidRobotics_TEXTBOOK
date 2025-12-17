/**
 * Streaming response processing service using Server-Sent Events (SSE)
 */

import { formatStreamChunk, updateMessageContent, updateStreamingStatus, setErrorState } from '../utils/messageFormatter.js';
import { formatError, logError, handleStreamingError, handleMalformedResponse } from '../utils/errorHandler.js';
import { getConfig, getStreamingUrl } from './chatService.js';

// Check if EventSource is available (with polyfill support)
const EventSource = window.EventSource || globalThis.EventSource;

/**
 * Establish an SSE connection to stream responses from the backend
 * @param {string} query - The user query to send
 * @param {Function} onMessageUpdate - Callback for incremental message updates
 * @param {Function} onStatusChange - Callback for status changes
 * @param {Function} onError - Callback for error handling
 * @returns {Object} EventSource connection object with close method
 */
export function connectToStreamingEndpoint(query, onMessageUpdate, onStatusChange, onError) {
  // Validate query
  const validation = validateQuery(query);
  if (!validation.isValid) {
    const error = formatError(validation.error, 'query_validation');
    onError(error);
    return null;
  }

  // Prepare request body
  const requestBody = prepareQuery(query);
  const streamingUrl = getStreamingUrl();

  // Create a custom EventSource-like object that supports POST requests
  // Since standard EventSource doesn't support POST, we'll use fetch with streaming
  return createStreamingConnection(streamingUrl, requestBody, onMessageUpdate, onStatusChange, onError);
}

/**
 * Create a streaming connection using fetch API with ReadableStream
 * @param {string} url - The streaming endpoint URL
 * @param {Object} requestBody - The request body to send
 * @param {Function} onMessageUpdate - Callback for incremental message updates
 * @param {Function} onStatusChange - Callback for status changes
 * @param {Function} onError - Callback for error handling
 * @param {number} [attemptNumber=0] - Current retry attempt number
 * @returns {Object} Connection object with close method
 */
function createStreamingConnection(url, requestBody, onMessageUpdate, onStatusChange, onError, attemptNumber = 0) {
  let isClosed = false;
  let controller = new AbortController();
  let timeoutId = null;
  const config = getConfig();

  // Set initial timeout before fetch
  timeoutId = setTimeout(() => {
    if (!isClosed) {
      const error = formatError('Request timeout - no response received', 'request_timeout');
      onError(error);
      controller.abort();
    }
  }, config.TIMEOUT_MS);

  // Start the streaming process
  const streamPromise = fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Accept': 'application/json, text/event-stream',
      'Cache-Control': 'no-cache',
    },
    body: JSON.stringify(requestBody),
    signal: controller.signal
  })
  .then(async (response) => {
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      const error = formatError({
        error: errorData.error || `HTTP error! status: ${response.status}`,
        status: response.status
      }, 'streaming_response');
      onError(error);
      return;
    }

    if (!response.body) {
      const error = formatError('No response body received', 'streaming_response');
      onError(error);
      return;
    }

    // Check content type - handle both streaming and regular JSON responses
    const contentType = response.headers.get('content-type');

    // Handle regular JSON response (non-streaming)
    if (contentType && contentType.includes('application/json')) {
      try {
        const jsonData = await response.json();

        // Clear timeout
        clearTimeout(timeoutId);

        // Check if there's an error in the response
        if (jsonData.error || (jsonData.response && jsonData.response.startsWith('Error'))) {
          const errorMessage = jsonData.error || jsonData.response || 'Unknown error';
          onError(formatError(errorMessage, 'backend_error'));
          return;
        }

        // Send the complete response as a single chunk
        if (jsonData.response) {
          onMessageUpdate({
            content: jsonData.response,
            done: true
          });
          onStatusChange('complete');
        } else {
          onError(formatError('No response field in JSON', 'response_format'));
        }
        return;
      } catch (parseError) {
        const error = formatError('Failed to parse JSON response: ' + parseError.message, 'json_parse');
        onError(error);
        return;
      }
    }

    // Check if the response is streaming format
    if (!contentType || !contentType.includes('text/event-stream')) {
      const error = formatError('Invalid content type for streaming response', 'streaming_response');
      onError(error);
      return;
    }

    // Process the stream
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    // Update status to indicate streaming has started
    onStatusChange('connected');

    // Set initial timeout
    timeoutId = setTimeout(() => {
      if (!isClosed) {
        const error = formatError('Stream timeout - no response received', 'stream_timeout');
        onError(error);
        controller.abort();
      }
    }, config.TIMEOUT_MS);

    try {
      while (!isClosed) {
        const { done, value } = await reader.read();

        if (done) {
          break;
        }

        // Reset timeout on each chunk received
        clearTimeout(timeoutId);
        timeoutId = setTimeout(() => {
          if (!isClosed) {
            const error = formatError('Stream interrupted - timeout waiting for next chunk', 'stream_timeout');
            onError(error);
            controller.abort();
          }
        }, config.TIMEOUT_MS);

        // Decode the chunk
        const chunk = decoder.decode(value, { stream: true });
        buffer += chunk;

        // Process complete lines from the buffer
        const lines = buffer.split(/\r?\n/);
        buffer = lines.pop(); // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const dataStr = line.slice(6); // Remove 'data: ' prefix
            if (dataStr.trim()) {
              try {
                const data = JSON.parse(dataStr);
                handleStreamData(data, onMessageUpdate, onStatusChange, onError);
              } catch (parseError) {
                const error = handleMalformedResponse(dataStr);
                logError(parseError, 'stream_data_parse', { raw_data: dataStr });
                onError(formatError(error, 'stream_data_parse'));
              }
            }
          } else if (line.startsWith('event: ')) {
            // Handle different event types if needed
            const eventType = line.slice(7); // Remove 'event: ' prefix
            if (eventType === 'error') {
              // This would be handled in the data processing
            }
          }
        }
      }
    } catch (error) {
      if (!isClosed) {
        const formattedError = formatError(error, 'stream_read');
        onError(formattedError);
      }
    } finally {
      clearTimeout(timeoutId);
      reader.releaseLock();
      if (!isClosed) {
        onStatusChange('disconnected');
      }
    }
  })
  .catch((error) => {
    clearTimeout(timeoutId);
    if (!isClosed && attemptNumber < config.RETRY_ATTEMPTS) {
      // Retry with exponential backoff
      const retryDelay = config.RETRY_DELAY_MS * Math.pow(2, attemptNumber);
      console.log(`Retrying connection (attempt ${attemptNumber + 1}/${config.RETRY_ATTEMPTS}) after ${retryDelay}ms`);
      onStatusChange('retrying');

      setTimeout(() => {
        if (!isClosed) {
          createStreamingConnection(url, requestBody, onMessageUpdate, onStatusChange, onError, attemptNumber + 1);
        }
      }, retryDelay);
    } else if (!isClosed) {
      const formattedError = formatError(error, 'stream_connection');
      onError(formattedError);
    }
  });

  // Return a connection object with close method
  return {
    close: () => {
      isClosed = true;
      controller.abort();
    },
    isClosed: () => isClosed
  };
}

/**
 * Handle incoming stream data
 * @param {Object} data - The parsed stream data
 * @param {Function} onMessageUpdate - Callback for incremental message updates
 * @param {Function} onStatusChange - Callback for status changes
 * @param {Function} onError - Callback for error handling
 */
function handleStreamData(data, onMessageUpdate, onStatusChange, onError) {
  // Validate expected data structure
  if (!data.hasOwnProperty('content') || !data.hasOwnProperty('done')) {
    const error = handleMalformedResponse(JSON.stringify(data));
    onError(formatError(error, 'stream_data_validation'));
    return;
  }

  // Check if this is an error chunk
  if (data.error) {
    const error = formatError(data.error, 'stream_error_response');
    onError(error);
    return;
  }

  // Process the chunk
  const chunk = formatStreamChunk(data.content, data.done, data.error);

  // Update the message with the new content
  onMessageUpdate(chunk);

  // If this is the final chunk, update status
  if (chunk.done) {
    onStatusChange('complete');
  }
}

/**
 * Validate query before sending
 * @param {string} query - The query to validate
 * @returns {Object} Validation result
 */
function validateQuery(query) {
  if (!query || typeof query !== 'string') {
    return {
      isValid: false,
      error: 'Query must be a string'
    };
  }

  if (query.trim().length === 0) {
    return {
      isValid: false,
      error: 'Query cannot be empty or whitespace only'
    };
  }

  if (query.length > 1000) {
    return {
      isValid: false,
      error: 'Query exceeds maximum length of 1000 characters'
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Prepare query for sending
 * @param {string} query - The query to prepare
 * @returns {Object} Prepared request body
 */
function prepareQuery(query) {
  return {
    query: query.trim()
  };
}

/**
 * Check if streaming is supported in the current environment
 * @returns {boolean} Whether streaming is supported
 */
export function isStreamingSupported() {
  return typeof EventSource !== 'undefined' ||
         (typeof fetch !== 'undefined' && typeof ReadableStream !== 'undefined');
}

/**
 * Get streaming configuration
 * @returns {Object} Streaming configuration
 */
export function getStreamingConfig() {
  return {
    ...getConfig(),
    streamingSupported: isStreamingSupported()
  };
}