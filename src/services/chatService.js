/**
 * Backend communication logic for the chat interface
 */

// Default configuration - production values hardcoded
// React components can override these using updateConfig()
const DEFAULT_CONFIG = {
  BACKEND_URL: 'https://syedmuhammadanasmaududi-rag-chabot.hf.space',
  STREAMING_ENDPOINT: '/api/chat',
  MESSAGE_LIMIT: 100,
  TIMEOUT_MS: 30000,
  RETRY_ATTEMPTS: 3,
  RETRY_DELAY_MS: 1000
};

// Store current configuration
let currentConfig = { ...DEFAULT_CONFIG };

/**
 * Get current chat service configuration
 * @returns {Object} Current configuration
 */
export function getConfig() {
  return { ...currentConfig };
}

/**
 * Update chat service configuration
 * @param {Object} newConfig - New configuration values to merge
 */
export function updateConfig(newConfig) {
  currentConfig = {
    ...currentConfig,
    ...newConfig
  };
}

/**
 * Validate backend URL format
 * @param {string} url - URL to validate
 * @returns {Object} Validation result {valid: boolean, error?: string}
 */
export function validateBackendUrl(url) {
  if (!url || typeof url !== 'string') {
    return { valid: false, error: 'Backend URL is required' };
  }

  try {
    const parsed = new URL(url);

    // Must be HTTP or HTTPS
    if (!['http:', 'https:'].includes(parsed.protocol)) {
      return { valid: false, error: 'Backend URL must use HTTP or HTTPS protocol' };
    }

    // Must have a host
    if (!parsed.host) {
      return { valid: false, error: 'Backend URL must have a valid host' };
    }

    return { valid: true };
  } catch (error) {
    return { valid: false, error: `Invalid URL format: ${error.message}` };
  }
}

/**
 * Get the full streaming endpoint URL
 * @returns {string} Full URL for streaming endpoint
 * @throws {Error} If backend URL is invalid
 */
export function getStreamingUrl() {
  const { BACKEND_URL, STREAMING_ENDPOINT } = currentConfig;

  // Validate backend URL and warn if invalid
  const validation = validateBackendUrl(BACKEND_URL);
  if (!validation.valid) {
    console.error('Backend URL validation failed:', validation.error);
    console.warn('⚠️ BACKEND_URL is missing or invalid. Please check your environment configuration.');
    throw new Error(validation.error);
  }

  // Ensure proper URL formatting
  const baseUrl = BACKEND_URL.endsWith('/') ? BACKEND_URL.slice(0, -1) : BACKEND_URL;
  const endpoint = STREAMING_ENDPOINT.startsWith('/') ? STREAMING_ENDPOINT : `/${STREAMING_ENDPOINT}`;
  return `${baseUrl}${endpoint}`;
}

/**
 * Validate a query before sending
 * @param {string} query - The user query to validate
 * @returns {Object} Validation result with isValid and error message
 */
export function validateQuery(query) {
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
 * Prepare a query for sending to the backend
 * @param {string} query - The user query
 * @returns {Object} Prepared request body
 */
export function prepareQuery(query) {
  return {
    query: query.trim()
  };
}

/**
 * Send a query to the backend and return a promise
 * This function is for non-streaming requests (if needed for health checks, etc.)
 * @param {string} query - The user query
 * @returns {Promise<Object>} Promise resolving to response
 */
export async function sendQuery(query) {
  const validation = validateQuery(query);
  if (!validation.isValid) {
    throw new Error(validation.error);
  }

  const requestBody = prepareQuery(query);
  const url = getStreamingUrl();

  const response = await fetch(url, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(requestBody)
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.error || `HTTP error! status: ${response.status}`);
  }

  return response.json();
}

/**
 * Initialize the chat service with configuration
 * @param {Object} config - Configuration options to override defaults
 */
export function initializeChatService(config = {}) {
  updateConfig(config);

  // Validate configuration
  if (!currentConfig.BACKEND_URL) {
    console.warn('No BACKEND_URL configured, using default');
  }

  if (!currentConfig.STREAMING_ENDPOINT) {
    console.warn('No STREAMING_ENDPOINT configured, using default');
  }
}

// Initialize with default configuration
initializeChatService();

// Export default configuration for reference
export { DEFAULT_CONFIG };