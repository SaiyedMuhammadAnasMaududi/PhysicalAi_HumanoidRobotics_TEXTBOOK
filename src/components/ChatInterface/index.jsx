/**
 * Main export file for the ChatInterface component
 * Exports the React component for use in Docusaurus and config utilities
 */

import ChatInterfaceReact from './ChatInterfaceReact';
import { getConfig, updateConfig, getStreamingUrl, validateBackendUrl } from '../../services/chatService.js';

// Export the main component as default
export default ChatInterfaceReact;

// Export config utilities for external consumption
export const ChatConfig = {
  getConfig,
  updateConfig,
  getStreamingUrl,
  validateBackendUrl
};