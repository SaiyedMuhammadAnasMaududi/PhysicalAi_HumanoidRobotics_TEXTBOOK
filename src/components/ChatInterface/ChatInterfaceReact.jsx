/**
 * React component for the streaming chat interface
 * This component wraps the vanilla JavaScript chat interface for use in Docusaurus
 */

import React, { useEffect, useRef, useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import './ChatStyles.css'; // Import the styles

/**
 * Internal component that only runs in the browser
 */
function ChatInterfaceInternal({ backendUrl, streamingEndpoint, messageLimit, onStateChange, className = '', style = {} }) {
  const chatContainerRef = useRef(null);
  const chatInstanceRef = useRef(null);
  const [chatLoaded, setChatLoaded] = useState(false);
  const [error, setError] = useState(null);

  useEffect(() => {
    let isMounted = true;

    // Dynamic import to avoid SSR issues
    import('./ChatContainer.jsx')
      .then(({ ChatContainer }) => {
        if (!isMounted || !chatContainerRef.current) {
          return;
        }

        // Clear any existing content safely
        while (chatContainerRef.current.firstChild) {
          chatContainerRef.current.removeChild(chatContainerRef.current.firstChild);
        }

        // Create the chat container with the specified configuration
        const chatComponent = ChatContainer({
          backendUrl,
          streamingEndpoint,
          messageLimit,
          onStateChange
        });

        // Append the chat component to the container
        chatContainerRef.current.appendChild(chatComponent);
        chatInstanceRef.current = chatComponent;

        if (isMounted) {
          setChatLoaded(true);
        }
      })
      .catch(error => {
        console.error('Error loading chat container:', error);
        if (isMounted) {
          setError(error.message);
        }
      });

    // Cleanup function
    return () => {
      isMounted = false;

      // Clean up chat instance
      if (chatInstanceRef.current && chatInstanceRef.current.cleanup) {
        try {
          chatInstanceRef.current.cleanup();
        } catch (e) {
          console.error('Error cleaning up chat:', e);
        }
      }

      // Clear the container safely
      if (chatContainerRef.current) {
        while (chatContainerRef.current.firstChild) {
          chatContainerRef.current.removeChild(chatContainerRef.current.firstChild);
        }
      }

      chatInstanceRef.current = null;
    };
  }, [backendUrl, streamingEndpoint, messageLimit, onStateChange]);

  const combinedClassName = `chat-interface-container ${className}`.trim();
  const combinedStyle = {
    height: '500px',
    ...style
  };

  if (error) {
    return (
      <div ref={chatContainerRef} className={combinedClassName} style={combinedStyle}>
        <div className="chat-error">Failed to load chat: {error}</div>
      </div>
    );
  }

  return (
    <div ref={chatContainerRef} className={combinedClassName} style={combinedStyle}>
      {!chatLoaded && <div className="chat-loading">Loading chat interface...</div>}
    </div>
  );
}

/**
 * React component for the chat interface
 * @param {Object} props - Component properties
 * @param {string} [props.backendUrl] - Backend URL to override default
 * @param {string} [props.streamingEndpoint] - Streaming endpoint to override default
 * @param {number} [props.messageLimit] - Maximum number of messages to display
 * @param {Function} [props.onStateChange] - Callback when chat state changes
 * @param {string} [props.className] - Additional CSS class name
 * @param {Object} [props.style] - Additional inline styles
 * @returns {JSX.Element} React component
 */
const ChatInterfaceReact = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <ChatInterfaceInternal {...props} />}
    </BrowserOnly>
  );
};

export default ChatInterfaceReact;
