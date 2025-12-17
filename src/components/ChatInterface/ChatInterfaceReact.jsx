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
  const [chatLoaded, setChatLoaded] = useState(false);

  useEffect(() => {
    // Dynamic import to avoid SSR issues
    import('./ChatContainer.jsx')
      .then(({ ChatContainer }) => {
        // Create the chat interface when component mounts
        if (chatContainerRef.current) {
          // Clear any existing content
          chatContainerRef.current.innerHTML = '';

          // Create the chat container with the specified configuration
          const chatComponent = ChatContainer({
            backendUrl,
            streamingEndpoint,
            messageLimit,
            onStateChange
          });

          // Append the chat component to the container
          chatContainerRef.current.appendChild(chatComponent);

          // Store the component instance for cleanup
          chatContainerRef.current.chatComponent = chatComponent;
          setChatLoaded(true);
        }

        // Cleanup function
        return () => {
          if (chatContainerRef.current && chatContainerRef.current.chatComponent) {
            // Call the cleanup function if available
            if (chatContainerRef.current.chatComponent.cleanup) {
              chatContainerRef.current.chatComponent.cleanup();
            }
            chatContainerRef.current.innerHTML = '';
          }
        };
      })
      .catch(error => {
        console.error('Error loading chat container:', error);
      });
  }, [backendUrl, streamingEndpoint, messageLimit, onStateChange]);

  const combinedClassName = `chat-interface-container ${className}`.trim();
  const combinedStyle = {
    height: '500px',
    ...style
  };

  return (
    <div ref={chatContainerRef} className={combinedClassName} style={combinedStyle}>
      {!chatLoaded && <div>Loading chat interface...</div>}
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
    <BrowserOnly>
      {() => <ChatInterfaceInternal {...props} />}
    </BrowserOnly>
  );
};

export default ChatInterfaceReact;