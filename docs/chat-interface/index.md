---
sidebar_position: 999
title: 'Interactive Chat'
---

# Interactive Chat Interface

Welcome to the interactive chat interface for the Physical AI & Humanoid Robotics textbook! You can ask questions about the content covered in this textbook and get real-time streaming responses.

import ChatInterface from '@site/src/components/ChatInterface';

<ChatInterface
  backendUrl="http://localhost:8000"
  streamingEndpoint="/api/chat/stream"
  messageLimit={100}
/>

## How to Use

- Type your question in the input field at the bottom
- Press Enter or click Send to submit
- Watch the response stream in real-time
- Scroll up to review previous conversation history
- The interface will maintain context within the current session

## About This Feature

This chat interface is integrated with our RAG (Retrieval Augmented Generation) system that provides accurate responses based on the textbook content. The streaming functionality allows you to see responses as they're generated, providing a more interactive experience.

The interface is fully responsive and accessible, with features like:
- Auto-scroll that respects manual scrolling
- Error handling and recovery
- Docusaurus theme integration
- Keyboard navigation support