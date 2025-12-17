# Quickstart Guide: Agent-Based RAG Backend using OpenAI Agent SDK with LiteLLM Models

**Feature**: 001-agent-rag-backend
**Date**: 2025-12-17
**Input**: Implementation plan and API contracts

## Overview

This guide provides step-by-step instructions to quickly set up and run the agent-based RAG backend using OpenAI Agent SDK with LiteLLM models. The system integrates an AI agent with the existing retrieval pipeline to answer questions about technical book content through a FastAPI endpoint, using function_tool for retrieval and SQLiteSession for session management.

## Prerequisites

Before starting, ensure you have:

- Python 3.11 or higher
- Access to Gemini API (for agent functionality via LiteLLM)
- Access to Cohere API (for embedding, from Specs 1-2)
- Access to Qdrant database (with 'physicalai' collection populated from Specs 1-2)
- Completed Specs 1-2 (embedding pipeline and retrieval system)

## Setup Instructions

### 1. Clone or Navigate to Repository

```bash
cd /path/to/PhysicalAi_HumanoidRobotics_TEXTBOOK
```

### 2. Set up Environment Variables

Create or update your `.env` file in the backend directory with the following variables:

```bash
# From previous specs (Specs 1 & 2)
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=physicalai
EMBEDDING_MODEL=embed-multilingual-v3.0

# New for agent functionality
GEMINI_API_KEY=your_gemini_api_key_here
LITELLM_MODEL=gemini/gemini-2.0-flash
AGENT_TEMPERATURE=0.1
```

### 3. Install Dependencies

Add the required dependencies to your `backend/requirements.txt`:

```txt
agents>=0.1.0
litellm>=1.0.0
fastapi>=0.100.0
uvicorn>=0.20.0
python-dotenv>=1.0.0
pydantic>=2.0.0
sqlite3
```

Then install:

```bash
pip install -r backend/requirements.txt
```

### 4. Verify Retrieval Pipeline

Before starting the agent service, verify that the retrieval pipeline from Specs 1-2 works:

```bash
python3 backend/scripts/test_retrieval.py "What is ROS2?" --top_k 3
```

You should see results returned from the Qdrant database.

## Running the Service

### 1. Start the FastAPI Server

```bash
cd backend
uvicorn api.main:app --host 0.0.0.0 --port 8000 --reload
```

The server will start on `http://localhost:8000`.

### 2. Verify Service is Running

Check the server status:

```bash
curl http://localhost:8000/health
```

Expected response: `{"status": "healthy", "timestamp": "2025-12-17T02:05:52.449099"}`

## Using the Chat API

### 1. Basic Query

Send a query to the chat endpoint:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?"
  }'
```

### 2. Query with Context

Send a query with additional user context:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain computer vision in robotics",
    "user_context": {
      "technical_level": "beginner"
    }
  }'
```

### 3. Expected Response Format

A successful response will look like:

```json
{
  "response": "ROS2 (Robot Operating System 2) is an open-source framework...",
  "sources": [
    {
      "source_file": "../docs/module-01-ros2/01-introduction.md",
      "section_title": "Introduction to ROS 2",
      "chunk_id": "uuid-here",
      "similarity_score": 0.7123
    }
  ],
  "execution_time_ms": 1577,
  "timestamp": "2025-12-17T02:05:52.449099",
  "query_id": "req-uuid-here"
}
```

## Testing Different Queries

### Technical Queries
```bash
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "How do I set up a ROS2 workspace?"}'
```

### Cross-Module Queries
```bash
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "How does perception connect to control in robotics?"}'
```

### General Queries
```bash
curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What are the key components of a humanoid robot?"}'
```

## Configuration Options

### Model Selection

Change the LLM model by updating the `LITELLM_MODEL` environment variable:

```bash
# For Gemini Flash (recommended for speed/cost)
LITELLM_MODEL=gemini/gemini-2.0-flash

# For other Gemini models
LITELLM_MODEL=gemini/gemini-pro
```

### Agent Parameters

Adjust agent behavior with these environment variables:

```bash
# Temperature (0.0 = deterministic, 1.0 = creative)
AGENT_TEMPERATURE=0.1

# Maximum tokens for response
AGENT_MAX_TOKENS=1000

# Timeout for agent processing (in seconds)
AGENT_TIMEOUT=30
```

## Troubleshooting

### Common Issues

#### 1. API Keys Not Working
- Verify all API keys in `.env` are correct
- Check that the keys have the required permissions
- Ensure `.env` file is in the correct directory and being loaded

#### 2. Qdrant Connection Issues
- Verify QDRANT_URL and QDRANT_API_KEY are correct
- Check that the 'physicalai' collection exists and has data
- Test retrieval directly with `test_retrieval.py`

#### 3. Slow Response Times
- Check that all API services are accessible
- Verify network connectivity to external APIs
- Monitor system resources (CPU, memory)

#### 4. Empty or Irrelevant Responses
- Verify the retrieval pipeline is working correctly
- Check that the vector database has relevant content
- Adjust retrieval parameters if needed

### Error Response Examples

#### Bad Request (400)
```json
{
  "error": "Query cannot be empty",
  "error_code": "EMPTY_QUERY",
  "timestamp": "2025-12-17T02:05:52.449099"
}
```

#### Server Error (500)
```json
{
  "error": "Failed to process query due to internal error",
  "error_code": "INTERNAL_ERROR",
  "timestamp": "2025-12-17T02:05:52.449099"
}
```

## Performance Testing

### Load Testing
```bash
# Test concurrent requests
for i in {1..10}; do
  curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}' &
done
wait
```

### Response Time Monitoring
```bash
# Time a single request
time curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query": "What is ROS2?"}'
```

## Next Steps

1. **Integration Testing**: Test with various query types to ensure proper grounding via function_tool
2. **Performance Tuning**: Optimize response times based on your requirements
3. **Error Handling**: Implement additional error handling for production use
4. **Monitoring**: Set up logging and monitoring for the service
5. **Documentation**: Generate API documentation using FastAPI's built-in documentation at `/docs`

## API Documentation

Once the server is running, you can access interactive API documentation at:
- `http://localhost:8000/docs` - Swagger UI
- `http://localhost:8000/redoc` - ReDoc

These provide interactive testing and detailed API information.