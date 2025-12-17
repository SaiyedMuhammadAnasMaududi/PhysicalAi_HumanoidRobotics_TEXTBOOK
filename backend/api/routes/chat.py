"""
Chat Route Implementation for Agent-Based RAG Backend

This module implements the POST /chat endpoint with request/response validation.
"""
from fastapi import APIRouter, HTTPException
from typing import Dict, Any
import logging

from ..models.chat import ChatRequest, ChatResponse
from ..services.chat import ChatService

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest) -> ChatResponse:
    """
    [T023] Implement POST /chat endpoint in backend/api/routes/chat.py with request/response validation
    """
    try:
        # Validate the request
        if not chat_request.query or not chat_request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(chat_request.query) > 1000:
            raise HTTPException(status_code=400, detail="Query exceeds maximum length of 1000 characters")

        # Log the incoming request
        logger.info(f"Received chat request: {chat_request.query[:50]}...")

        # Process the request using the chat service
        chat_service = ChatService()
        response = chat_service.process_request(chat_request)

        # Log the successful response
        logger.info(f"Processed chat request successfully, execution time: {response.execution_time_ms}ms")

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error and raise an internal server error
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")