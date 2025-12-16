"""
Chatbot API endpoints
"""
from fastapi import APIRouter, Depends, HTTPException
from typing import Dict, Any
from datetime import datetime
import uuid
from ..schemas.query import QueryRequest
from ..schemas.response import QueryResponse
from ..database import get_db, AsyncSessionLocal
from ..services.session_service import SessionService
from ..services.query_service import QueryService
from ..vector_db import VectorDBService
from ..llm_service import LLMService
from ..middleware.sanitization import InputSanitizationMiddleware
from ..logging import log_interaction, get_logger

router = APIRouter()
logger = get_logger(__name__)


@router.post("/query", response_model=QueryResponse)
async def process_query(
    query_request: QueryRequest,
    db: AsyncSessionLocal = Depends(get_db)
):
    """
    Submit a query to the RAG chatbot
    Process a user query and return a response based on book content
    """
    start_time = datetime.utcnow()

    try:
        # Validate and sanitize input
        middleware = InputSanitizationMiddleware()
        validated_request = middleware.sanitize_query_request(query_request.dict())

        # Use the validated request data
        query_text = validated_request['query']
        selected_text = validated_request.get('selected_text')
        session_id = validated_request.get('session_id')

        # Create session if not provided
        if not session_id:
            session_service = SessionService(db)
            session = await session_service.create_session()
            session_id = str(session.session_id)
        else:
            # Validate that session exists and is active
            session_service = SessionService(db)
            session = await session_service.get_session(session_id)
            if not session:
                raise HTTPException(status_code=400, detail="Invalid or inactive session")

        # Initialize services
        vector_db = VectorDBService()
        llm_service = LLMService()
        query_service = QueryService(db, vector_db, llm_service)

        # Process the query
        result = await query_service.process_query(
            session_id=session_id,
            query_text=query_text,
            selected_text=selected_text
        )

        # Calculate query time
        query_time = (datetime.utcnow() - start_time).total_seconds() * 1000  # Convert to milliseconds
        result["query_time"] = query_time

        # Log successful query processing
        await log_interaction(
            db,
            session_id,
            result.get('response_id', ''),
            "query_success",
            {"query_time_ms": query_time}
        )

        return QueryResponse(**result)

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        await log_interaction(
            db,
            session_id or "unknown",
            "unknown",
            "query_error",
            {"error": str(e)}
        )
        raise HTTPException(
            status_code=500,
            detail=f"Failed to process the query: {str(e)}"
        )


@router.post("/session")
async def create_session(
    db: AsyncSessionLocal = Depends(get_db)
):
    """
    Create a new user session
    Initialize a new session for a user
    """
    try:
        session_service = SessionService(db)
        session = await session_service.create_session()

        return {
            "session_id": str(session.session_id),
            "created_at": session.created_at.isoformat()
        }
    except Exception as e:
        logger.error(f"Error creating session: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create session: {str(e)}"
        )