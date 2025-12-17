"""
Chat Service Interface for Agent-Based RAG Backend

This module implements the chat service that connects API endpoints to the agent service.
"""
from typing import Dict, Any
import sys
from pathlib import Path

# Add backend directory to path for imports
backend_dir = Path(__file__).parent.parent.parent
sys.path.insert(0, str(backend_dir))

from src.basic_agent_service import BasicAgentService
from api.models.chat import ChatRequest, ChatResponse, SourceReference
from datetime import datetime


class ChatService:
    """
    [T014] Create basic chat service interface in backend/api/services/chat.py
    """

    def __init__(self):
        """
        Initialize the chat service with the agent service
        """
        self.agent_service = BasicAgentService()

    def process_request(self, chat_request: ChatRequest) -> ChatResponse:
        """
        [T025] Add response formatting to match API contract schema per contracts/chat_api.md
        [T027] Add source attribution to responses with proper citation format
        """
        # Call the agent service to process the query
        result = self.agent_service.query(
            query_text=chat_request.query,
            user_context=chat_request.user_context
        )

        # Convert the result to a ChatResponse model
        # Extract sources from the agent response if available
        sources = []
        if "sources" in result and result["sources"]:
            for source_data in result["sources"]:
                source_ref = SourceReference(
                    source_file=source_data.get("source_file", ""),
                    section_title=source_data.get("section_title", ""),
                    chunk_id=source_data.get("chunk_id", ""),
                    similarity_score=source_data.get("similarity_score", 0.0),
                    content_preview=source_data.get("content_preview", "")
                )
                sources.append(source_ref)

        response = ChatResponse(
            response=result["response"],
            sources=sources,
            execution_time_ms=result["execution_time_ms"],
            timestamp=datetime.fromisoformat(result["timestamp"]) if isinstance(result["timestamp"], str) else result["timestamp"],
            query_id=result["query_id"]
        )

        return response