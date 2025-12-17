"""
Basic Agent Service for the RAG System

This module implements a basic service that orchestrates agent responses without retrieval integration.
"""
import os
import uuid
import time
from datetime import datetime
from typing import Dict, Any
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class BasicAgentService:
    """
    [T010] Create basic agent service in backend/src/basic_agent_service.py that orchestrates agent responses
    """

    def __init__(self):
        """
        Initialize the basic agent service
        """
        # Just initialize basic configuration
        self.litellm_model = os.getenv("LITELLM_MODEL", "gpt-3.5-turbo")
        self.agent_temperature = float(os.getenv("AGENT_TEMPERATURE", "0.1"))

        print(f"BasicAgentService initialized with model: {self.litellm_model}")

    def query(self, query_text: str, user_context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        [T011] Implement simple query method in BasicAgentService that calls agent without retrieval
        """
        start_time = time.time()

        # Generate a query ID for tracking
        query_id = f"req-{str(uuid.uuid4())}"

        # Call the agent with retrieval integration
        from .agent import RAGAgent
        agent = RAGAgent()
        response = agent.query(query_text, user_context)

        # Calculate execution time
        execution_time_ms = (time.time() - start_time) * 1000

        # Extract source information if available in the response
        sources = []
        if hasattr(agent, 'last_retrieved_context') and agent.last_retrieved_context:
            for chunk in agent.last_retrieved_context:
                sources.append({
                    "source_file": chunk.get("source_file", ""),
                    "section_title": chunk.get("section_title", ""),
                    "chunk_id": chunk.get("chunk_id", ""),
                    "similarity_score": chunk.get("similarity_score", 0.0),
                    "content_preview": chunk.get("content", "")[:200]  # First 200 chars as preview
                })

        # Return response with metadata
        return {
            "response": response,
            "sources": sources,
            "execution_time_ms": execution_time_ms,
            "timestamp": datetime.now().isoformat(),
            "query_id": query_id
        }