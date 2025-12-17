"""
Agent Service orchestrator for the RAG system.

This service coordinates end-to-end query processing between API, agent, and retrieval components.
"""
import os
import logging
from datetime import datetime
from typing import Optional, Dict, Any
from dotenv import load_dotenv
import uuid

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AgentService:
    """
    Coordinate end-to-end query processing between API, agent, and retrieval
    """

    def __init__(self):
        """
        Initialize agent service with configuration from environment variables
        """
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.litellm_model = os.getenv("LITELLM_MODEL", "gpt-3.5-turbo")
        self.agent_temperature = float(os.getenv("AGENT_TEMPERATURE", "0.1"))
        self.cohere_api_key = os.getenv("COHERE_API_KEY")
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Validate required configuration
        if not self.openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        logger.info("AgentService initialized with required configuration")

    def generate_query_id(self) -> str:
        """
        Generate a unique identifier for tracking this query
        """
        return f"req-{str(uuid.uuid4())}"

    def get_config(self) -> Dict[str, Any]:
        """
        Get the current configuration
        """
        return {
            "litellm_model": self.litellm_model,
            "agent_temperature": self.agent_temperature,
            "qdrant_collection": os.getenv("QDRANT_COLLECTION", "physicalai")
        }