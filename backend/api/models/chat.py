"""
Data models for the chat API endpoint.

These models define the structure for requests and responses in the agent-driven RAG system.
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class SourceReference(BaseModel):
    """
    Represents a citation to a specific book section used in the response
    """
    source_file: str = Field(..., description="Path to the source document")
    section_title: str = Field(..., description="Title of the section containing the information")
    chunk_id: str = Field(..., description="Unique identifier of the content chunk")
    similarity_score: float = Field(..., description="Relevance score between 0.0 and 1.0", ge=0.0, le=1.0)
    content_preview: Optional[str] = Field(None, description="Short preview of the cited content")


class ChatRequest(BaseModel):
    """
    Represents a user's query to the chat system
    """
    query: str = Field(..., description="The natural language question from the user", min_length=1, max_length=1000)
    user_context: Optional[Dict[str, Any]] = Field(None, description="Additional context provided by the user")


class ChatResponse(BaseModel):
    """
    Represents the agent's response to a user's query
    """
    response: str = Field(..., description="The agent-generated answer", min_length=1)
    sources: List[SourceReference] = Field(..., description="Citations to book sections used in response")
    execution_time_ms: float = Field(..., description="Time taken to process the query in milliseconds", ge=0)
    timestamp: datetime = Field(..., description="When the response was generated")
    query_id: str = Field(..., description="Unique identifier for tracking")