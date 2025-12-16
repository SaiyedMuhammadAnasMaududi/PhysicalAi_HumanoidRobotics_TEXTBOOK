"""
Query response schema
"""
from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid


class Citation(BaseModel):
    """Schema for citation objects"""
    chapter: str = Field(
        ...,
        description="Book chapter reference"
    )
    section: str = Field(
        ...,
        description="Specific section within the chapter"
    )
    similarity_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score between 0 and 1"
    )

    class Config:
        schema_extra = {
            "example": {
                "chapter": "Chapter 3",
                "section": "Section 2.1",
                "similarity_score": 0.85
            }
        }


class QueryResponse(BaseModel):
    """Schema for query responses"""
    response_id: str = Field(
        ...,
        description="Unique identifier for the response"
    )
    answer: str = Field(
        ...,
        description="The generated answer to the query"
    )
    citations: List[Citation] = Field(
        ...,
        description="List of source citations used in the response"
    )
    provenance: Optional[str] = Field(
        None,
        description="Source information for the response"
    )
    query_time: float = Field(
        ...,
        description="Time taken to process the query in milliseconds"
    )

    class Config:
        schema_extra = {
            "example": {
                "response_id": "f0e9d8c7-b6a5-4321-fedc-ba9876543210",
                "answer": "Physical AI in humanoid robotics combines machine learning algorithms with physical interaction models to enable robots to understand and interact with their environment effectively...",
                "citations": [
                    {
                        "chapter": "Chapter 3",
                        "section": "Section 2.1",
                        "similarity_score": 0.85
                    }
                ],
                "provenance": "Based on content from Chapter 3, Section 2.1 and Chapter 5, Section 3.4",
                "query_time": 1250.0
            }
        }


class ErrorResponse(BaseModel):
    """Schema for error responses"""
    error: str = Field(
        ...,
        description="Error code"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )

    class Config:
        schema_extra = {
            "example": {
                "error": "QUERY_PROCESSING_ERROR",
                "message": "Failed to process the query due to retrieval error"
            }
        }


class HealthResponse(BaseModel):
    """Schema for health check responses"""
    status: str = Field(
        ...,
        description="Overall operational status"
    )
    qdrant_status: str = Field(
        ...,
        description="Vector database status"
    )
    neon_status: str = Field(
        ...,
        description="Database status"
    )
    models_status: str = Field(
        ...,
        description="LLM availability status"
    )
    timestamp: datetime = Field(
        ...,
        description="When the health check was performed"
    )

    class Config:
        schema_extra = {
            "example": {
                "status": "operational",
                "qdrant_status": "connected",
                "neon_status": "connected",
                "models_status": "available",
                "timestamp": "2025-12-16T10:00:00Z"
            }
        }