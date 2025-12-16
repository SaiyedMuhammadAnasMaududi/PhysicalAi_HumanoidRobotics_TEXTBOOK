"""
Query request schema
"""
from pydantic import BaseModel, Field
from typing import Optional
import uuid


class QueryRequest(BaseModel):
    """Schema for query requests"""
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="The user's question or query"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Optional highlighted text for context"
    )
    session_id: Optional[str] = Field(
        None,
        description="Optional session identifier"
    )

    class Config:
        schema_extra = {
            "example": {
                "query": "Explain the principles of physical AI in humanoid robotics",
                "selected_text": "Physical AI combines machine learning with physical interaction...",
                "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
            }
        }

    def validate_session_id(self):
        """Validate session ID format if provided"""
        if self.session_id:
            try:
                uuid.UUID(self.session_id)
                return True
            except ValueError:
                return False