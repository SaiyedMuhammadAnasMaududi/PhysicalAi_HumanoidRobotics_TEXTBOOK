"""
Database model for Response entity
Stores the generated response to a user query
"""
from sqlalchemy import Column, String, DateTime, JSON, Boolean, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class Response(Base):
    __tablename__ = "responses"

    response_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.query_id"), nullable=False)  # Reference to the original query
    response_text = Column(String, nullable=False)  # The generated response text
    source_citations = Column(JSON, nullable=False)  # List of source citations used in response
    generation_timestamp = Column(DateTime, default=datetime.utcnow)  # When response was generated
    response_metadata = Column(JSON)  # Additional metadata about generation process
    accuracy_verified = Column(Boolean, default=False, nullable=False)  # Whether response was verified against book content

    def __repr__(self):
        return f"<Response(response_id={self.response_id}, query_id={self.query_id}, accuracy_verified={self.accuracy_verified})>"