"""
Database model for Query entity
Represents a user's query to the chatbot
"""
from sqlalchemy import Column, String, DateTime, Boolean, JSON, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class Query(Base):
    __tablename__ = "queries"

    query_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("user_sessions.session_id"), nullable=False)  # Reference to user session
    query_text = Column(String, nullable=False)  # The user's original query text (sanitized)
    query_timestamp = Column(DateTime, default=datetime.utcnow)  # When the query was submitted
    selected_text = Column(String, nullable=True)  # Optional highlighted text for context
    query_type = Column(String, nullable=False)  # Type of query ('general' or 'selected-text')
    processed = Column(Boolean, default=False)  # Whether query has been processed
    query_metadata = Column(JSON)  # Additional query metadata

    def __repr__(self):
        return f"<Query(query_id={self.query_id}, session_id={self.session_id}, query_type={self.query_type})>"