"""
Database model for UserSession entity
Tracks individual user sessions with the chatbot
"""
from sqlalchemy import Column, String, DateTime, Boolean, JSON
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class UserSession(Base):
    __tablename__ = "user_sessions"

    session_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=True)  # Optional identifier for registered users
    created_at = Column(DateTime, default=datetime.utcnow)
    last_interaction = Column(DateTime, default=datetime.utcnow)
    active = Column(Boolean, default=True)  # Whether the session is still active
    session_metadata = Column(JSON)  # Additional session-specific data

    def __repr__(self):
        return f"<UserSession(session_id={self.session_id}, user_id={self.user_id}, active={self.active})>"