"""
Database model for AuditLog entity
Logs for tracking and auditing chatbot interactions
"""
from sqlalchemy import Column, String, DateTime, JSON, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class AuditLog(Base):
    __tablename__ = "audit_logs"

    log_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("user_sessions.session_id"), nullable=False)  # Reference to user session
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.query_id"), nullable=True)  # Reference to the query if applicable
    action = Column(String, nullable=False)  # Type of action ('query', 'response', 'error', 'health_check')
    timestamp = Column(DateTime, default=datetime.utcnow)  # When the action occurred
    user_ip_hash = Column(String, nullable=True)  # Hashed IP for privacy
    log_metadata = Column(JSON)  # Additional context about the action

    def __repr__(self):
        return f"<AuditLog(log_id={self.log_id}, action={self.action}, timestamp={self.timestamp})>"