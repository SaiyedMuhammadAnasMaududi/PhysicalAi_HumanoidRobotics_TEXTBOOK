"""
Database model for BookContent entity
Represents the book content that serves as the knowledge base
"""
from sqlalchemy import Column, String, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class BookContent(Base):
    __tablename__ = "book_content"

    content_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter = Column(String, nullable=False)  # Book chapter identifier
    section = Column(String, nullable=False)  # Section within the chapter
    content_text = Column(String, nullable=False)  # The actual text content
    # Note: embedding_vector is not stored in Postgres as it's in the vector DB (Qdrant)
    created_at = Column(DateTime, default=datetime.utcnow)  # When this content was indexed
    version = Column(String, nullable=False)  # Version of the book content
    content_metadata = Column(JSON)  # Additional metadata about the content

    def __repr__(self):
        return f"<BookContent(content_id={self.content_id}, chapter={self.chapter}, section={self.section})>"