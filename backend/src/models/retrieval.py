"""
Database model for RetrievalResult entity
Stores the results from the vector database retrieval
"""
from sqlalchemy import Column, String, DateTime, Float, JSON, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from .base import Base

class RetrievalResult(Base):
    __tablename__ = "retrieval_results"

    result_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.query_id"), nullable=False)  # Reference to the original query
    source_document = Column(String, nullable=False)  # Reference to source book section/chapter
    content_snippet = Column(String, nullable=False)  # Retrieved text snippet
    similarity_score = Column(Float, nullable=False)  # Similarity score from vector search (0-1)
    retrieval_timestamp = Column(DateTime, default=datetime.utcnow)  # When retrieval was performed
    chunk_id = Column(String, nullable=True)  # ID of the specific content chunk retrieved

    def __repr__(self):
        return f"<RetrievalResult(result_id={self.result_id}, query_id={self.query_id}, similarity_score={self.similarity_score})>"