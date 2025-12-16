"""
Response generation service
"""
from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
import uuid
from datetime import datetime
from ..models.response import Response
from ..models.query import Query
from ..logging import log_interaction


class ResponseService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_response(
        self,
        query_id: str,
        response_text: str,
        source_citations: List[Dict[str, Any]],
        accuracy_verified: bool = True
    ) -> Response:
        """Create a new response record"""
        response = Response(
            query_id=query_id,
            response_text=response_text,
            source_citations=source_citations,
            accuracy_verified=accuracy_verified
        )
        self.db.add(response)
        await self.db.commit()
        await self.db.refresh(response)
        return response

    async def get_response(self, response_id: str) -> Optional[Response]:
        """Get a response by ID"""
        stmt = select(Response).where(Response.response_id == response_id)
        result = await self.db.execute(stmt)
        response = result.scalar_one_or_none()
        return response

    async def get_response_by_query(self, query_id: str) -> Optional[Response]:
        """Get the response for a specific query"""
        stmt = select(Response).where(Response.query_id == query_id)
        result = await self.db.execute(stmt)
        response = result.scalar_one_or_none()
        return response

    async def validate_response_accuracy(
        self,
        response_text: str,
        source_citations: List[Dict[str, Any]],
        original_query: str
    ) -> bool:
        """Validate that the response is accurate and properly cited"""
        # Basic validation checks:
        # 1. Response should not be empty
        if not response_text or len(response_text.strip()) == 0:
            return False

        # 2. Citations should exist and be properly formatted
        if not source_citations or len(source_citations) == 0:
            return False

        # 3. Each citation should have required fields
        for citation in source_citations:
            if not all(key in citation for key in ["chapter", "section", "similarity_score"]):
                return False

        # 4. Similarity scores should be in valid range
        for citation in source_citations:
            score = citation.get("similarity_score", 0)
            if not (0 <= score <= 1):
                return False

        # More sophisticated validation would go here
        # For now, we'll return True if basic checks pass
        return True

    async def format_response_for_api(
        self,
        response: Response,
        query: Query,
        query_time_ms: Optional[float] = None
    ) -> Dict[str, Any]:
        """Format a response for API output"""
        return {
            "response_id": str(response.response_id),
            "answer": response.response_text,
            "citations": response.source_citations,
            "provenance": f"Based on content from {len(response.source_citations)} sources in the book",
            "query_time": query_time_ms or 0
        }

    async def log_response_generation(
        self,
        session_id: str,
        query_id: str,
        response_id: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> None:
        """Log the response generation for audit purposes"""
        await log_interaction(
            self.db,
            session_id,
            query_id,
            "response_generation",
            metadata or {}
        )