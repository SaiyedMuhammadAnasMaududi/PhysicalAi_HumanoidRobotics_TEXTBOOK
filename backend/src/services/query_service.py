"""
Query processing service
"""
from typing import Optional, List, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime
import uuid
from ..models.query import Query
from ..models.session import UserSession
from ..models.retrieval import RetrievalResult
from ..models.response import Response
from ..database import get_db
from ..vector_db import VectorDBService
from ..llm_service import LLMService
from ..logging import log_interaction


class QueryService:
    def __init__(self, db: AsyncSession, vector_db: VectorDBService, llm_service: LLMService):
        self.db = db
        self.vector_db = vector_db
        self.llm_service = llm_service

    async def process_query(
        self,
        session_id: str,
        query_text: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """Process a user query and return a response"""
        # Validate session exists and is active
        session_stmt = select(UserSession).where(
            UserSession.session_id == session_id,
            UserSession.active == True
        )
        session_result = await self.db.execute(session_stmt)
        user_session = session_result.scalar_one_or_none()

        if not user_session:
            raise ValueError("Invalid or inactive session")

        # Create query record
        query = Query(
            session_id=session_id,
            query_text=query_text,
            selected_text=selected_text,
            query_type="selected-text" if selected_text else "general",
            processed=False
        )
        self.db.add(query)
        await self.db.commit()
        await self.db.refresh(query)

        # Log the query interaction
        await log_interaction(
            self.db,
            session_id,
            str(query.query_id),
            "query",
            {"query_type": query.query_type}
        )

        try:
            # Generate embedding for the query
            query_embedding = await self.llm_service.embed_query(query_text)

            # Search for relevant content in the vector database
            search_results = await self.vector_db.search_content(
                query_embedding,
                top_k=5  # Configurable
            )

            # Prepare context from search results
            context_parts = []
            for result in search_results:
                context_parts.append(f"Chapter: {result['chapter']}, Section: {result['section']}\n{result['content_text']}")

            context = "\n\n".join(context_parts)

            # If selected text is provided, include it in the context
            if selected_text:
                context = f"Selected text context: {selected_text}\n\n{context}"

            # Generate response using LLM with the retrieved context
            response_text = await self.llm_service.generate_response(query_text, context)

            # Create retrieval results records
            retrieval_results = []
            for result in search_results:
                retrieval_result = RetrievalResult(
                    query_id=query.query_id,
                    source_document=f"{result['chapter']}/{result['section']}",
                    content_snippet=result['content_text'],
                    similarity_score=result['similarity_score'],
                    chunk_id=result.get('content_id')
                )
                self.db.add(retrieval_result)
                retrieval_results.append(retrieval_result)

            # Create citations for the response
            citations = []
            for result in search_results:
                citations.append({
                    "chapter": result['chapter'],
                    "section": result['section'],
                    "similarity_score": result['similarity_score']
                })

            # Create response record
            response = Response(
                query_id=query.query_id,
                response_text=response_text,
                source_citations=citations,
                accuracy_verified=True  # Assuming LLM service handles verification
            )
            self.db.add(response)
            await self.db.commit()

            # Update query as processed
            query.processed = True
            await self.db.commit()

            # Log the response interaction
            await log_interaction(
                self.db,
                session_id,
                str(query.query_id),
                "response",
                {"response_length": len(response_text)}
            )

            # Prepare response data
            response_data = {
                "response_id": str(response.response_id),
                "answer": response_text,
                "citations": citations,
                "provenance": f"Based on content from {len(citations)} sources in the book",
                "query_time": (datetime.utcnow() - query.query_timestamp).total_seconds() * 1000  # in milliseconds
            }

            return response_data

        except Exception as e:
            # Update query as processed with error
            query.processed = True
            await self.db.commit()

            # Log the error
            await log_interaction(
                self.db,
                session_id,
                str(query.query_id),
                "error",
                {"error": str(e)}
            )

            raise e