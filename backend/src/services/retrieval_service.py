"""
Basic text retrieval from Qdrant based on user query
"""
from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from ..vector_db import VectorDBService
from ..llm_service import LLMService
from ..models.retrieval import RetrievalResult
from ..models.query import Query


class RetrievalService:
    """Service for retrieving relevant content from the vector database based on user queries"""

    def __init__(self, vector_db: VectorDBService, llm_service: LLMService):
        self.vector_db = vector_db
        self.llm_service = llm_service

    async def retrieve_content(
        self,
        query_text: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from the vector database based on the query
        """
        # Generate embedding for the query
        query_embedding = await self.llm_service.embed_query(query_text)

        # Search for relevant content in the vector database
        search_results = await self.vector_db.search_content(
            query_embedding,
            top_k=top_k
        )

        return search_results

    async def retrieve_content_with_selected_text(
        self,
        query_text: str,
        selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content, giving priority to content related to selected text if provided
        """
        if selected_text:
            # If selected text is provided, search for content related to both the query and selected text
            # First, embed the selected text
            selected_embedding = await self.llm_service.embed_text(selected_text)
            query_embedding = await self.llm_service.embed_query(query_text)

            # For now, we'll use the selected text embedding as the primary search
            # In a more advanced implementation, we could combine both embeddings
            search_results = await self.vector_db.search_content(
                selected_embedding,
                top_k=top_k
            )
        else:
            # Standard search using just the query
            query_embedding = await self.llm_service.embed_query(query_text)
            search_results = await self.vector_db.search_content(
                query_embedding,
                top_k=top_k
            )

        return search_results

    async def store_retrieval_results(
        self,
        db: AsyncSession,
        query_id: str,
        search_results: List[Dict[str, Any]]
    ) -> List[RetrievalResult]:
        """
        Store retrieval results in the database
        """
        from sqlalchemy import insert

        retrieval_entities = []
        for result in search_results:
            retrieval_result = RetrievalResult(
                query_id=query_id,
                source_document=f"{result['chapter']}/{result['section']}",
                content_snippet=result['content_text'],
                similarity_score=result['similarity_score'],
                chunk_id=result.get('content_id')
            )
            db.add(retrieval_result)
            retrieval_entities.append(retrieval_result)

        await db.commit()
        return retrieval_entities

    async def retrieve_and_store_content(
        self,
        db: AsyncSession,
        query_id: str,
        query_text: str,
        selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Retrieve content and store the results in the database
        """
        # Retrieve content from vector database
        search_results = await self.retrieve_content_with_selected_text(
            query_text,
            selected_text,
            top_k
        )

        # Store retrieval results in database
        await self.store_retrieval_results(db, query_id, search_results)

        return search_results

    async def get_retrieval_results_for_query(
        self,
        db: AsyncSession,
        query_id: str
    ) -> List[RetrievalResult]:
        """
        Get previously stored retrieval results for a specific query
        """
        from sqlalchemy.future import select

        stmt = select(RetrievalResult).where(RetrievalResult.query_id == query_id)
        result = await db.execute(stmt)
        retrieval_results = result.scalars().all()
        return retrieval_results

    async def get_content_snippets(
        self,
        query_text: str,
        selected_text: Optional[str] = None,
        top_k: int = 5
    ) -> List[str]:
        """
        Get just the content snippets from retrieval results
        """
        search_results = await self.retrieve_content_with_selected_text(
            query_text,
            selected_text,
            top_k
        )

        return [result['content_text'] for result in search_results]