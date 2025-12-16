"""
Integration test for end-to-end query processing
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from unittest.mock import AsyncMock, patch
from ...src.main import app
from ...src.models import Base
from ...src.database import get_db
from ...src.vector_db import VectorDBService
from ...src.llm_service import LLMService


# Create test database engine
TEST_DATABASE_URL = "sqlite+aiosqlite:///:memory:"

engine = create_async_engine(TEST_DATABASE_URL)
TestingSessionLocal = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
async def setup_test_db():
    """Set up test database with tables"""
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    async with TestingSessionLocal() as session:
        yield session


@pytest.mark.asyncio
async def test_end_to_end_query_processing(client, setup_test_db):
    """
    Test the complete end-to-end query processing flow
    """
    # Mock the external services to avoid actual API calls
    with patch('src.llm_service.LLMService.generate_response', new_callable=AsyncMock) as mock_llm_gen, \
         patch('src.llm_service.LLMService.embed_query', new_callable=AsyncMock) as mock_embed_query, \
         patch('src.vector_db.VectorDBService.search_content', new_callable=AsyncMock) as mock_search:

        # Configure mocks
        mock_embed_query.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]  # Mock embedding
        mock_search.return_value = [
            {
                "content_id": "test-content-1",
                "content_text": "This is a test content snippet related to the query",
                "chapter": "Chapter 1",
                "section": "Section 1.1",
                "similarity_score": 0.85,
                "metadata": {}
            }
        ]
        mock_llm_gen.return_value = "This is a test response based on the retrieved content."

        # First, create a session
        response = client.post("/api/chatbot/session")
        assert response.status_code == 200
        session_data = response.json()
        assert "session_id" in session_data
        session_id = session_data["session_id"]

        # Now test the query endpoint
        query_payload = {
            "query": "What is physical AI?",
            "session_id": session_id
        }

        response = client.post("/api/chatbot/query", json=query_payload)

        # Check that the response is successful
        assert response.status_code == 200

        # Check the response structure
        response_data = response.json()
        assert "response_id" in response_data
        assert "answer" in response_data
        assert "citations" in response_data
        assert "query_time" in response_data

        # Verify the content of the response
        assert response_data["answer"] == "This is a test response based on the retrieved content."
        assert len(response_data["citations"]) == 1
        assert response_data["citations"][0]["chapter"] == "Chapter 1"
        assert response_data["citations"][0]["section"] == "Section 1.1"
        assert response_data["citations"][0]["similarity_score"] == 0.85


@pytest.mark.asyncio
async def test_query_with_selected_text(client, setup_test_db):
    """
    Test query processing with selected text context
    """
    # Mock the external services
    with patch('src.llm_service.LLMService.generate_response', new_callable=AsyncMock) as mock_llm_gen, \
         patch('src.llm_service.LLMService.embed_query', new_callable=AsyncMock) as mock_embed_query, \
         patch('src.vector_db.VectorDBService.search_content', new_callable=AsyncMock) as mock_search:

        # Configure mocks
        mock_embed_query.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]
        mock_search.return_value = [
            {
                "content_id": "test-content-2",
                "content_text": "Selected text content with additional context",
                "chapter": "Chapter 2",
                "section": "Section 2.3",
                "similarity_score": 0.92,
                "metadata": {}
            }
        ]
        mock_llm_gen.return_value = "Response specifically addressing the selected text."

        # Create a session
        response = client.post("/api/chatbot/session")
        assert response.status_code == 200
        session_id = response.json()["session_id"]

        # Test query with selected text
        query_payload = {
            "query": "Explain this concept",
            "selected_text": "Physical AI combines machine learning with physical interaction",
            "session_id": session_id
        }

        response = client.post("/api/chatbot/query", json=query_payload)

        assert response.status_code == 200
        response_data = response.json()

        assert response_data["answer"] == "Response specifically addressing the selected text."
        assert len(response_data["citations"]) == 1
        assert response_data["citations"][0]["chapter"] == "Chapter 2"


@pytest.mark.asyncio
async def test_query_without_session(client, setup_test_db):
    """
    Test query processing without providing a session ID (should create one automatically)
    """
    # Mock the external services
    with patch('src.llm_service.LLMService.generate_response', new_callable=AsyncMock) as mock_llm_gen, \
         patch('src.llm_service.LLMService.embed_query', new_callable=AsyncMock) as mock_embed_query, \
         patch('src.vector_db.VectorDBService.search_content', new_callable=AsyncMock) as mock_search:

        # Configure mocks
        mock_embed_query.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]
        mock_search.return_value = [
            {
                "content_id": "test-content-3",
                "content_text": "General content for the query",
                "chapter": "Chapter 3",
                "section": "Section 3.1",
                "similarity_score": 0.78,
                "metadata": {}
            }
        ]
        mock_llm_gen.return_value = "General response to the query."

        # Test query without session ID (should create session automatically)
        query_payload = {
            "query": "What are the principles of humanoid robotics?"
        }

        response = client.post("/api/chatbot/query", json=query_payload)

        assert response.status_code == 200
        response_data = response.json()

        assert response_data["answer"] == "General response to the query."
        assert "response_id" in response_data
        assert "session_id" in response_data  # Should return new session ID


@pytest.mark.asyncio
async def test_health_endpoint(client):
    """
    Test the health check endpoint
    """
    with patch('src.vector_db.VectorDBService.health_check', return_value=True), \
         patch('src.database.engine', return_value=AsyncMock()), \
         patch('src.llm_service.LLMService.embed_text', new_callable=AsyncMock) as mock_embed:

        mock_embed.return_value = [0.1, 0.2, 0.3]

        response = client.get("/api/health")
        assert response.status_code == 200

        health_data = response.json()
        assert health_data["status"] in ["operational", "degraded"]
        assert "qdrant_status" in health_data
        assert "neon_status" in health_data
        assert "models_status" in health_data
        assert "timestamp" in health_data