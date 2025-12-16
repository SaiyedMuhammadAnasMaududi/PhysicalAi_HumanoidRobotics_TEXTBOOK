"""
Unit tests for retrieval service
"""
import pytest
from unittest.mock import AsyncMock, MagicMock
from sqlalchemy.ext.asyncio import AsyncSession
from ...src.services.retrieval_service import RetrievalService
from ...src.vector_db import VectorDBService
from ...src.llm_service import LLMService


@pytest.fixture
def mock_vector_db():
    """Mock vector database service"""
    return AsyncMock(spec=VectorDBService)


@pytest.fixture
def mock_llm_service():
    """Mock LLM service"""
    return AsyncMock(spec=LLMService)


@pytest.fixture
def mock_db_session():
    """Mock database session"""
    return AsyncMock(spec=AsyncSession)


@pytest.fixture
def retrieval_service(mock_vector_db, mock_llm_service):
    """Create a retrieval service instance with mocked dependencies"""
    return RetrievalService(mock_vector_db, mock_llm_service)


@pytest.mark.asyncio
async def test_retrieve_content_basic(retrieval_service, mock_vector_db, mock_llm_service):
    """Test basic content retrieval"""
    # Configure mocks
    mock_llm_service.embed_query.return_value = [0.1, 0.2, 0.3]
    expected_results = [
        {
            "content_id": "content-1",
            "content_text": "Test content 1",
            "chapter": "Chapter 1",
            "section": "Section 1.1",
            "similarity_score": 0.85
        }
    ]
    mock_vector_db.search_content.return_value = expected_results

    # Call the method
    results = await retrieval_service.retrieve_content("test query", top_k=1)

    # Verify the results
    assert results == expected_results
    mock_llm_service.embed_query.assert_called_once_with("test query")
    mock_vector_db.search_content.assert_called_once_with([0.1, 0.2, 0.3], top_k=1)


@pytest.mark.asyncio
async def test_retrieve_content_with_selected_text(retrieval_service, mock_vector_db, mock_llm_service):
    """Test content retrieval with selected text context"""
    # Configure mocks
    mock_llm_service.embed_text.return_value = [0.4, 0.5, 0.6]
    expected_results = [
        {
            "content_id": "content-2",
            "content_text": "Selected text content",
            "chapter": "Chapter 2",
            "section": "Section 2.1",
            "similarity_score": 0.92
        }
    ]
    mock_vector_db.search_content.return_value = expected_results

    # Call the method
    results = await retrieval_service.retrieve_content_with_selected_text(
        "test query",
        selected_text="selected text",
        top_k=1
    )

    # Verify the results
    assert results == expected_results
    mock_llm_service.embed_text.assert_called_once_with("selected text")
    mock_vector_db.search_content.assert_called_once_with([0.4, 0.5, 0.6], top_k=1)


@pytest.mark.asyncio
async def test_retrieve_content_without_selected_text(retrieval_service, mock_vector_db, mock_llm_service):
    """Test content retrieval without selected text (should use query embedding)"""
    # Configure mocks
    mock_llm_service.embed_query.return_value = [0.7, 0.8, 0.9]
    expected_results = [
        {
            "content_id": "content-3",
            "content_text": "Query-based content",
            "chapter": "Chapter 3",
            "section": "Section 3.1",
            "similarity_score": 0.78
        }
    ]
    mock_vector_db.search_content.return_value = expected_results

    # Call the method
    results = await retrieval_service.retrieve_content_with_selected_text(
        "test query",
        selected_text=None,
        top_k=1
    )

    # Verify the results
    assert results == expected_results
    mock_llm_service.embed_query.assert_called_once_with("test query")
    mock_vector_db.search_content.assert_called_once_with([0.7, 0.8, 0.9], top_k=1)


@pytest.mark.asyncio
async def test_store_retrieval_results(retrieval_service, mock_db_session):
    """Test storing retrieval results in the database"""
    from ...src.models.retrieval import RetrievalResult

    # Sample search results
    search_results = [
        {
            "content_id": "content-1",
            "content_text": "Test content 1",
            "chapter": "Chapter 1",
            "section": "Section 1.1",
            "similarity_score": 0.85
        }
    ]

    # Call the method
    results = await retrieval_service.store_retrieval_results(
        mock_db_session, "query-123", search_results
    )

    # Verify that the database session was used correctly
    assert len(results) == 1
    assert isinstance(results[0], RetrievalResult)
    mock_db_session.commit.assert_called_once()


@pytest.mark.asyncio
async def test_retrieve_and_store_content(retrieval_service, mock_vector_db, mock_llm_service, mock_db_session):
    """Test retrieving content and storing the results"""
    # Configure mocks
    mock_llm_service.embed_query.return_value = [0.1, 0.2, 0.3]
    search_results = [
        {
            "content_id": "content-4",
            "content_text": "Test content 4",
            "chapter": "Chapter 4",
            "section": "Section 4.1",
            "similarity_score": 0.88
        }
    ]
    mock_vector_db.search_content.return_value = search_results

    # Call the method
    results = await retrieval_service.retrieve_and_store_content(
        mock_db_session,
        "query-456",
        "test query",
        None,
        1
    )

    # Verify the results
    assert results == search_results
    mock_llm_service.embed_query.assert_called_once_with("test query")
    mock_vector_db.search_content.assert_called_once_with([0.1, 0.2, 0.3], top_k=1)


@pytest.mark.asyncio
async def test_get_content_snippets(retrieval_service, mock_vector_db, mock_llm_service):
    """Test getting just content snippets"""
    # Configure mocks
    mock_llm_service.embed_query.return_value = [0.1, 0.2, 0.3]
    search_results = [
        {
            "content_id": "content-5",
            "content_text": "First snippet",
            "chapter": "Chapter 5",
            "section": "Section 5.1",
            "similarity_score": 0.75
        },
        {
            "content_id": "content-6",
            "content_text": "Second snippet",
            "chapter": "Chapter 6",
            "section": "Section 6.1",
            "similarity_score": 0.82
        }
    ]
    mock_vector_db.search_content.return_value = search_results

    # Call the method
    snippets = await retrieval_service.get_content_snippets("test query", None, 2)

    # Verify the results
    assert snippets == ["First snippet", "Second snippet"]


@pytest.mark.asyncio
async def test_retrieve_content_with_filters(retrieval_service, mock_vector_db, mock_llm_service):
    """Test content retrieval with filters (though filters are not currently implemented in the service)"""
    # Configure mocks
    mock_llm_service.embed_query.return_value = [0.1, 0.2, 0.3]
    expected_results = [
        {
            "content_id": "content-7",
            "content_text": "Filtered content",
            "chapter": "Chapter 7",
            "section": "Section 7.1",
            "similarity_score": 0.90
        }
    ]
    mock_vector_db.search_content.return_value = expected_results

    # Call the method with filters (though filters are currently ignored in implementation)
    results = await retrieval_service.retrieve_content(
        "test query",
        top_k=1,
        filters={"chapter": "Chapter 7"}
    )

    # Verify the results
    assert results == expected_results
    # Note: In the current implementation, filters are not used, so we just verify the embedding and search were called
    mock_llm_service.embed_query.assert_called_once_with("test query")
    mock_vector_db.search_content.assert_called_once_with([0.1, 0.2, 0.3], top_k=1)