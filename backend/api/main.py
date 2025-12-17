"""
FastAPI Application for Agent-Based RAG Backend

This module creates the main FastAPI application with proper configuration.
"""
from fastapi import FastAPI
from datetime import datetime
from .routes.chat import router as chat_router


def create_app() -> FastAPI:
    """
    [T022] Create FastAPI application in backend/api/main.py with proper configuration
    """
    app = FastAPI(
        title="Agent-Based RAG Backend API",
        description="API for agent-based retrieval-augmented generation system using OpenAI Agent SDK with LiteLLM models",
        version="1.0.0"
    )

    # Include the chat router
    app.include_router(chat_router, prefix="/api", tags=["chat"])

    @app.get("/health")
    async def health_check():
        return {"status": "healthy", "timestamp": datetime.now().isoformat()}

    return app


# Create the application instance
app = create_app()