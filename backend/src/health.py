"""
Health check endpoints infrastructure
"""
from fastapi import APIRouter
from typing import Dict, Any
from datetime import datetime
from .vector_db import VectorDBService
from .config import settings
import os


router = APIRouter()


async def check_qdrant_status() -> Dict[str, Any]:
    """Check the status of Qdrant vector database"""
    try:
        vector_db = VectorDBService()
        is_healthy = vector_db.health_check()
        return {
            "status": "connected" if is_healthy else "disconnected",
            "timestamp": datetime.utcnow().isoformat()
        }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat()
        }


async def check_neon_status() -> Dict[str, Any]:
    """Check the status of Neon Postgres database"""
    try:
        # Import database here to avoid circular imports
        from .database import engine
        async with engine.begin() as conn:
            # Simple query to test connection
            result = await conn.execute("SELECT 1")
            row = await result.fetchone()
            if row:
                return {
                    "status": "connected",
                    "timestamp": datetime.utcnow().isoformat()
                }
            else:
                return {
                    "status": "disconnected",
                    "timestamp": datetime.utcnow().isoformat()
                }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat()
        }


async def check_models_status() -> Dict[str, Any]:
    """Check the status of LLM models"""
    try:
        from .llm_service import LLMService
        llm_service = LLMService()

        # Test embedding functionality as a basic health check
        test_embedding = await llm_service.embed_text("health check")
        if test_embedding and len(test_embedding) > 0:
            return {
                "status": "available",
                "provider": os.getenv("MODEL_PROVIDER", "gemini"),
                "timestamp": datetime.utcnow().isoformat()
            }
        else:
            return {
                "status": "unavailable",
                "timestamp": datetime.utcnow().isoformat()
            }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat()
        }


@router.get("/health")
async def health_check() -> Dict[str, Any]:
    """Comprehensive health check for all system components"""
    qdrant_status = await check_qdrant_status()
    neon_status = await check_neon_status()
    models_status = await check_models_status()

    # Determine overall status
    all_healthy = (
        qdrant_status["status"] == "connected" and
        neon_status["status"] == "connected" and
        models_status["status"] == "available"
    )

    overall_status = "operational" if all_healthy else "degraded"

    return {
        "status": overall_status,
        "qdrant_status": qdrant_status["status"],
        "neon_status": neon_status["status"],
        "models_status": models_status["status"],
        "timestamp": datetime.utcnow().isoformat(),
        "details": {
            "qdrant": qdrant_status,
            "neon": neon_status,
            "models": models_status
        }
    }


@router.get("/ready")
async def readiness_check() -> Dict[str, Any]:
    """Readiness check to determine if the service is ready to accept traffic"""
    # For readiness, we might have different criteria than health
    # For now, we'll use the same checks as health
    qdrant_status = await check_qdrant_status()
    neon_status = await check_neon_status()

    is_ready = (
        qdrant_status["status"] == "connected" and
        neon_status["status"] == "connected"
    )

    return {
        "status": "ready" if is_ready else "not_ready",
        "qdrant_ready": qdrant_status["status"] == "connected",
        "neon_ready": neon_status["status"] == "connected",
        "timestamp": datetime.utcnow().isoformat()
    }