"""
Basic logging and error handling infrastructure
"""
import logging
from datetime import datetime
import os
from typing import Optional
from .models.audit import AuditLog
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import insert

# Configure root logger
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Create logger for the application
logger = logging.getLogger(__name__)

# Set specific log levels for different components
logging.getLogger('sqlalchemy').setLevel(logging.WARNING)
logging.getLogger('uvicorn').setLevel(logging.INFO)

async def log_interaction(
    db: AsyncSession,
    session_id: str,
    query_id: Optional[str] = None,
    action: str = 'unknown',
    metadata: Optional[dict] = None
) -> None:
    """
    Log interaction for audit trail
    """
    try:
        audit_log = AuditLog(
            session_id=session_id,
            query_id=query_id,
            action=action,
            metadata=metadata or {}
        )
        db.add(audit_log)
        await db.commit()
    except Exception as e:
        # Don't let logging errors break the main flow
        logger.error(f"Failed to log interaction: {e}")

def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name
    """
    return logging.getLogger(name)