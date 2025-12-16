"""
Session management service
"""
import uuid
from datetime import datetime, timedelta
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from ..models.session import UserSession
from ..database import get_db


class SessionService:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_session(self, user_id: Optional[str] = None) -> UserSession:
        """Create a new user session"""
        session = UserSession(
            user_id=user_id,
            created_at=datetime.utcnow(),
            last_interaction=datetime.utcnow(),
            active=True
        )
        self.db.add(session)
        await self.db.commit()
        await self.db.refresh(session)
        return session

    async def get_session(self, session_id: str) -> Optional[UserSession]:
        """Get an existing session by ID"""
        stmt = select(UserSession).where(UserSession.session_id == session_id)
        result = await self.db.execute(stmt)
        session = result.scalar_one_or_none()

        if session and session.active:
            # Update last interaction time
            session.last_interaction = datetime.utcnow()
            await self.db.commit()
            return session
        return None

    async def end_session(self, session_id: str) -> bool:
        """End a session by setting it to inactive"""
        stmt = select(UserSession).where(UserSession.session_id == session_id)
        result = await self.db.execute(stmt)
        session = result.scalar_one_or_none()

        if session:
            session.active = False
            await self.db.commit()
            return True
        return False

    async def is_session_active(self, session_id: str) -> bool:
        """Check if a session is active"""
        stmt = select(UserSession).where(
            UserSession.session_id == session_id,
            UserSession.active == True
        )
        result = await self.db.execute(stmt)
        session = result.scalar_one_or_none()
        return session is not None