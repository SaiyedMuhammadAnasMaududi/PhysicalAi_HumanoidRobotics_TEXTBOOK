"""
Basic authentication infrastructure
"""
from datetime import datetime, timedelta
from typing import Optional
import jwt
from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from passlib.context import CryptContext
from pydantic import BaseModel
import os
from .config import settings


# Initialize password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()


class TokenData(BaseModel):
    username: Optional[str] = None


class AuthHandler:
    """Handles authentication operations"""

    def __init__(self):
        # Use a secure secret key from environment or generate a default
        self.secret = os.getenv("JWT_SECRET_KEY", settings.qdrant_api_key[:25] if settings.qdrant_api_key else "default_secret_key_for_development")
        self.algorithm = "HS256"
        self.access_token_expire_minutes = 30

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against a hashed password"""
        return pwd_context.verify(plain_password, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """Generate a hash for a plain password"""
        return pwd_context.hash(password)

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create an access token with optional expiration"""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=self.access_token_expire_minutes)

        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, self.secret, algorithm=self.algorithm)
        return encoded_jwt

    def decode_access_token(self, token: str) -> Optional[TokenData]:
        """Decode an access token and return token data"""
        try:
            payload = jwt.decode(token, self.secret, algorithms=[self.algorithm])
            username: str = payload.get("sub")
            if username is None:
                return None
            token_data = TokenData(username=username)
            return token_data
        except jwt.PyJWTError:
            return None

    async def verify_token(
        self,
        credentials: HTTPAuthorizationCredentials = Depends(security)
    ) -> Optional[TokenData]:
        """Verify the provided token"""
        token = credentials.credentials
        token_data = self.decode_access_token(token)

        if token_data is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return token_data


# Create a global instance
auth_handler = AuthHandler()


# Convenience functions for common auth operations
def get_current_user(token_data: TokenData = Depends(auth_handler.verify_token)) -> TokenData:
    """Dependency to get the current authenticated user"""
    return token_data


def create_user_token(username: str) -> str:
    """Create a token for a user"""
    return auth_handler.create_access_token(data={"sub": username})


def verify_user_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a user's password"""
    return auth_handler.verify_password(plain_password, hashed_password)


def hash_user_password(password: str) -> str:
    """Hash a user's password"""
    return auth_handler.get_password_hash(password)


# For the RAG Chatbot system, we may not need full user authentication
# since it's primarily focused on book content access
# This auth module provides the infrastructure if needed in the future