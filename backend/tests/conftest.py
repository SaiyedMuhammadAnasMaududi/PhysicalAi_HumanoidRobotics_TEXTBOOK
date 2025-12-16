"""
Pytest configuration and fixtures
"""
import pytest
from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine
from ..src.models import Base
import os

# Create test database engine
DATABASE_URL = os.getenv("NEON_DB_URL", "postgresql+asyncpg://user:password@localhost/dbname")
TEST_DATABASE_URL = f"{DATABASE_URL}_test"

@pytest.fixture(scope="session")
def engine():
    engine = create_async_engine(TEST_DATABASE_URL)
    yield engine
    engine.dispose()

@pytest.fixture(scope="session")
async def tables(engine):
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    yield
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)