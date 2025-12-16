#!/usr/bin/env python3
"""
Database Initialization Script
Creates all database tables for the RAG Chatbot system
"""

import sys
import asyncio
from pathlib import Path

# Add the backend directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config import settings
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text

# Import the shared Base first
from src.models.base import Base

# Import all models to register them with Base
from src.models.session import UserSession
from src.models.query import Query
from src.models.retrieval import RetrievalResult
from src.models.response import Response
from src.models.audit import AuditLog
from src.models.content import BookContent


async def init_database():
    """Initialize the database schema"""

    print("=" * 60)
    print("🔧 RAG Chatbot Database Initialization")
    print("=" * 60)

    # Get database URL from config
    db_url = settings.neon_db_url

    # Convert postgresql:// to postgresql+asyncpg:// if needed
    if db_url.startswith("postgresql://"):
        db_url = db_url.replace("postgresql://", "postgresql+asyncpg://", 1)
    elif not db_url.startswith("postgresql+asyncpg://"):
        print(f"⚠️  Warning: Database URL should start with 'postgresql+asyncpg://'")
        print(f"   Current URL: {db_url[:30]}...")

    # Remove psycopg2-specific query params that asyncpg doesn't support
    # asyncpg handles SSL and other options via connect_args instead
    from urllib.parse import urlparse, parse_qs, urlencode, urlunparse

    parsed = urlparse(db_url)
    query_params = parse_qs(parsed.query)

    # Extract sslmode before removing it
    sslmode = query_params.pop('sslmode', ['require'])[0] if 'sslmode' in query_params else 'require'
    ssl_require = sslmode in ["require", "verify-ca", "verify-full"]

    # Remove other psycopg2-specific params that asyncpg doesn't support
    unsupported_params = ['sslmode', 'channel_binding', 'sslrootcert', 'sslcert', 'sslkey']
    for param in unsupported_params:
        query_params.pop(param, None)

    # Rebuild URL without unsupported params
    new_query = urlencode({k: v[0] for k, v in query_params.items()}, doseq=True)
    db_url = urlunparse((parsed.scheme, parsed.netloc, parsed.path, parsed.params, new_query, parsed.fragment))

    print(f"\n📊 Database URL: {db_url[:50]}...")
    print(f"📦 Creating tables...")
    print(f"🔒 SSL: {'Required' if ssl_require else 'Not Required'}")

    try:
        # Create async engine with SSL configuration
        connect_args = {}
        if ssl_require:
            connect_args["ssl"] = "require"

        engine = create_async_engine(
            db_url,
            echo=True,  # Show SQL statements
            connect_args=connect_args,
        )

        # Create all tables
        async with engine.begin() as conn:
            # Use the shared Base to create all registered tables
            await conn.run_sync(Base.metadata.create_all)

        print("\n✅ Database tables created successfully!")
        print("\nCreated tables:")
        print("  • user_sessions - User session tracking")
        print("  • queries - User query history")
        print("  • retrieval_results - Vector search results")
        print("  • responses - Generated responses")
        print("  • audit_logs - System audit trail")
        print("  • book_contents - Indexed book content")

        # Verify tables
        async with engine.connect() as conn:
            result = await conn.execute(
                text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public'")
            )
            tables = [row[0] for row in result]
            print(f"\n📋 Verified {len(tables)} tables in database:")
            for table in sorted(tables):
                print(f"  ✓ {table}")

        await engine.dispose()

        print("\n" + "=" * 60)
        print("🎉 Database initialization complete!")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ Error initializing database: {e}")
        print("\nTroubleshooting:")
        print("  1. Check that POSTGRES_URL is set correctly in .env")
        print("  2. Verify Neon database is accessible")
        print("  3. Ensure database user has CREATE TABLE permissions")
        print("  4. Check connection string format:")
        print("     postgresql://user:password@host.neon.tech/dbname?sslmode=require")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(init_database())
