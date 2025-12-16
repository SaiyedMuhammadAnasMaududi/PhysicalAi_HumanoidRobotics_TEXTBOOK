#!/usr/bin/env python3
"""
Standalone Database Initialization Script (Alternative Entry Point)
Run from backend directory: python init_db.py
"""

import sys
import os
from pathlib import Path

# Add current directory to path
sys.path.insert(0, str(Path(__file__).parent))

# Load environment variables
from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent / ".env")

import asyncio
from src.db_init import init_database

if __name__ == "__main__":
    print("Starting database initialization...")
    asyncio.run(init_database())
