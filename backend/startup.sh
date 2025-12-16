#!/bin/bash
# Startup script for the RAG Chatbot backend

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d "env" ]; then
    source env/bin/activate
fi

# Install dependencies if requirements.txt exists
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
fi

# Run the FastAPI application
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload