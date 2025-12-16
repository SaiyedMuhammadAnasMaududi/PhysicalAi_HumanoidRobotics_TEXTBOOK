#!/bin/bash

# Setup script for the embedding pipeline environment

set -e  # Exit on any error

echo "Setting up environment for embedding pipeline..."

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Check if .env file exists, create template if not
if [ ! -f ".env" ]; then
    echo "Creating .env template..."
    cat > .env << EOF
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-cluster-url.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=localhost      # Use for local Qdrant instance
QDRANT_PORT=6333          # Use for local Qdrant instance

# Pipeline Configuration
DOCS_PATH=./docs           # Path to markdown files (default: ./docs)
CHUNK_SIZE=512             # Size of text chunks in tokens (default: 512)
CHUNK_OVERLAP=50           # Overlap between chunks in tokens (default: 50)
CHUNKING_STRATEGY=semantic # Options: semantic, fixed (default: semantic)
EMBEDDING_MODEL=embed-multilingual-v3.0 # Cohere model (default: embed-multilingual-v3.0)
BATCH_SIZE=10              # Number of chunks to process in each API call (default: 10)
QDRANT_COLLECTION=physicalai # Qdrant collection name (default: physicalai)
EOF
    echo "Created .env template. Please update with your actual API keys."
fi

# Create docs directory if it doesn't exist
if [ ! -d "docs" ]; then
    mkdir -p docs
    echo "Created docs directory. Place your markdown files here."
fi

echo "Environment setup complete!"
echo "To run the pipeline:"
echo "  1. Update .env with your API keys"
echo "  2. Place markdown files in the docs/ directory"
echo "  3. Run: python src/main.py"