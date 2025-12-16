# Backend: Book Content Embedding Pipeline

This backend component processes markdown files from the `/docs` directory, generates semantic embeddings using Cohere, and stores them in a Qdrant vector database collection named 'physicalai' for the RAG chatbot system.

## Features

- Scans and parses markdown files from the `/docs` directory
- Generates semantic embeddings using Cohere embedding models
- Stores embeddings in Qdrant vector database with metadata
- Configurable via environment variables
- Semantic and fixed-size chunking strategies

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant database (cloud or local instance)

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create a `.env` file with your configuration:
   ```bash
   # Copy the template
   cp .env.example .env
   # Edit the file with your API keys and settings
   ```

3. Place your markdown files in the `/docs` directory

## Configuration

Create a `.env` file with the following variables:

```bash
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
```

## Usage

Run the embedding pipeline:

```bash
# From the backend directory
python src/main.py
```

Or using the run script:

```bash
python scripts/run_embedding_pipeline.py
```

## Pipeline Phases

1. **Content Ingestion**: Scans `/docs` directory and parses markdown files
2. **Content Chunking**: Splits documents into semantically coherent chunks
3. **Embedding Generation**: Creates embeddings using Cohere API
4. **Vector Storage**: Stores embeddings in Qdrant collection 'physicalai'

## Testing

Run the unit tests:

```bash
pytest tests/unit/
```

## Architecture

The entire pipeline is implemented in a single `main.py` file with the following components:

- `DocumentIngestor`: Handles file scanning and markdown parsing
- `Chunker`: Manages content chunking strategies
- `EmbeddingGenerator`: Interfaces with Cohere API
- `VectorStorage`: Manages Qdrant database operations
- `EmbeddingPipeline`: Orchestrates the entire process