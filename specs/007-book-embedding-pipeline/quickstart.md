# Quickstart Guide: Book Content Embedding Pipeline

**Date**: 2025-12-16
**Feature**: 007-book-embedding-pipeline

## Prerequisites

- Python 3.11 or higher
- Cohere API key
- Qdrant database (cloud instance or local)
- Book content in `/docs` directory as markdown files

## Setup

### 1. Environment Configuration

Create a `.env` file in the project root:

```bash
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-cluster-url.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=localhost      # Use for local Qdrant instance
QDRANT_PORT=6333          # Use for local Qdrant instance

# Pipeline Configuration
DOCS_PATH=/docs           # Path to markdown files (default: ./docs)
CHUNK_SIZE=512            # Size of text chunks in tokens (default: 512)
CHUNK_OVERLAP=50          # Overlap between chunks in tokens (default: 50)
CHUNKING_STRATEGY=semantic # Options: semantic, fixed (default: semantic)
EMBEDDING_MODEL=embed-multilingual-v3.0 # Cohere model (default: embed-multilingual-v3.0)
BATCH_SIZE=10             # Number of chunks to process in each API call (default: 10)
QDRANT_COLLECTION=physicalai # Qdrant collection name (default: physicalai)
```

### 2. Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Or install specific packages
pip install cohere qdrant-client python-markdown python-dotenv PyYAML
```

### 3. Run the Embedding Pipeline

```bash
# Run with default settings
python backend/src/main.py

# Run with specific configuration
python backend/src/main.py --config .env
```

## Pipeline Phases

### Phase 1: Content Ingestion
- Scans `/docs` directory recursively
- Parses markdown files
- Extracts content and metadata
- Creates document chunks with provenance information

### Phase 2: Content Chunking
- Splits documents into semantically coherent chunks
- Preserves context and meaning
- Maintains chunk sequence information
- Generates content hashes for deduplication

### Phase 3: Embedding Generation
- Sends chunks to Cohere API for embedding
- Handles rate limiting and retries
- Stores embeddings with metadata
- Validates embedding quality

### Phase 4: Vector Storage
- Creates and uploads embeddings to Qdrant database collection named 'physicalai'
- Stores metadata in payload
- Creates indexes for efficient querying
- Performs consistency validation

## Configuration Options

### Chunking Strategies
- `semantic`: Preserves context by chunking at semantic boundaries
- `fixed`: Splits content into fixed-size chunks

### Environment Variables
| Variable | Default | Description |
|----------|---------|-------------|
| `DOCS_PATH` | `./docs` | Directory containing markdown files |
| `CHUNK_SIZE` | `512` | Size of chunks in tokens |
| `CHUNK_OVERLAP` | `50` | Overlap between chunks in tokens |
| `CHUNKING_STRATEGY` | `semantic` | Chunking approach |
| `EMBEDDING_MODEL` | `embed-multilingual-v3.0` | Cohere model to use |
| `BATCH_SIZE` | `10` | API batch size |
| `QDRANT_COLLECTION` | `physicalai` | Qdrant collection name |

## Verification

After running the pipeline, verify:

1. **Qdrant Collection**: Check that the `physicalai` collection exists and contains vectors
2. **Metadata**: Verify that source file information is preserved in the payload
3. **Chunking Quality**: Review a few chunks to ensure semantic coherence
4. **Processing Logs**: Check logs for any errors or warnings

## Troubleshooting

### Common Issues
- **API Rate Limits**: Pipeline includes retry logic; adjust `BATCH_SIZE` if needed
- **Missing API Keys**: Ensure `.env` file contains required keys
- **File Access**: Verify the `DOCS_PATH` directory exists and is readable
- **Qdrant Connection**: Check URL and API key for Qdrant instance

### Performance Tips
- Increase `BATCH_SIZE` for faster processing (respect API limits)
- Use local Qdrant instance for development
- Process large books in smaller batches to manage memory