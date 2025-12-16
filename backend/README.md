# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics book RAG chatbot.

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Environment

Copy the root `.env` file or ensure these variables are set:

```bash
QUADRANT_URL=https://your-cluster.qdrant.io
QUADRANT_API_KEY=your_api_key
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
GEMINI_API_KEY=your_gemini_key
```

### 3. Initialize Database

**Windows:**
```powershell
cd backend
python init_db.py
```

**Linux/Mac:**
```bash
cd backend
python3 init_db.py
```

This creates all necessary database tables in your Neon Postgres instance.

### 4. Index Book Content

```bash
cd backend
python scripts/index_content.py
```

This indexes your book content into Qdrant vector database.

### 5. Run Development Server

```bash
cd backend
uvicorn src.main:app --reload
```

Visit http://localhost:8000/docs for API documentation.

## Project Structure

```
backend/
├── src/
│   ├── __init__.py          # Package initialization
│   ├── main.py              # FastAPI application
│   ├── config.py            # Configuration management
│   ├── database.py          # Database connection pool
│   ├── db_init.py           # Database initialization module
│   ├── vector_db.py         # Qdrant client
│   ├── llm_service.py       # LLM integration
│   ├── api/                 # API endpoints
│   ├── models/              # SQLAlchemy models
│   ├── services/            # Business logic
│   ├── middleware/          # Middleware (CORS, rate limiting, etc.)
│   └── schemas/             # Pydantic schemas
├── scripts/
│   └── index_content.py     # Content indexing script
├── tests/                   # Test suite
├── init_db.py              # Database init entry point
├── init_db.bat             # Windows batch file
├── requirements.txt         # Python dependencies
├── Dockerfile              # Docker configuration
├── railway.json            # Railway deployment config
├── render.yaml             # Render deployment config
└── vercel.json             # Vercel deployment config
```

## API Endpoints

### Health Check
```bash
GET /health
GET /api/health
```

### Query Chatbot
```bash
POST /api/chatbot/query
Content-Type: application/json

{
  "query": "What is Physical AI?",
  "selected_text": "optional highlighted text",
  "session_id": "optional session id"
}
```

### Session Management
```bash
POST /api/chatbot/session
Content-Type: application/json

{
  "user_id": "optional user identifier"
}
```

## Development

### Run Tests
```bash
pytest
```

### Check API Documentation
Visit http://localhost:8000/docs for interactive Swagger UI

### View Logs
Logs are written to stdout and include request/response details.

## Deployment

See the main `DEPLOYMENT_QUICKSTART.md` for deployment instructions.

### Quick Deploy to Railway
```bash
cd ..
./scripts/deploy-backend.sh railway
```

### Quick Deploy to Render
Upload `render.yaml` via Render dashboard.

### Quick Deploy to Vercel
```bash
cd ..
./scripts/deploy-backend.sh vercel
```

## Troubleshooting

### Module Not Found Error
Make sure you're in the `backend` directory and use:
```bash
python init_db.py
```
Instead of:
```bash
python -m src.db_init
```

### Database Connection Error
1. Check `POSTGRES_URL` in `.env` file
2. Verify Neon database is accessible
3. Ensure connection string includes `?sslmode=require`

### Qdrant Connection Error
1. Verify `QUADRANT_URL` and `QUADRANT_API_KEY`
2. Check Qdrant Cloud dashboard
3. Ensure cluster is active

### Import Errors
Install all dependencies:
```bash
pip install -r requirements.txt
```

## Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `QUADRANT_URL` | Qdrant cluster URL | Yes |
| `QUADRANT_API_KEY` | Qdrant API key | Yes |
| `POSTGRES_URL` | Neon database connection string | Yes |
| `GEMINI_API_KEY` | Google Gemini API key | Yes |
| `MODEL_PROVIDER` | LLM provider (gemini/litellm) | No (default: gemini) |
| `BACKEND_CORS_ORIGINS` | Allowed CORS origins (JSON array) | No |
| `DEBUG` | Enable debug mode | No (default: false) |
| `MAX_CONCURRENT_USERS` | Max concurrent users | No (default: 10) |

## License

See main project LICENSE file.
