"""
Main FastAPI application for the RAG Chatbot
"""
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.chatbot import router as chatbot_router
from .health import router as health_router
from .config import settings
from .database import init_db, close_db
from .middleware.sanitization import InputSanitizationMiddleware
from .middleware.rate_limit import security_middleware


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events"""
    # Startup
    print("Initializing database...")
    await init_db()
    print("Database initialized successfully")

    yield  # Application runs here

    # Shutdown
    print("Closing database connections...")
    await close_db()
    print("Database connections closed")


# Create FastAPI app with lifespan
app = FastAPI(
    title="RAG Chatbot API for Physical AI & Humanoid Robotics Book",
    description="API for the integrated RAG chatbot system",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.backend_cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add security middleware (rate limiting and headers)
app.middleware("http")(security_middleware)

# Add input sanitization middleware
app.middleware("http")(InputSanitizationMiddleware())

# Include API routers
app.include_router(chatbot_router, prefix="/api/chatbot", tags=["chatbot"])
app.include_router(health_router, prefix="/api", tags=["health"])

# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "RAG Chatbot API for Physical AI & Humanoid Robotics Book",
        "version": "1.0.0",
        "status": "operational"
    }

# Health check at root as well
@app.get("/health")
async def root_health():
    from .health import health_check
    return await health_check()