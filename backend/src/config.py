"""
Configuration management for different environments
"""
from typing import List, Optional
from pathlib import Path
from pydantic import field_validator, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=str(Path(__file__).parent.parent.parent / ".env"),
        env_file_encoding='utf-8',
        case_sensitive=False,  # Allow case-insensitive env var names
        extra='ignore',
        protected_namespaces=('settings_',)
    )

    # Qdrant Configuration (supports both QUADRANT and QDRANT spellings)
    qdrant_url: Optional[str] = Field(None, validation_alias='QUADRANT_URL')
    qdrant_api_key: Optional[str] = Field(None, validation_alias='QUADRANT_API_KEY')

    # Database Configuration
    neon_db_url: str = Field(
        default="postgresql+asyncpg://user:password@localhost/dbname",
        validation_alias='POSTGRES_URL'
    )

    # LLM Configuration
    model_provider: str = Field(default="gemini")
    gemini_api_key: Optional[str] = None

    # OpenRouter Configuration
    openrouter_api_key: Optional[str] = None
    openrouter_base_url: str = Field(default="https://openrouter.ai/api/v1")

    # Qwen Configuration
    qwen_api_key: Optional[str] = None
    qwen_base_url: str = Field(default="https://dashscope.aliyuncs.com/compatible-mode/v1")

    # Embedding Configuration
    embedding_provider: str = Field(default="gemini")  # gemini, qwen, openrouter, or local
    embedding_model: str = Field(default="all-MiniLM-L6-v2")  # Model to use (SentenceTransformer model for local)

    # LiteLLM Configuration (for OpenRouter and other providers)
    litellm_model: str = Field(default="gpt-3.5-turbo")

    # Application Configuration
    backend_cors_origins: str = Field(
        default='["http://localhost", "http://localhost:3000", "http://localhost:8080"]'
    )

    # FastAPI Configuration
    host: str = Field(default="0.0.0.0")
    port: int = Field(default=8000)
    debug: bool = Field(default=False)

    # Performance Configuration
    max_concurrent_users: int = Field(default=10)
    response_timeout: int = Field(default=30)
    retrieval_top_k: int = Field(default=5)

    @field_validator('neon_db_url', mode='before')
    @classmethod
    def convert_db_url(cls, v):
        """Convert postgresql:// to postgresql+asyncpg:// for asyncpg"""
        if v and isinstance(v, str) and v.startswith("postgresql://") and "+asyncpg" not in v:
            return v.replace("postgresql://", "postgresql+asyncpg://", 1)
        return v

    @field_validator('qdrant_url')
    @classmethod
    def qdrant_url_must_be_set(cls, v):
        if not v:
            raise ValueError('QDRANT_URL (or QUADRANT_URL) must be set in environment variables')
        return v

    @field_validator('qdrant_api_key')
    @classmethod
    def qdrant_api_key_must_be_set(cls, v):
        if not v:
            raise ValueError('QDRANT_API_KEY (or QUADRANT_API_KEY) must be set in environment variables')
        return v

    @field_validator('gemini_api_key')
    @classmethod
    def gemini_api_key_must_be_set(cls, v):
        if not v:
            raise ValueError('GEMINI_API_KEY must be set in environment variables')
        return v

    @field_validator('neon_db_url')
    @classmethod
    def neon_db_url_must_be_set(cls, v):
        # Skip validation if URL was already converted
        if "+asyncpg" in v:
            return v
        if "dbname" in v and "user:password" in v:
            raise ValueError('NEON_DB_URL (or POSTGRES_URL) must be properly configured with real credentials')
        return v

    @field_validator('model_provider')
    @classmethod
    def model_provider_must_be_valid(cls, v):
        if v not in ["gemini", "litellm"]:
            raise ValueError('MODEL_PROVIDER must be either "gemini" or "litellm"')
        return v

    @field_validator('debug', mode='before')
    @classmethod
    def parse_debug(cls, v):
        """Parse debug flag from string"""
        if isinstance(v, str):
            return v.lower() in ('true', '1', 'yes')
        return bool(v)

    def get_cors_origins(self) -> List[str]:
        """Parse CORS origins from JSON string"""
        import json
        if isinstance(self.backend_cors_origins, str):
            return json.loads(self.backend_cors_origins)
        return self.backend_cors_origins


# Create a singleton instance
settings = Settings()
