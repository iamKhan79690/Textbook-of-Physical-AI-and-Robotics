"""
Configuration module using pydantic-settings for type-safe environment variable management.
"""
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI Configuration
    openai_api_key: str
    openai_model: str = "gpt-4o-mini"  # Faster model
    openai_embedding_model: str = "text-embedding-3-small"
    openai_max_tokens: int = 300  # Reduced for faster responses
    openai_temperature: float = 0.3  # Lower for more focused, faster responses

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook"

    # Neon Serverless Postgres
    database_url: str
    db_pool_min_size: int = 2
    db_pool_max_size: int = 10

    # Rate Limiting
    rate_limit_per_minute: int = 30

    # CORS Origins
    cors_origins: str = "https://devhammad0.github.io,http://localhost:3000"

    # ChatKit Configuration
    chatkit_domain_key: str = "domain_pk_local_dev"

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )


# Global settings instance
settings = Settings()
