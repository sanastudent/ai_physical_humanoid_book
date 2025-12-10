"""
Health check configuration settings

Validates and loads environment variables required for health checks
"""
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, model_validator
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables BEFORE instantiating settings
load_dotenv()


class HealthCheckSettings(BaseSettings):
    """
    Configuration for health check system.
    Loads from environment variables with validation.
    """
    model_config = SettingsConfigDict(env_file='.env', extra='ignore')

    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    qdrant_api_key: Optional[str] = None

    # Embeddings Provider Configuration
    anthropic_api_key: Optional[str] = None
    openai_api_key: Optional[str] = None
    google_api_key: Optional[str] = None

    # Health Check Configuration
    health_check_timeout: int = Field(default=30, ge=5, le=300, description="Global timeout in seconds")
    component_timeout: int = Field(default=5, ge=1, le=30, description="Per-component timeout in seconds")
    test_collection_name: str = "book_embeddings_test"

    @model_validator(mode='after')
    def validate_qdrant_connection(self):
        """Ensure either URL or host+port is provided"""
        if not self.qdrant_url and not self.qdrant_host:
            raise ValueError("Either QDRANT_URL or QDRANT_HOST must be set")
        return self


# Global settings instance
try:
    settings = HealthCheckSettings()
except Exception as e:
    # If settings fail to load, create a minimal instance with defaults
    # This prevents import errors but health checks will report configuration issues
    settings = HealthCheckSettings(
        qdrant_host=os.getenv("QDRANT_HOST", "localhost"),
        qdrant_port=int(os.getenv("QDRANT_PORT", "6333"))
    )
