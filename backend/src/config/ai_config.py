"""
AI Model Configuration for Primary/Fallback Routing

This module provides configuration for AI model routing with primary and fallback options
as required by FR-016 and FR-017 in the specification.
"""
import os
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings


class AIModelConfig(BaseModel):
    """Configuration for a single AI model"""
    name: str
    provider: str
    model: str
    api_key: str
    base_url: Optional[str] = None
    temperature: float = 0.7
    max_tokens: int = 2048


class AIConfig(BaseSettings):
    """Main AI configuration with primary and fallback models"""
    # Primary model configuration
    primary_model_name: str = Field(default="gemini-pro", description="Primary AI model name")
    primary_model_provider: str = Field(default="google", description="Primary model provider")
    primary_api_key: str = Field(default="", description="Primary API key")

    # Fallback model configuration
    fallback_model_name: str = Field(default="gemini-1.0-pro", description="Fallback AI model name")
    fallback_model_provider: str = Field(default="google", description="Fallback model provider")
    fallback_api_key: str = Field(default="", description="Fallback API key")

    # Routing configuration
    enable_fallback: bool = Field(default=True, description="Enable fallback routing")
    fallback_timeout: int = Field(default=30, description="Timeout in seconds before fallback")
    max_retries: int = Field(default=2, description="Max retries before fallback")

    # Model-specific settings
    temperature: float = Field(default=0.7, description="Default temperature for generation")
    max_tokens: int = Field(default=2048, description="Default max tokens for generation")

    class Config:
        env_prefix = "AI_"
        case_sensitive = False

    def get_primary_config(self) -> AIModelConfig:
        """Get primary model configuration"""
        return AIModelConfig(
            name=self.primary_model_name,
            provider=self.primary_model_provider,
            model=self.primary_model_name,
            api_key=self.primary_api_key,
            temperature=self.temperature,
            max_tokens=self.max_tokens
        )

    def get_fallback_config(self) -> Optional[AIModelConfig]:
        """Get fallback model configuration if enabled"""
        if not self.enable_fallback or not self.fallback_api_key:
            return None

        return AIModelConfig(
            name=self.fallback_model_name,
            provider=self.fallback_model_provider,
            model=self.fallback_model_name,
            api_key=self.fallback_api_key,
            temperature=self.temperature,
            max_tokens=self.max_tokens
        )

    def get_model_configs(self) -> Dict[str, AIModelConfig]:
        """Get all available model configurations"""
        configs = {"primary": self.get_primary_config()}
        fallback_config = self.get_fallback_config()
        if fallback_config:
            configs["fallback"] = fallback_config
        return configs


# Global configuration instance
ai_config = AIConfig()