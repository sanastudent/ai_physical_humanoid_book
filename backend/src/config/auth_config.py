"""
BetterAuth Configuration for Neon DB
Handles authentication configuration, password hashing, and session management
"""

import os
from datetime import datetime, timedelta
from typing import Optional
from passlib.context import CryptContext
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Password hashing configuration (bcrypt with salt rounds=10)
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class AuthConfig:
    """Authentication configuration for BetterAuth-compatible backend"""

    # Database configuration
    NEON_DB_URL: str = os.getenv("NEON_DB_URL", "")

    # BetterAuth secret for session encryption
    SECRET_KEY: str = os.getenv("BETTERAUTH_SECRET", "")

    # Session configuration
    SESSION_EXPIRE_HOURS: int = 24  # Default session expiration (24 hours)
    SESSION_EXPIRE_DAYS_REMEMBER: int = 30  # Extended session with "remember me"

    # Password validation rules
    PASSWORD_MIN_LENGTH: int = 8
    PASSWORD_REQUIRE_UPPERCASE: bool = True
    PASSWORD_REQUIRE_LOWERCASE: bool = True
    PASSWORD_REQUIRE_DIGIT: bool = True

    # Cookie configuration
    COOKIE_NAME: str = "session_token"
    COOKIE_HTTPONLY: bool = True  # Prevent JavaScript access
    COOKIE_SECURE: bool = True  # HTTPS only (disable in development if needed)
    COOKIE_SAMESITE: str = "lax"  # CSRF protection

    @classmethod
    def validate_config(cls) -> None:
        """Validate required configuration"""
        if not cls.NEON_DB_URL:
            raise ValueError("NEON_DB_URL environment variable is required")

        if not cls.SECRET_KEY:
            raise ValueError("BETTERAUTH_SECRET environment variable is required")

        if len(cls.SECRET_KEY) < 32:
            raise ValueError("BETTERAUTH_SECRET must be at least 32 characters long")

    @classmethod
    def get_session_expiration(cls, remember_me: bool = False) -> datetime:
        """
        Calculate session expiration datetime

        Args:
            remember_me: If True, use extended expiration (30 days), else default (24 hours)

        Returns:
            datetime: Expiration timestamp
        """
        if remember_me:
            return datetime.utcnow() + timedelta(days=cls.SESSION_EXPIRE_DAYS_REMEMBER)
        else:
            return datetime.utcnow() + timedelta(hours=cls.SESSION_EXPIRE_HOURS)


# Password hashing utilities
def hash_password(password: str) -> str:
    """
    Hash password using bcrypt

    Args:
        password: Plain text password

    Returns:
        str: Hashed password
    """
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify password against hash

    Args:
        plain_password: Plain text password to verify
        hashed_password: Stored bcrypt hash

    Returns:
        bool: True if password matches, False otherwise
    """
    return pwd_context.verify(plain_password, hashed_password)


# Validation is optional on import - will be checked at runtime when needed
# This prevents blocking startup when environment variables are not yet loaded
