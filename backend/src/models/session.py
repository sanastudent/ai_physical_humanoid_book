"""
Session model for BetterAuth authentication
"""

from datetime import datetime
from typing import Optional
from uuid import UUID
from pydantic import BaseModel, Field


class SessionBase(BaseModel):
    """Base session model with common fields"""
    user_id: UUID = Field(..., description="Associated user ID")
    expires_at: datetime = Field(..., description="Session expiration time")


class SessionCreate(SessionBase):
    """Model for creating a new session"""
    token: str = Field(..., min_length=32, description="Session token (min 32 chars)")


class Session(SessionBase):
    """Full session model (database representation)"""
    id: UUID = Field(..., description="Unique session identifier")
    token: str = Field(..., description="Session token")
    created_at: datetime = Field(..., description="Session creation timestamp")
    updated_at: datetime = Field(..., description="Last session activity timestamp")
    session_type: Optional[str] = Field(default="legacy", description="Session type (legacy or betterauth)")
    provider_id: Optional[str] = Field(default="credentials", description="Authentication provider ID")

    class Config:
        from_attributes = True


class SessionResponse(BaseModel):
    """Session model for API responses (excludes token)"""
    id: UUID = Field(..., description="Unique session identifier")
    user_id: UUID = Field(..., description="Associated user ID")
    expires_at: datetime = Field(..., description="Session expiration time")
    created_at: datetime = Field(..., description="Session creation timestamp")
    session_type: Optional[str] = Field(default="legacy", description="Session type (legacy or betterauth)")
    provider_id: Optional[str] = Field(default="credentials", description="Authentication provider ID")

    class Config:
        from_attributes = True
