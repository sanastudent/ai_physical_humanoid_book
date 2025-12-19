"""
User model for BetterAuth authentication
"""

from datetime import datetime
from typing import Optional
from uuid import UUID
from pydantic import BaseModel, EmailStr, Field


class UserBase(BaseModel):
    """Base user model with common fields"""
    email: EmailStr = Field(..., description="User email address (must be unique)")


class UserCreate(UserBase):
    """Model for creating a new user"""
    password: str = Field(
        ...,
        min_length=8,
        description="Password (min 8 chars, 1 uppercase, 1 lowercase, 1 number)"
    )


class User(UserBase):
    """Full user model (database representation)"""
    id: UUID = Field(..., description="Unique user identifier")
    password_hash: str = Field(..., description="bcrypt hashed password")
    created_at: datetime = Field(..., description="Account creation timestamp")
    updated_at: datetime = Field(..., description="Last account update timestamp")
    email_verified: bool = Field(default=False, description="Whether email has been verified")
    email_verified_at: Optional[datetime] = Field(None, description="When email was verified")

    class Config:
        from_attributes = True  # For Pydantic v2 (formerly orm_mode)


class UserResponse(UserBase):
    """User model for API responses (excludes password_hash)"""
    id: UUID = Field(..., description="Unique user identifier")
    created_at: datetime = Field(..., description="Account creation timestamp")
    updated_at: datetime = Field(..., description="Last account update timestamp")
    email_verified: bool = Field(default=False, description="Whether email has been verified")
    email_verified_at: Optional[datetime] = Field(None, description="When email was verified")

    class Config:
        from_attributes = True
