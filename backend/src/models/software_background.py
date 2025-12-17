"""
Software Background model for user software development preferences
"""

from datetime import datetime
from typing import List
from uuid import UUID
from pydantic import BaseModel, Field
from enum import Enum


class ExperienceLevel(str, Enum):
    """Software development experience levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class SoftwareBackgroundBase(BaseModel):
    """Base software background model"""
    experience_level: ExperienceLevel = Field(
        ...,
        description="Software development experience level"
    )
    preferred_languages: List[str] = Field(
        ...,
        min_length=1,
        description="Preferred programming languages (e.g., Python, JavaScript, Go)",
        examples=[["Python", "JavaScript", "Go"]]
    )
    preferred_frameworks: List[str] = Field(
        default_factory=list,
        description="Preferred frameworks (e.g., FastAPI, React, Django)",
        examples=[["FastAPI", "React", "Fiber"]]
    )


class SoftwareBackgroundCreate(SoftwareBackgroundBase):
    """Model for creating software background"""
    pass


class SoftwareBackground(SoftwareBackgroundBase):
    """Full software background model (database representation)"""
    id: UUID = Field(..., description="Unique background record ID")
    user_id: UUID = Field(..., description="Associated user ID (one-to-one)")
    created_at: datetime = Field(..., description="Record creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    class Config:
        from_attributes = True
        use_enum_values = True
