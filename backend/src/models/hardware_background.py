"""
Hardware Background model for user hardware development preferences
"""

from datetime import datetime
from typing import List
from uuid import UUID
from pydantic import BaseModel, Field
from enum import Enum


class HardwareExperienceLevel(str, Enum):
    """Hardware development experience levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareBackgroundBase(BaseModel):
    """Base hardware background model"""
    experience_level: HardwareExperienceLevel = Field(
        ...,
        description="Hardware development experience level"
    )
    preferred_platforms: List[str] = Field(
        ...,
        min_length=1,
        description="Preferred development platforms (e.g., desktop, mobile, embedded)",
        examples=[["desktop", "mobile"]]
    )
    device_types: List[str] = Field(
        default_factory=list,
        description="Familiar device types (e.g., laptop, smartphone, raspberry-pi)",
        examples=[["laptop", "smartphone"]]
    )


class HardwareBackgroundCreate(HardwareBackgroundBase):
    """Model for creating hardware background"""
    pass


class HardwareBackground(HardwareBackgroundBase):
    """Full hardware background model (database representation)"""
    id: UUID = Field(..., description="Unique background record ID")
    user_id: UUID = Field(..., description="Associated user ID (one-to-one)")
    created_at: datetime = Field(..., description="Record creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    class Config:
        from_attributes = True
        use_enum_values = True
