"""
Combined Background models for request/response handling
"""

from typing import Optional
from uuid import UUID
from pydantic import BaseModel, Field

from .software_background import SoftwareBackground, SoftwareBackgroundCreate
from .hardware_background import HardwareBackground, HardwareBackgroundCreate


class BackgroundInput(BaseModel):
    """
    Input model for creating/updating user background data
    Used in POST /background and PUT /background requests
    """
    software: SoftwareBackgroundCreate = Field(
        ...,
        description="Software development background information"
    )
    hardware: HardwareBackgroundCreate = Field(
        ...,
        description="Hardware development background information"
    )


class BackgroundData(BaseModel):
    """
    Combined background data model (database representation)
    """
    user_id: UUID = Field(..., description="Associated user ID")
    software: SoftwareBackground = Field(
        ...,
        description="Software background with metadata"
    )
    hardware: HardwareBackground = Field(
        ...,
        description="Hardware background with metadata"
    )


class BackgroundResponse(BaseModel):
    """
    Background data model for API responses
    Simplified version without nested complexity
    """
    user_id: UUID = Field(..., description="Associated user ID")
    software: SoftwareBackground = Field(..., description="Software background")
    hardware: HardwareBackground = Field(..., description="Hardware background")

    class Config:
        from_attributes = True
