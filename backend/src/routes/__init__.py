"""
API route handlers for BetterAuth authentication system
"""

from .auth import router as auth_router
from .background import router as background_router

__all__ = [
    "auth_router",
    "background_router",
]
