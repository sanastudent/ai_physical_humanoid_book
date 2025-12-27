"""
Session Validation Middleware for BetterAuth Compatibility
Implements middleware to validate sessions for protected routes
"""
from fastapi import HTTPException, Request, Response
from fastapi.responses import JSONResponse
from typing import Optional, Dict, Any
import time

from ..auth.better_auth_adapter import get_session_by_token, get_user_by_id
from ..models.session import Session
from ..models.user import UserResponse


class SessionValidationMiddleware:
    """
    Middleware for validating BetterAuth-compatible sessions
    Can be applied to routes that require authentication
    """

    def __init__(self):
        pass

    @staticmethod
    async def validate_session(request: Request) -> Optional[Dict[str, Any]]:
        """
        Validate session from request and return user/session data if valid

        Args:
            request: FastAPI request object

        Returns:
            Dict with user and session data if valid, None if invalid/expired
        """
        # Extract session token from cookie (BetterAuth standard name)
        session_token = request.cookies.get("authjs.session-token")

        if not session_token:
            return None

        try:
            # Validate session using BetterAuth adapter
            session = await get_session_by_token(session_token)
            if not session:
                return None

            # Get user info
            user = await get_user_by_id(str(session['user_id']))
            if not user:
                return None

            # Return structured data compatible with BetterAuth
            return {
                "user": {
                    "id": str(user['id']),
                    "email": user['email'],
                    "emailVerified": user['email_verified_at'],
                    "name": None,  # Name not stored in our system currently
                    "createdAt": user['created_at'].isoformat() if user['created_at'] else None,
                    "updatedAt": user['updated_at'].isoformat() if user['updated_at'] else None
                },
                "session": {
                    "id": str(session['id']),
                    "userId": str(session['user_id']),
                    "expires": session['expires_at'].isoformat(),
                    "sessionToken": session['token']
                }
            }
        except Exception:
            # If there's any error validating the session, return None
            return None

    @staticmethod
    async def require_auth(request: Request, call_next):
        """
        Middleware function that requires valid authentication
        Returns 401 if no valid session exists
        """
        session_data = await SessionValidationMiddleware.validate_session(request)

        if not session_data:
            # Return 401 for unauthorized access
            return JSONResponse(
                status_code=401,
                content={"error": "Unauthorized", "message": "Valid session required"}
            )

        # Add session data to request state for use in route handlers
        request.state.user = session_data["user"]
        request.state.session = session_data["session"]

        response = await call_next(request)
        return response

    @staticmethod
    async def optional_auth(request: Request, call_next):
        """
        Middleware function that adds session info if available, but doesn't require it
        """
        session_data = await SessionValidationMiddleware.validate_session(request)

        # Add session data to request state if available, otherwise None
        request.state.user = session_data["user"] if session_data else None
        request.state.session = session_data["session"] if session_data else None

        response = await call_next(request)
        return response


# Convenience functions for use in route handlers
async def get_current_user(request: Request) -> Optional[Dict[str, Any]]:
    """
    Get current user from request state (requires middleware to be applied)

    Args:
        request: FastAPI request object

    Returns:
        Current user data if available, None otherwise
    """
    return getattr(request.state, 'user', None)


async def get_current_session(request: Request) -> Optional[Dict[str, Any]]:
    """
    Get current session from request state (requires middleware to be applied)

    Args:
        request: FastAPI request object

    Returns:
        Current session data if available, None otherwise
    """
    return getattr(request.state, 'session', None)


async def require_current_user(request: Request) -> Dict[str, Any]:
    """
    Require current user to be authenticated (raises 401 if not)

    Args:
        request: FastAPI request object

    Returns:
        Current user data

    Raises:
        HTTPException: 401 if no valid session
    """
    user = await get_current_user(request)
    if not user:
        raise HTTPException(status_code=401, detail="Valid session required")
    return user