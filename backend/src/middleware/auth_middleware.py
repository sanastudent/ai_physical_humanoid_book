"""
Session Validation Middleware for BetterAuth
Validates session tokens and protects authenticated endpoints
"""

from datetime import datetime
from typing import Optional, Callable
from fastapi import Request, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from ..config.auth_config import AuthConfig
from ..database.connection import get_db_pool


security = HTTPBearer(auto_error=False)


class SessionValidationMiddleware:
    """
    Middleware for validating user sessions

    Checks for valid session token in cookies or Authorization header
    Validates session expiration and user existence
    """

    @staticmethod
    def get_session_token_from_request(request: Request) -> Optional[str]:
        """
        Extract session token from request

        Checks in order:
        1. Cookie (session_token)
        2. Authorization header (Bearer token)

        Args:
            request: FastAPI request object

        Returns:
            str: Session token if found, None otherwise
        """
        # Check cookie first (preferred for web applications)
        token = request.cookies.get(AuthConfig.COOKIE_NAME)
        if token:
            return token

        # Check Authorization header as fallback
        auth_header = request.headers.get("Authorization")
        if auth_header and auth_header.startswith("Bearer "):
            return auth_header.replace("Bearer ", "")

        return None

    @staticmethod
    async def validate_session(request: Request) -> dict:
        """
        Validate session token and return session data

        Args:
            request: FastAPI request object

        Returns:
            dict: Session data with user_id, session_id, expires_at

        Raises:
            HTTPException: 401 if session is invalid or expired
        """
        token = SessionValidationMiddleware.get_session_token_from_request(request)

        if not token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="No active session or session expired",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Validate session in database
        db_pool = get_db_pool()

        with db_pool.get_cursor() as cursor:
            # Query session with user join to verify user still exists
            cursor.execute(
                """
                SELECT
                    s.id as session_id,
                    s.user_id,
                    s.expires_at,
                    u.email
                FROM sessions s
                JOIN users u ON s.user_id = u.id
                WHERE s.token = %s
                """,
                (token,)
            )
            session = cursor.fetchone()

        if not session:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid session token",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Check if session is expired
        if session['expires_at'] < datetime.utcnow():
            # Clean up expired session
            with db_pool.get_cursor() as cursor:
                cursor.execute("DELETE FROM sessions WHERE token = %s", (token,))

            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Session expired",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Update session activity timestamp
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "UPDATE sessions SET updated_at = NOW() WHERE token = %s",
                (token,)
            )

        return {
            "session_id": session['session_id'],
            "user_id": session['user_id'],
            "email": session['email'],
            "expires_at": session['expires_at']
        }


# Dependency for FastAPI route protection
async def require_authentication(request: Request) -> dict:
    """
    FastAPI dependency for protecting authenticated routes

    Usage:
        @app.get("/protected")
        async def protected_route(session: dict = Depends(require_authentication)):
            user_id = session["user_id"]
            return {"message": f"Hello user {user_id}"}

    Args:
        request: FastAPI request object

    Returns:
        dict: Session data

    Raises:
        HTTPException: 401 if authentication fails
    """
    return await SessionValidationMiddleware.validate_session(request)


# Optional authentication (doesn't raise error if no session)
async def get_optional_session(request: Request) -> Optional[dict]:
    """
    FastAPI dependency for optional authentication

    Returns session data if authenticated, None otherwise (no error)

    Usage:
        @app.get("/content")
        async def content_route(session: Optional[dict] = Depends(get_optional_session)):
            if session:
                # Personalized content for logged-in user
                return {"content": "personalized", "user_id": session["user_id"]}
            else:
                # Generic content for anonymous user
                return {"content": "generic"}

    Args:
        request: FastAPI request object

    Returns:
        dict: Session data if authenticated, None otherwise
    """
    try:
        return await SessionValidationMiddleware.validate_session(request)
    except HTTPException:
        return None
