"""
Authentication API Routes - Signup, Signin, Signout, Session
"""

from fastapi import APIRouter, HTTPException, Response, Cookie, Depends
from typing import Optional
from uuid import UUID

from ..models.user import UserCreate, UserResponse
from ..services.user_service import UserService
from ..services.session_service import SessionService
from ..config.auth_config import AuthConfig
from ..utils.validators import ValidationError
from ..middleware.auth_middleware import require_authentication

router = APIRouter(prefix="/auth", tags=["authentication"])


class SignupRequest(UserCreate):
    """Signup request model (email + password)"""
    pass


class SigninRequest(UserCreate):
    """Signin request model (email + password)"""
    remember_me: bool = False


class AuthResponse(UserResponse):
    """Authentication response with user data"""
    pass


@router.post("/signup", response_model=AuthResponse, status_code=201)
async def signup(request: SignupRequest, response: Response):
    """
    Create a new user account (Step 1 of signup flow)

    **Flow:**
    1. User submits email and password
    2. Backend validates and creates user account
    3. Backend creates session and returns session cookie
    4. Frontend redirects to background questions (Step 2)

    **Request Body:**
    - email: Valid email address (RFC 5322)
    - password: Min 8 chars, 1 uppercase, 1 lowercase, 1 digit

    **Response:**
    - 201: User created successfully, session cookie set
    - 400: Validation error (invalid email, weak password, duplicate email)
    - 500: Server error
    """
    try:
        # Create user account
        user = UserService.create_user(request)

        # Create session for new user
        session = SessionService.create_session(user.id, remember_me=False)

        # Set session cookie
        response.set_cookie(
            key=AuthConfig.COOKIE_NAME,
            value=session.token,
            httponly=AuthConfig.COOKIE_HTTPONLY,
            secure=AuthConfig.COOKIE_SECURE,
            samesite=AuthConfig.COOKIE_SAMESITE,
            max_age=AuthConfig.SESSION_EXPIRE_HOURS * 3600  # Convert hours to seconds
        )

        return user

    except ValidationError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Signup error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to create account")


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest, response: Response):
    """
    Sign in with email and password

    **Request Body:**
    - email: User's email address
    - password: User's password
    - remember_me: Optional, extends session to 30 days (default: false/24 hours)

    **Response:**
    - 200: Signed in successfully, session cookie set
    - 401: Invalid credentials
    - 500: Server error
    """
    try:
        # Authenticate user
        user = UserService.authenticate_user(request.email, request.password)

        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Create session
        session = SessionService.create_session(user.id, remember_me=request.remember_me)

        # Set session cookie with appropriate expiration
        max_age = (
            AuthConfig.SESSION_EXPIRE_DAYS_REMEMBER * 24 * 3600
            if request.remember_me
            else AuthConfig.SESSION_EXPIRE_HOURS * 3600
        )

        response.set_cookie(
            key=AuthConfig.COOKIE_NAME,
            value=session.token,
            httponly=AuthConfig.COOKIE_HTTPONLY,
            secure=AuthConfig.COOKIE_SECURE,
            samesite=AuthConfig.COOKIE_SAMESITE,
            max_age=max_age
        )

        return user

    except HTTPException:
        raise
    except Exception as e:
        print(f"Signin error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to sign in")


@router.post("/signout", status_code=204)
async def signout(
    response: Response,
    session_token: Optional[str] = Cookie(None, alias=AuthConfig.COOKIE_NAME)
):
    """
    Sign out and invalidate current session

    **Response:**
    - 204: Signed out successfully, session cookie cleared
    - 401: No active session
    """
    if not session_token:
        raise HTTPException(status_code=401, detail="No active session")

    try:
        # Invalidate session in database
        SessionService.invalidate_session(session_token)

        # Clear session cookie
        response.delete_cookie(
            key=AuthConfig.COOKIE_NAME,
            httponly=AuthConfig.COOKIE_HTTPONLY,
            secure=AuthConfig.COOKIE_SECURE,
            samesite=AuthConfig.COOKIE_SAMESITE
        )

        return None  # 204 No Content

    except Exception as e:
        print(f"Signout error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to sign out")


@router.get("/session", response_model=AuthResponse)
async def get_session(
    session_token: Optional[str] = Cookie(None, alias=AuthConfig.COOKIE_NAME)
):
    """
    Get current user session

    **Use Cases:**
    - Check if user is authenticated
    - Get current user data
    - Validate session on page load

    **Response:**
    - 200: Valid session, returns user data
    - 401: No session or session expired
    """
    if not session_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    try:
        # Validate session
        session = SessionService.get_current_session(session_token)

        if not session:
            raise HTTPException(status_code=401, detail="Session expired or invalid")

        # Get user data
        user = UserService.get_user_by_id(session.user_id)

        if not user:
            raise HTTPException(status_code=401, detail="User not found")

        return user

    except HTTPException:
        raise
    except Exception as e:
        print(f"Session validation error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to validate session")


@router.delete("/session", status_code=204)
async def delete_all_sessions(session_data: dict = Depends(require_authentication)):
    """
    Delete all sessions for current user (logout from all devices)

    **Authentication Required**

    **Response:**
    - 204: All sessions deleted successfully
    - 401: Not authenticated
    """
    try:
        user_id = session_data["user_id"]
        count = SessionService.invalidate_user_sessions(user_id)

        print(f"Deleted {count} sessions for user {user_id}")
        return None  # 204 No Content

    except Exception as e:
        print(f"Delete all sessions error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to delete sessions")
