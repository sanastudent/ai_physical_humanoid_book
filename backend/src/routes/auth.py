"""
BetterAuth-Compatible Authentication API Routes
Implements BetterAuth-compatible endpoints for signup, signin, signout, session validation
"""
from fastapi import APIRouter, HTTPException, Response, Cookie, Request
from typing import Optional, Dict, Any
from uuid import UUID
import secrets
from datetime import datetime, timedelta

from ..auth.better_auth_adapter import (
    create_new_user,
    get_user_by_email,
    create_new_session,
    get_session_by_token,
    remove_session,
    remove_user_sessions
)
from ..config.auth_config import AuthConfig
from ..utils.validators import validate_email, validate_password_strength, ValidationError

router = APIRouter(prefix="/auth", tags=["authentication"])


class RegisterRequest:
    """Registration request model"""
    def __init__(self, email: str, password: str, name: Optional[str] = None):
        self.email = email
        self.password = password
        self.name = name


class LoginRequest:
    """Login request model"""
    def __init__(self, email: str, password: str, remember: bool = False):
        self.email = email
        self.password = password
        self.remember = remember


class AuthResponse:
    """Authentication response model"""
    def __init__(self, user: Dict[str, Any], session: Dict[str, Any]):
        self.user = user
        self.session = session


@router.post("/register", response_model=Dict[str, Any])
async def register(request: Request, response: Response):
    """
    Register a new user with BetterAuth-compatible endpoint

    **Request Body:**
    - email: Valid email address
    - password: User password (min 8 chars, 1 uppercase, 1 lowercase, 1 digit)
    - name: Optional user name

    **Response:**
    - Sets BetterAuth-compatible session cookie
    - Returns user and session information
    """
    try:
        # Parse JSON from request
        import json
        body_bytes = await request.body()
        body = json.loads(body_bytes.decode('utf-8'))

        email = body.get('email')
        password = body.get('password')
        name = body.get('name', None)

        if not email or not password:
            raise HTTPException(status_code=400, detail="Email and password are required")

        # Validate email format
        validated_email = validate_email(email)

        # Validate password strength
        validate_password_strength(password)

        # Check if user already exists
        existing_user = await get_user_by_email(validated_email)
        if existing_user:
            raise HTTPException(status_code=400, detail="Email already registered")

        # Create new user
        user_data = {
            'email': validated_email,
            'password': password,
            'name': name,
            'email_verified': False  # Will be verified separately
        }

        user = await create_new_user(user_data)

        # Create session for the new user
        session_data = {
            'user_id': str(user['id']),
            'expires_at': datetime.utcnow() + timedelta(days=30 if body.get('remember', False) else 1),
            'session_type': 'betterauth',
            'provider_id': 'credentials'
        }

        session = await create_new_session(session_data)

        # Set BetterAuth-compatible session cookie
        cookie_value = session['token']
        max_age = 30 * 24 * 3600 if body.get('remember', False) else 24 * 3600  # 30 days or 24 hours

        response.set_cookie(
            key="authjs.session-token",  # BetterAuth standard cookie name
            value=cookie_value,
            httponly=True,
            secure=AuthConfig.COOKIE_SECURE,  # Set to False for development without HTTPS
            samesite=AuthConfig.COOKIE_SAMESITE,
            max_age=max_age,
            path="/"
        )

        # Return BetterAuth-compatible response
        auth_response = {
            "user": {
                "id": str(user['id']),
                "email": user['email'],
                "emailVerified": user['email_verified_at'],
                "name": name,
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

        return auth_response

    except ValidationError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Registration error: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Failed to register user: {str(e)}")


@router.post("/login", response_model=Dict[str, Any])
async def login(request: Request, response: Response):
    """
    Login existing user with BetterAuth-compatible endpoint

    **Request Body:**
    - email: User's email address
    - password: User's password
    - remember: Optional, extend session duration if true

    **Response:**
    - Sets BetterAuth-compatible session cookie
    - Returns user and session information
    """
    try:
        # Parse JSON from request
        import json
        body_bytes = await request.body()
        body = json.loads(body_bytes.decode('utf-8'))

        email = body.get('email')
        password = body.get('password')
        remember = body.get('remember', False)

        if not email or not password:
            raise HTTPException(status_code=400, detail="Email and password are required")

        # Validate email format
        validated_email = validate_email(email)

        # Find user by email
        user = await get_user_by_email(validated_email)
        if not user:
            raise HTTPException(status_code=400, detail="Invalid email or password")

        # Verify password (using the existing validation function)
        from ..utils.validators import verify_password
        if not verify_password(password, user['password_hash']):
            raise HTTPException(status_code=400, detail="Invalid email or password")

        # Create session for the user
        session_data = {
            'user_id': str(user['id']),
            'expires_at': datetime.utcnow() + timedelta(days=30 if remember else 1),
            'session_type': 'betterauth',
            'provider_id': 'credentials'
        }

        session = await create_new_session(session_data)

        # Set BetterAuth-compatible session cookie
        cookie_value = session['token']
        max_age = 30 * 24 * 3600 if remember else 24 * 3600  # 30 days or 24 hours

        response.set_cookie(
            key="authjs.session-token",  # BetterAuth standard cookie name
            value=cookie_value,
            httponly=True,
            secure=AuthConfig.COOKIE_SECURE,  # Set to False for development without HTTPS
            samesite=AuthConfig.COOKIE_SAMESITE,
            max_age=max_age,
            path="/"
        )

        # Return BetterAuth-compatible response
        auth_response = {
            "user": {
                "id": str(user['id']),
                "email": user['email'],
                "emailVerified": user['email_verified_at'],
                "name": body.get('name'),  # Could be None if not provided
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

        return auth_response

    except ValidationError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Login error: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Failed to login: {str(e)}")


@router.get("/session")
async def get_session(session_token: Optional[str] = Cookie(None, alias="authjs.session-token")):
    """
    Get current session information in BetterAuth-compatible format

    **Response:**
    - Returns user and session info if valid session exists
    - Returns null if no valid session
    """
    if not session_token:
        # Return null as per BetterAuth standard for no session
        return None

    try:
        # Validate session
        session = await get_session_by_token(session_token)
        if not session:
            # Return null as per BetterAuth standard for invalid session
            return None

        # Get user info
        user = await get_user_by_email(session['user_id'])  # This won't work directly, need user ID
        # We need to get user by ID instead
        from ..auth.better_auth_adapter import get_user_by_id
        user = await get_user_by_id(session['user_id'])

        if not user:
            # Return null as per BetterAuth standard for invalid user
            return None

        # Return BetterAuth-compatible session response
        session_response = {
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

        return session_response

    except Exception as e:
        print(f"Session validation error: {str(e)}")
        # Return null as per BetterAuth standard for error
        return None


@router.delete("/session")
async def signout(response: Response, session_token: Optional[str] = Cookie(None, alias="authjs.session-token")):
    """
    Sign out and invalidate current session

    **Response:**
    - Clears session cookie
    - Returns success message
    """
    if not session_token:
        raise HTTPException(status_code=401, detail={
            "error": "SignoutFailed",
            "message": "No active session"
        })

    try:
        # Invalidate the session in the database
        success = await remove_session(session_token)
        if not success:
            raise HTTPException(status_code=401, detail={
                "error": "SignoutFailed",
                "message": "Session not found"
            })

        # Clear the session cookie
        response.delete_cookie(
            key="authjs.session-token",
            path="/",
            httponly=True,
            secure=AuthConfig.COOKIE_SECURE,
            samesite=AuthConfig.COOKIE_SAMESITE
        )

        return {"status": 200, "message": "Successfully signed out"}

    except HTTPException:
        raise
    except Exception as e:
        print(f"Signout error: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "SignoutFailed",
            "message": "Failed to sign out"
        })


# Additional endpoints that might be needed for BetterAuth compatibility

@router.get("/user")
async def get_user(session_token: Optional[str] = Cookie(None, alias="authjs.session-token")):
    """
    Get current user information

    **Response:**
    - Returns user info if valid session exists
    - Returns 401 if no valid session
    """
    if not session_token:
        raise HTTPException(status_code=401, detail={
            "error": "AuthenticationRequired",
            "message": "Not authenticated"
        })

    try:
        # Validate session
        session = await get_session_by_token(session_token)
        if not session:
            raise HTTPException(status_code=401, detail={
                "error": "InvalidSession",
                "message": "Invalid session"
            })

        # Get user info
        from ..auth.better_auth_adapter import get_user_by_id
        user = await get_user_by_id(session['user_id'])

        if not user:
            raise HTTPException(status_code=401, detail={
                "error": "UserNotFound",
                "message": "User not found"
            })

        # Return BetterAuth-compatible user response
        user_response = {
            "id": str(user['id']),
            "email": user['email'],
            "emailVerified": user['email_verified_at'],
            "name": None,  # Name not stored in our system currently
            "image": None,  # Avatar/image not implemented
            "createdAt": user['created_at'].isoformat() if user['created_at'] else None,
            "updatedAt": user['updated_at'].isoformat() if user['updated_at'] else None
        }

        return user_response

    except HTTPException:
        raise
    except Exception as e:
        print(f"Get user error: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "GetUserFailed",
            "message": "Failed to get user"
        })


# Endpoint aliases for frontend compatibility
# Frontend uses /auth/signup and /auth/signin, so we create aliases to /register and /login

@router.post("/signup", response_model=Dict[str, Any])
async def signup(request: Request, response: Response):
    """
    Alias for /register endpoint - for frontend compatibility
    Identical functionality to /register
    """
    return await register(request, response)


@router.post("/signin", response_model=Dict[str, Any])
async def signin(request: Request, response: Response):
    """
    Alias for /login endpoint - for frontend compatibility
    Identical functionality to /login
    """
    return await login(request, response)


@router.post("/signout", response_model=Dict[str, Any])
async def signout_post(response: Response, session_token: Optional[str] = Cookie(None, alias="authjs.session-token")):
    """
    POST alias for DELETE /session endpoint - for frontend compatibility
    Identical functionality to DELETE /session
    """
    return await signout(response, session_token)