# Quick Start: BetterAuth Integration Completion

**Feature**: 003-betterauth-completion
**Date**: 2025-12-18
**Audience**: Developers implementing BetterAuth completion

## Overview

This quick start guide helps developers understand and implement the BetterAuth integration completion. The feature addresses gaps identified in analysis by creating a backend that is fully compatible with BetterAuth frontend library, implementing proper session management, and ensuring all authentication flows work correctly with BetterAuth standards.

---

## Prerequisites

Before starting implementation, ensure you have:

**Environment**:
- Python 3.8+ installed
- Node.js 18+ installed
- PostgreSQL client tools (for migration verification)
- Git (for branch management)

**Access**:
- Neon PostgreSQL database connection string
- BetterAuth secret key (min 32 characters)
- Git repository access for the project

**Codebase Knowledge**:
- Familiarity with FastAPI framework
- Understanding of React and Docusaurus
- Basic knowledge of PostgreSQL and migrations
- BetterAuth library concepts and patterns

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend (Docusaurus + React)         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  BetterAuthProvider (Context)                        │  │
│  │  - User authentication state                          │  │
│  │  - Session management (BetterAuth-compatible)        │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  PersonalizationContext (Context)                    │  │
│  │  - User preferences                                   │  │
│  │  - Personalized content cache (localStorage)         │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Auth Components                                      │  │
│  │  - BetterAuth SignupForm                              │  │
│  │  - BetterAuth SigninForm                              │  │
│  │  - BackgroundQuestionsForm (unchanged)               │  │
│  │  - AuthNavbarItem (updated for BetterAuth)           │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ HTTPS + Cookies (BetterAuth-compatible)
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      Backend (FastAPI)                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  BetterAuth-compatible Middleware                     │  │
│  │  - Session validation (BetterAuth patterns)          │  │
│  │  - Cookie parsing (BetterAuth format)                │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Routes (BetterAuth-compatible)                      │  │
│  │  - /auth/register (signup)                           │  │
│  │  - /auth/login (signin)                              │  │
│  │  - /auth/session (get/end session)                   │  │
│  │  - /auth/user (get user info)                        │  │
│  │  - /background (GET/POST - unchanged)                │  │
│  │  - /translate (POST - unchanged)                     │  │
│  │  - /personalize (POST - unchanged)                   │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Services                                             │  │
│  │  - UserService (updated for BetterAuth)              │  │
│  │  - SessionService (updated for BetterAuth)           │  │
│  │  - BackgroundService (unchanged)                     │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  BetterAuth Adapter                                   │  │
│  │  - Custom adapter for Neon Postgres                  │  │
│  │  - User creation/retrieval (BetterAuth patterns)     │  │
│  │  - Session management (BetterAuth patterns)          │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  Neon PostgreSQL Database                    │
│  - users (with BetterAuth fields: email_verified, etc.)     │
│  - sessions (with BetterAuth fields: session_type, etc.)    │
│  - software_background (unchanged)                         │
│  - hardware_background (unchanged)                         │
└─────────────────────────────────────────────────────────────┘
```

---

## Setup Steps

### 1. Environment Configuration

Create or update `.env` file in the project root:

```bash
# Database
NEON_DB_URL=postgresql://user:password@host/database?sslmode=require

# BetterAuth Configuration
BETTERAUTH_SECRET=your-secret-key-min-32-characters-random-string
BETTERAUTH_URL=http://localhost:8000  # Backend URL for BetterAuth

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000

# Frontend Configuration
REACT_APP_BACKEND_URL=http://localhost:8000
```

**Generate BETTERAUTH_SECRET**:
```bash
# Using Python
python -c "import secrets; print(secrets.token_hex(32))"

# Using OpenSSL
openssl rand -hex 32
```

### 2. Database Migrations

Run all migrations to ensure the schema supports BetterAuth fields:

```bash
cd backend
python migrations/run_migrations.py
```

**Verify Migrations**:
```sql
-- Connect to Neon DB
psql $NEON_DB_URL

-- Check tables exist with BetterAuth fields
\dt

-- Verify users table has email_verified column
\d users;

-- Verify sessions table has session_type column
\d sessions;
```

### 3. Install Dependencies

**Backend**:
```bash
cd backend
pip install better-ai==0.8.0  # BetterAuth Python equivalent
pip install -r requirements.txt
```

**Frontend**:
```bash
cd frontend/my-book
npm install @better-auth/react  # BetterAuth React client
npm install
```

### 4. Start Development Servers

**Backend** (Terminal 1):
```bash
cd backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Frontend** (Terminal 2):
```bash
cd frontend/my-book
npm start
```

Access:
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs

---

## Implementation Checklist

### ✅ Already Implemented (Foundation)
**Backend**:
- [x] Database schema with users, sessions, background tables
- [x] Basic authentication routes (custom implementation)
- [x] Password hashing with bcrypt
- [x] Session management (custom implementation)
- [x] Background endpoints for user profile data
- [x] Personalization and translation endpoints

**Frontend**:
- [x] AuthProvider context (custom implementation)
- [x] PersonalizationContext
- [x] SignupForm and SigninForm components (custom implementation)
- [x] BackgroundQuestionsForm component
- [x] Docusaurus i18n configuration

### 🆕 To Be Implemented (BetterAuth Completion)

**Backend**:
- [ ] Create BetterAuth-compatible adapter for Neon Postgres (`backend/src/auth/better_auth_adapter.py`)
- [ ] Update auth routes to BetterAuth patterns (`backend/src/routes/auth.py`)
- [ ] Update session middleware for BetterAuth compatibility (`backend/src/middleware/auth_middleware.py`)
- [ ] Update User and Session models for BetterAuth fields (`backend/src/models/user.py`, `backend/src/models/session.py`)
- [ ] Update UserService and SessionService for BetterAuth patterns (`backend/src/services/user_service.py`, `backend/src/services/session_service.py`)
- [ ] Add email verification fields and handling
- [ ] Register BetterAuth-compatible routes in main app (`backend/src/main.py`)

**Frontend**:
- [ ] Update AuthProvider to use BetterAuth client (`frontend/my-book/src/contexts/AuthProvider.tsx`)
- [ ] Update SignupForm and SigninForm to use BetterAuth methods (`frontend/my-book/src/components/Auth/`)
- [ ] Update authClient utilities for BetterAuth (`frontend/my-book/src/utils/authClient.ts`)
- [ ] Update AuthNavbarItem for BetterAuth state (`frontend/my-book/src/components/Auth/AuthNavbarItem.tsx`)

**Migration**:
- [ ] Implement migration logic for existing users from custom auth to BetterAuth
- [ ] Add migration tracking fields to database schema
- [ ] Create migration scripts for existing user accounts
- [ ] Implement transparent migration during user login

**Testing**:
- [ ] Write BetterAuth-compatible authentication tests (`backend/tests/test_auth.py`)
- [ ] Update frontend component tests for BetterAuth
- [ ] Create migration-specific tests (`backend/tests/test_user_migration.py`)
- [ ] End-to-end authentication flow tests

---

## Key Implementation Tasks

### Task 1: Backend BetterAuth Adapter

**File**: `backend/src/auth/better_auth_adapter.py`

**Implementation**:
```python
from typing import Dict, Any, Optional
from psycopg2.extras import RealDictCursor
from ..database.connection import get_db_connection
from ..utils.validators import hash_password, verify_password

class BetterAuthAdapter:
    def __init__(self):
        self.connection = get_db_connection()

    async def create_user(self, user_data: Dict[str, Any]) -> Dict[str, Any]:
        """Create a new user in the BetterAuth-compatible format."""
        conn = self.connection
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        # Hash password if provided
        password_hash = None
        if 'password' in user_data:
            password_hash = hash_password(user_data['password'])

        # Insert user
        cursor.execute("""
            INSERT INTO users (email, password_hash, email_verified, email_verified_at)
            VALUES (%(email)s, %(password_hash)s, %(email_verified)s, %(email_verified_at)s)
            RETURNING id, email, email_verified, created_at, updated_at
        """, {
            'email': user_data['email'],
            'password_hash': password_hash,
            'email_verified': user_data.get('email_verified', False),
            'email_verified_at': user_data.get('email_verified_at')
        })

        user = cursor.fetchone()
        conn.commit()
        cursor.close()

        return dict(user)

    async def find_user_by_email(self, email: str) -> Optional[Dict[str, Any]]:
        """Find a user by email for authentication."""
        conn = self.connection
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        cursor.execute("""
            SELECT id, email, password_hash, email_verified, email_verified_at, created_at, updated_at
            FROM users WHERE email = %s
        """, (email,))

        user = cursor.fetchone()
        cursor.close()

        return dict(user) if user else None

    async def create_session(self, user_id: str, session_data: Dict[str, Any]) -> Dict[str, Any]:
        """Create a BetterAuth-compatible session."""
        conn = self.connection
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        cursor.execute("""
            INSERT INTO sessions (user_id, token, expires_at, session_type, provider_id)
            VALUES (%(user_id)s, %(token)s, %(expires_at)s, %(session_type)s, %(provider_id)s)
            RETURNING id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
        """, {
            'user_id': user_id,
            'token': session_data['token'],
            'expires_at': session_data['expires_at'],
            'session_type': session_data.get('session_type', 'betterauth'),
            'provider_id': session_data.get('provider_id', 'credentials')
        })

        session = cursor.fetchone()
        conn.commit()
        cursor.close()

        return dict(session)

    async def find_session_by_token(self, token: str) -> Optional[Dict[str, Any]]:
        """Find a session by token for validation."""
        conn = self.connection
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        cursor.execute("""
            SELECT id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
            FROM sessions WHERE token = %s AND expires_at > NOW()
        """, (token,))

        session = cursor.fetchone()
        cursor.close()

        return dict(session) if session else None
```

### Task 2: BetterAuth-Compatible Auth Routes

**File**: `backend/src/routes/auth.py`

**Implementation**:
```python
from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from datetime import datetime, timedelta
import secrets
from ..auth.better_auth_adapter import BetterAuthAdapter
from ..services.user_service import get_user_background

router = APIRouter(prefix="/auth", tags=["authentication"])

class RegisterRequest(BaseModel):
    email: str
    password: str
    name: Optional[str] = None

class LoginRequest(BaseModel):
    email: str
    password: str
    remember: bool = False

class AuthResponse(BaseModel):
    user: dict
    session: dict

@router.post("/register", response_model=AuthResponse)
async def register(request: RegisterRequest):
    """BetterAuth-compatible registration endpoint."""
    adapter = BetterAuthAdapter()

    # Check if user already exists
    existing_user = await adapter.find_user_by_email(request.email)
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")

    # Create new user
    user_data = {
        'email': request.email,
        'password': request.password,
        'email_verified': False  # Email verification can be added later
    }

    user = await adapter.create_user(user_data)

    # Create session
    session_token = secrets.token_urlsafe(32)
    expires_at = datetime.utcnow() + timedelta(days=30 if request.remember else 1)  # 30 days if remember, 1 day otherwise

    session_data = {
        'token': session_token,
        'expires_at': expires_at,
        'session_type': 'betterauth',
        'provider_id': 'credentials'
    }

    session = await adapter.create_session(user['id'], session_data)

    # Set session cookie
    response = AuthResponse(
        user={
            'id': user['id'],
            'email': user['email'],
            'emailVerified': user['email_verified'],
            'name': request.name,
            'createdAt': user['created_at'],
            'updatedAt': user['updated_at']
        },
        session={
            'id': session['id'],
            'userId': session['user_id'],
            'expires': session['expires_at'],
            'sessionToken': session['token']
        }
    )

    return response

@router.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    """BetterAuth-compatible login endpoint."""
    adapter = BetterAuthAdapter()

    # Find user by email
    user = await adapter.find_user_by_email(request.email)
    if not user or not verify_password(request.password, user['password_hash']):
        raise HTTPException(status_code=400, detail="Invalid email or password")

    # Create session
    session_token = secrets.token_urlsafe(32)
    expires_at = datetime.utcnow() + timedelta(days=30 if request.remember else 1)

    session_data = {
        'token': session_token,
        'expires_at': expires_at,
        'session_type': 'betterauth',
        'provider_id': 'credentials'
    }

    session = await adapter.create_session(user['id'], session_data)

    return AuthResponse(
        user={
            'id': user['id'],
            'email': user['email'],
            'emailVerified': user['email_verified'],
            'createdAt': user['created_at'],
            'updatedAt': user['updated_at']
        },
        session={
            'id': session['id'],
            'userId': session['user_id'],
            'expires': session['expires_at'],
            'sessionToken': session['token']
        }
    )

@router.get("/session")
async def get_session(request: Request):
    """Get current session information."""
    session_token = request.cookies.get("authjs.session-token")
    if not session_token:
        return None

    adapter = BetterAuthAdapter()
    session = await adapter.find_session_by_token(session_token)
    if not session:
        return None

    # Get user info
    cursor = adapter.connection.cursor()
    cursor.execute("SELECT id, email, email_verified, created_at, updated_at FROM users WHERE id = %s", (session['user_id'],))
    user = cursor.fetchone()
    cursor.close()

    if not user:
        return None

    return {
        'user': {
            'id': user[0],
            'email': user[1],
            'emailVerified': user[2],
            'createdAt': user[3],
            'updatedAt': user[4]
        },
        'session': {
            'id': session['id'],
            'userId': session['user_id'],
            'expires': session['expires_at']
        }
    }
```

---

## Testing Guide

### Backend Tests

**File**: `backend/tests/test_auth.py`

```python
import pytest
from fastapi.testclient import TestClient
from src.main import app

client = TestClient(app)

def test_register_success():
    response = client.post("/auth/register", json={
        "email": "test@example.com",
        "password": "SecurePass123"
    })
    assert response.status_code == 201
    assert "user" in response.json()
    assert "session" in response.json()

def test_register_duplicate_email():
    # First registration
    client.post("/auth/register", json={
        "email": "duplicate@example.com",
        "password": "SecurePass123"
    })

    # Second registration with same email
    response = client.post("/auth/register", json={
        "email": "duplicate@example.com",
        "password": "SecurePass123"
    })
    assert response.status_code == 400

def test_login_success():
    # Register first
    client.post("/auth/register", json={
        "email": "login@example.com",
        "password": "SecurePass123"
    })

    # Then login
    response = client.post("/auth/login", json={
        "email": "login@example.com",
        "password": "SecurePass123"
    })
    assert response.status_code == 200
    assert "user" in response.json()
    assert "session" in response.json()

def test_login_invalid_credentials():
    response = client.post("/auth/login", json={
        "email": "nonexistent@example.com",
        "password": "wrongpassword"
    })
    assert response.status_code == 400

def test_session_validation():
    # Register and get session
    register_response = client.post("/auth/register", json={
        "email": "session@example.com",
        "password": "SecurePass123"
    })

    # Use the session token to validate session
    session_token = register_response.cookies.get("authjs.session-token")
    response = client.get("/auth/session")
    assert response.status_code == 200
    assert response.json() is not None
```

**Run Backend Tests**:
```bash
cd backend
pytest tests/test_auth.py -v
```

---

## Verification Checklist

After implementation, verify:

**BetterAuth Compatibility**:
- [ ] Registration endpoint follows BetterAuth patterns
- [ ] Login endpoint creates BetterAuth-compatible sessions
- [ ] Session validation works with BetterAuth cookies
- [ ] User information retrieval follows BetterAuth format

**User Migration**:
- [ ] Existing users can log in with BetterAuth endpoints
- [ ] User data remains intact during migration
- [ ] Password hashes remain compatible (bcrypt)
- [ ] Session tracking works for both legacy and BetterAuth sessions

**Security**:
- [ ] Session cookies are HttpOnly and Secure
- [ ] Passwords are properly hashed with bcrypt
- [ ] Email verification fields are properly handled
- [ ] Session expiration works correctly

**Functionality**:
- [ ] New users can register and get BetterAuth-compatible sessions
- [ ] Returning users can log in and maintain sessions
- [ ] Background profile data is accessible for personalized content
- [ ] All existing features (personalization, translation) continue to work

**Performance**:
- [ ] Authentication requests complete within 500ms
- [ ] Session validation completes within 250ms
- [ ] User migration during login is transparent
- [ ] No performance degradation from existing system

---

## Deployment

### Environment Setup

**Production Environment Variables**:
```bash
NEON_DB_URL=<production-database-url>
BETTERAUTH_SECRET=<strong-production-secret>
BETTERAUTH_URL=<production-backend-url>
REACT_APP_BACKEND_URL=<production-backend-url>
```

### Deployment Commands

**Backend (Vercel/Railway)**:
```bash
# Vercel
vercel --prod

# Railway
railway up
```

**Frontend (GitHub Pages/Vercel)**:
```bash
cd frontend/my-book
npm run build
npm run deploy
```

### Post-Deployment Verification

1. Visit production URL
2. Test registration → login flow with BetterAuth endpoints
3. Verify session persistence across page navigation
4. Confirm background questions still work for new users
5. Verify that existing users can still log in (migration)
6. Test that personalization features still work with authenticated users

---

## Troubleshooting

### Common Issues

**Issue**: "Session cookie not set properly"
**Solution**: Check that cookie settings match BetterAuth requirements (HttpOnly, Secure, SameSite=Lax)

**Issue**: "User migration fails during login"
**Solution**: Verify that existing user data is compatible with BetterAuth schema

**Issue**: "Frontend components don't recognize BetterAuth session"
**Solution**: Ensure frontend is updated to use BetterAuth client library methods

**Issue**: "Database migration fails"
**Solution**: Check that all migration scripts are properly formatted and run in order

**Issue**: "Email verification not working"
**Solution**: Verify that email_verified fields are properly set during registration

---

## Next Steps

After completing this quick start:
1. Run `/sp.tasks` to generate detailed implementation tasks
2. Implement the BetterAuth adapter and routes
3. Update frontend components for BetterAuth compatibility
4. Test user migration for existing accounts
5. Deploy to staging environment
6. Conduct user acceptance testing
7. Deploy to production

---

## Resources

**Documentation**:
- [Feature Specification](spec.md)
- [Implementation Plan](plan.md)
- [Data Model](data-model.md)
- [API Contracts](contracts/)

**External References**:
- [BetterAuth Documentation](https://www.better-auth.com/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [React Documentation](https://react.dev/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [Neon Documentation](https://neon.tech/docs)

**Support**:
- Create issues in project repository
- Review existing PHRs in `history/prompts/003-betterauth-completion/`