# Quickstart Guide: BetterAuth Authentication Implementation

**Feature**: 002-betterauth-signup
**Date**: 2025-12-11
**Estimated Time**: 4-6 hours for full implementation

## Overview

This guide provides step-by-step instructions for implementing user authentication with background collection using BetterAuth and Neon DB. The implementation adds signup/signin functionality and integrates with the existing personalization feature.

## Prerequisites

Before starting, ensure you have:

- ✅ Node.js 18+ installed
- ✅ Python 3.11+ installed (for backend)
- ✅ Neon DB account and database provisioned
- ✅ Access to existing codebase with personalization feature
- ✅ BetterAuth library familiarity (optional, but helpful)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (Docusaurus/React)               │
│                                                              │
│  ┌────────────┐  ┌───────────────┐  ┌──────────────────┐   │
│  │ Signup Page│  │ Signin Page   │  │ Profile Settings │   │
│  └──────┬─────┘  └───────┬───────┘  └────────┬─────────┘   │
│         │                 │                    │             │
│         └─────────────────┼────────────────────┘             │
│                           │                                  │
│                    ┌──────▼──────┐                          │
│                    │ BetterAuth  │                          │
│                    │   Hooks     │                          │
│                    └──────┬──────┘                          │
│                           │                                  │
│                    ┌──────▼──────────┐                      │
│                    │ Personalization │                      │
│                    │    Context      │                      │
│                    └──────┬──────────┘                      │
└───────────────────────────┼─────────────────────────────────┘
                            │ HTTP Requests
┌───────────────────────────▼─────────────────────────────────┐
│                    Backend (FastAPI)                         │
│                                                              │
│  ┌────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ Auth Routes│  │ Background   │  │ Session         │    │
│  │            │  │ Routes       │  │ Middleware      │    │
│  └──────┬─────┘  └───────┬──────┘  └────────┬─────────┘   │
│         │                 │                   │             │
│         └─────────────────┼───────────────────┘             │
│                           │                                  │
│                    ┌──────▼──────┐                          │
│                    │   Neon DB   │                          │
│                    │ (Postgres)  │                          │
│                    └─────────────┘                          │
└─────────────────────────────────────────────────────────────┘
```

## Implementation Checklist

### Phase 1: Database Setup (30 mins)

- [ ] **Step 1.1**: Connect to Neon DB dashboard
- [ ] **Step 1.2**: Run migration 001 (users + sessions tables)
- [ ] **Step 1.3**: Run migration 002 (background tables)
- [ ] **Step 1.4**: Run migration 003 (triggers)
- [ ] **Step 1.5**: Verify tables exist with correct schema
- [ ] **Step 1.6**: Configure environment variables (`NEON_DB_URL`)

**Commands**:
```bash
# Set environment variable
export NEON_DB_URL="postgresql://user:password@host/database"

# Run migrations (assuming Prisma/Drizzle setup)
npx prisma migrate deploy
# OR
python -m alembic upgrade head
```

**Verification**:
```sql
-- Check tables exist
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
  AND table_name IN ('users', 'sessions', 'software_background', 'hardware_background');

-- Should return 4 rows
```

---

### Phase 2: Backend API Implementation (2-3 hours)

#### 2.1: Install Dependencies (5 mins)

- [ ] **Step 2.1.1**: Install Python database library
- [ ] **Step 2.1.2**: Install password hashing library
- [ ] **Step 2.1.3**: Install validation library

```bash
cd backend
pip install psycopg2-binary bcrypt pydantic email-validator
# OR add to requirements.txt and run:
pip install -r requirements.txt
```

#### 2.2: Create Database Models (20 mins)

- [ ] **Step 2.2.1**: Create `backend/src/models/user.py`
- [ ] **Step 2.2.2**: Create `backend/src/models/session.py`
- [ ] **Step 2.2.3**: Create `backend/src/models/background.py`

**File**: `backend/src/models/user.py`
```python
from pydantic import BaseModel, EmailStr
from datetime import datetime
from typing import Optional
import uuid

class UserCreate(BaseModel):
    email: EmailStr
    password: str  # Min 8 chars, validated in service layer

class User(BaseModel):
    id: uuid.UUID
    email: EmailStr
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

**File**: `backend/src/models/background.py`
```python
from pydantic import BaseModel, Field
from typing import List
from datetime import datetime
import uuid

class SoftwareBackground(BaseModel):
    experience_level: str = Field(..., pattern="^(beginner|intermediate|advanced)$")
    preferred_languages: List[str] = Field(..., min_items=1)
    preferred_frameworks: List[str]

class HardwareBackground(BaseModel):
    experience_level: str = Field(..., pattern="^(beginner|intermediate|advanced)$")
    preferred_platforms: List[str] = Field(..., min_items=1)
    device_types: List[str]

class BackgroundInput(BaseModel):
    software: SoftwareBackground
    hardware: HardwareBackground

class BackgroundData(BaseModel):
    user_id: uuid.UUID
    software: SoftwareBackground
    hardware: HardwareBackground
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

#### 2.3: Create Database Service (30 mins)

- [ ] **Step 2.3.1**: Create `backend/src/services/db.py` for connection pooling
- [ ] **Step 2.3.2**: Create `backend/src/services/auth_service.py`
- [ ] **Step 2.3.3**: Create `backend/src/services/background_service.py`

**File**: `backend/src/services/db.py`
```python
import psycopg2
from psycopg2.pool import SimpleConnectionPool
import os

db_url = os.getenv("NEON_DB_URL")
connection_pool = SimpleConnectionPool(1, 20, db_url)

def get_connection():
    return connection_pool.getconn()

def release_connection(conn):
    connection_pool.putconn(conn)
```

**File**: `backend/src/services/auth_service.py`
```python
import bcrypt
import uuid
from datetime import datetime, timedelta
from .db import get_connection, release_connection
from ..models.user import User, UserCreate

def hash_password(password: str) -> str:
    return bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

def verify_password(password: str, password_hash: str) -> bool:
    return bcrypt.checkpw(password.encode('utf-8'), password_hash.encode('utf-8'))

def create_user(user_data: UserCreate) -> User:
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            user_id = uuid.uuid4()
            password_hash = hash_password(user_data.password)

            cur.execute(
                """
                INSERT INTO users (id, email, password_hash)
                VALUES (%s, %s, %s)
                RETURNING id, email, created_at, updated_at
                """,
                (user_id, user_data.email, password_hash)
            )
            row = cur.fetchone()
            conn.commit()

            return User(
                id=row[0],
                email=row[1],
                created_at=row[2],
                updated_at=row[3]
            )
    finally:
        release_connection(conn)

def authenticate_user(email: str, password: str) -> Optional[User]:
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT id, email, password_hash, created_at, updated_at FROM users WHERE email = %s",
                (email,)
            )
            row = cur.fetchone()

            if row and verify_password(password, row[2]):
                return User(id=row[0], email=row[1], created_at=row[3], updated_at=row[4])
            return None
    finally:
        release_connection(conn)

def create_session(user_id: uuid.UUID, remember_me: bool = False) -> str:
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            session_id = uuid.uuid4()
            token = str(uuid.uuid4())
            expires_at = datetime.utcnow() + timedelta(days=30 if remember_me else 1)

            cur.execute(
                """
                INSERT INTO sessions (id, user_id, token, expires_at)
                VALUES (%s, %s, %s, %s)
                """,
                (session_id, user_id, token, expires_at)
            )
            conn.commit()
            return token
    finally:
        release_connection(conn)

def validate_session(token: str) -> Optional[User]:
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT u.id, u.email, u.created_at, u.updated_at
                FROM sessions s
                JOIN users u ON s.user_id = u.id
                WHERE s.token = %s AND s.expires_at > NOW()
                """,
                (token,)
            )
            row = cur.fetchone()

            if row:
                return User(id=row[0], email=row[1], created_at=row[2], updated_at=row[3])
            return None
    finally:
        release_connection(conn)

def delete_session(token: str):
    conn = get_connection()
    try:
        with conn.cursor() as cur:
            cur.execute("DELETE FROM sessions WHERE token = %s", (token,))
            conn.commit()
    finally:
        release_connection(conn)
```

#### 2.4: Create API Routes (45 mins)

- [ ] **Step 2.4.1**: Create `backend/src/api/auth.py`
- [ ] **Step 2.4.2**: Create `backend/src/api/background.py`
- [ ] **Step 2.4.3**: Add routes to main FastAPI app

**File**: `backend/src/api/auth.py`
```python
from fastapi import APIRouter, HTTPException, Response, Cookie
from typing import Optional
from ..models.user import UserCreate, User
from ..services.auth_service import create_user, authenticate_user, create_session, validate_session, delete_session

router = APIRouter(prefix="/api/auth", tags=["Authentication"])

@router.post("/signup", status_code=201)
async def signup(user_data: UserCreate, response: Response):
    try:
        user = create_user(user_data)
        token = create_session(user.id)

        response.set_cookie(
            key="session_token",
            value=token,
            httponly=True,
            secure=True,  # HTTPS only in production
            samesite="lax",
            max_age=86400  # 24 hours
        )

        return {"user": user, "message": "Account created successfully"}
    except Exception as e:
        if "unique constraint" in str(e).lower():
            raise HTTPException(status_code=409, detail="Email already registered")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.post("/signin")
async def signin(email: str, password: str, remember_me: bool = False, response: Response):
    user = authenticate_user(email, password)

    if not user:
        raise HTTPException(status_code=401, detail="Invalid email or password")

    token = create_session(user.id, remember_me)
    max_age = 2592000 if remember_me else 86400  # 30 days or 24 hours

    response.set_cookie(
        key="session_token",
        value=token,
        httponly=True,
        secure=True,
        samesite="lax",
        max_age=max_age
    )

    return {"user": user, "message": "Signed in successfully"}

@router.post("/signout")
async def signout(response: Response, session_token: Optional[str] = Cookie(None)):
    if session_token:
        delete_session(session_token)

    response.delete_cookie("session_token")
    return {"message": "Signed out successfully"}

@router.get("/session")
async def get_session(session_token: Optional[str] = Cookie(None)):
    if not session_token:
        return {"user": None, "session": None}

    user = validate_session(session_token)

    if not user:
        return {"user": None, "session": None}

    return {"user": user, "session": {"active": True}}
```

#### 2.5: Add Session Middleware (15 mins)

- [ ] **Step 2.5.1**: Create `backend/src/middleware/auth.py`
- [ ] **Step 2.5.2**: Add middleware to protect routes

**File**: `backend/src/middleware/auth.py`
```python
from fastapi import Request, HTTPException
from typing import Optional
from ..services.auth_service import validate_session

async def get_current_user(request: Request):
    token = request.cookies.get("session_token")

    if not token:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user = validate_session(token)

    if not user:
        raise HTTPException(status_code=401, detail="Session expired")

    return user
```

---

### Phase 3: Frontend Implementation (2-3 hours)

#### 3.1: Install BetterAuth (5 mins)

- [ ] **Step 3.1.1**: Install BetterAuth package

```bash
cd frontend/my-book
npm install better-auth
```

#### 3.2: Create Auth Context (20 mins)

- [ ] **Step 3.2.1**: Create `frontend/my-book/src/contexts/AuthContext.tsx`

**File**: `frontend/my-book/src/contexts/AuthContext.tsx`
```typescript
import React, { createContext, useContext, useState, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  createdAt: string;
  updatedAt: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  signin: (email: string, password: string, rememberMe?: boolean) => Promise<void>;
  signup: (email: string, password: string) => Promise<void>;
  signout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check session on mount
    fetch('http://localhost:8000/api/auth/session', { credentials: 'include' })
      .then(res => res.json())
      .then(data => {
        setUser(data.user);
        setLoading(false);
      })
      .catch(() => setLoading(false));
  }, []);

  const signup = async (email: string, password: string) => {
    const res = await fetch('http://localhost:8000/api/auth/signup', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
      credentials: 'include'
    });

    if (!res.ok) {
      const error = await res.json();
      throw new Error(error.detail || 'Signup failed');
    }

    const data = await res.json();
    setUser(data.user);
  };

  const signin = async (email: string, password: string, rememberMe = false) => {
    const res = await fetch(`http://localhost:8000/api/auth/signin?email=${email}&password=${password}&remember_me=${rememberMe}`, {
      method: 'POST',
      credentials: 'include'
    });

    if (!res.ok) {
      const error = await res.json();
      throw new Error(error.detail || 'Signin failed');
    }

    const data = await res.json();
    setUser(data.user);
  };

  const signout = async () => {
    await fetch('http://localhost:8000/api/auth/signout', {
      method: 'POST',
      credentials: 'include'
    });
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, loading, signin, signup, signout }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}
```

#### 3.3: Create Signup Page (30 mins)

- [ ] **Step 3.3.1**: Create `frontend/my-book/src/pages/signup.tsx`

**File**: `frontend/my-book/src/pages/signup.tsx`
```typescript
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useNavigate } from 'react-router-dom';

export default function SignupPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { signup } = useAuth();
  const navigate = useNavigate();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    try {
      await signup(email, password);
      navigate('/background-questions');
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <div className="signup-container">
      <h1>Create Account</h1>
      <form onSubmit={handleSubmit}>
        <input
          type="email"
          placeholder="Email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
        />
        <input
          type="password"
          placeholder="Password (min 8 chars)"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
          minLength={8}
        />
        {error && <div className="error">{error}</div>}
        <button type="submit">Sign Up</button>
      </form>
    </div>
  );
}
```

#### 3.4: Create Background Questions Page (45 mins)

- [ ] **Step 3.4.1**: Create `frontend/my-book/src/pages/background-questions.tsx`

*(See detailed implementation in tasks.md - this is a multi-step form)*

#### 3.5: Update Personalization Context (20 mins)

- [ ] **Step 3.5.1**: Extend existing PersonalizationContext to check `useAuth()` for user

---

### Phase 4: Testing & Validation (1 hour)

- [ ] **Step 4.1**: Test signup flow (email/password → background questions → personalization)
- [ ] **Step 4.2**: Test signin flow (email/password → auto-redirect to content)
- [ ] **Step 4.3**: Test signout flow (clear session → redirect to login)
- [ ] **Step 4.4**: Test personalization button (only visible to logged-in users)
- [ ] **Step 4.5**: Test background update flow (profile settings → save changes)
- [ ] **Step 4.6**: Test edge cases (duplicate email, invalid credentials, expired session)

---

## Common Issues & Troubleshooting

### Issue 1: CORS Errors
**Symptom**: "Access-Control-Allow-Origin" error in browser console
**Solution**: Add CORS middleware in FastAPI
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Issue 2: Session Cookie Not Set
**Symptom**: User authenticated but session not persisting
**Solution**: Ensure `credentials: 'include'` in all fetch requests

### Issue 3: Database Connection Fails
**Symptom**: "Unable to connect to database" error
**Solution**: Verify `NEON_DB_URL` environment variable is set correctly

---

## Next Steps

After completing this quickstart:

1. Run `/sp.tasks` to generate detailed implementation tasks
2. Implement password reset flow (future enhancement)
3. Add email verification (future enhancement)
4. Configure production environment variables
5. Deploy backend to production
6. Test end-to-end flow in production environment

---

## Estimated Timeline

| Phase | Task | Time |
|-------|------|------|
| 1 | Database Setup | 30 mins |
| 2 | Backend API Implementation | 2-3 hours |
| 3 | Frontend Implementation | 2-3 hours |
| 4 | Testing & Validation | 1 hour |
| **Total** | **Full Implementation** | **4-6 hours** |

---

## Success Criteria

✅ New users can signup with email/password
✅ Background questions are collected during signup
✅ Users can signin with credentials
✅ Sessions persist across page navigation
✅ Personalization button is visible only to logged-in users
✅ Users can update their background information
✅ All data is stored securely in Neon DB

---

**Ready to implement?** Run `/sp.tasks` to break this down into atomic, testable tasks.
