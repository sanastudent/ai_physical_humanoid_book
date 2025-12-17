# BetterAuth Authentication System - Backend Implementation

This document provides comprehensive documentation for the BetterAuth-compatible authentication system implemented in the FastAPI backend.

## Overview

The authentication system provides:
- Email/password authentication with secure password hashing (bcrypt)
- Session-based authentication with HTTP-only cookies
- User background collection (software and hardware preferences)
- Multi-step signup flow (credentials → background questions)
- PostgreSQL database with Neon DB

## Architecture

### Database Schema

```
users
├── id (UUID, primary key)
├── email (TEXT, unique)
├── password_hash (TEXT)
├── created_at (TIMESTAMP)
└── updated_at (TIMESTAMP)

sessions
├── id (UUID, primary key)
├── user_id (UUID, foreign key → users.id, CASCADE)
├── token (TEXT, unique, indexed)
├── expires_at (TIMESTAMP, indexed)
├── created_at (TIMESTAMP)
└── updated_at (TIMESTAMP)

software_background
├── id (UUID, primary key)
├── user_id (UUID, unique, foreign key → users.id, CASCADE)
├── experience_level (TEXT CHECK: beginner|intermediate|advanced)
├── preferred_languages (TEXT[], GIN index)
├── preferred_frameworks (TEXT[], GIN index)
├── created_at (TIMESTAMP)
└── updated_at (TIMESTAMP)

hardware_background
├── id (UUID, primary key)
├── user_id (UUID, unique, foreign key → users.id, CASCADE)
├── experience_level (TEXT CHECK: beginner|intermediate|advanced)
├── preferred_platforms (TEXT[], GIN index)
├── device_types (TEXT[], GIN index)
├── created_at (TIMESTAMP)
└── updated_at (TIMESTAMP)
```

### Service Layer

- **UserService** (`backend/src/services/user_service.py`)
  - `create_user()`: Create new user with email validation and password hashing
  - `authenticate_user()`: Verify credentials and return user data
  - `check_email_exists()`: Check for duplicate email registration
  - `get_user_by_id()`: Retrieve user by UUID
  - `get_user_by_email()`: Retrieve user by email address

- **SessionService** (`backend/src/services/session_service.py`)
  - `create_session()`: Generate secure token and create session (24h or 30d)
  - `get_current_session()`: Validate session token and check expiration
  - `invalidate_session()`: Delete session (signout)
  - `invalidate_user_sessions()`: Delete all sessions for user (logout all devices)
  - `get_user_sessions()`: Retrieve all active sessions for user
  - `cleanup_expired_sessions()`: Periodic cleanup (cron job)

- **BackgroundService** (`backend/src/services/background_service.py`)
  - `create_background()`: Create user background data (software + hardware)
  - `get_user_background()`: Retrieve user's background preferences
  - `update_background()`: Update existing background data
  - `check_background_exists()`: Verify background data exists
  - `delete_background()`: Remove background data

### Security Features

1. **Password Security**
   - Bcrypt hashing with salt rounds=10
   - Minimum 8 characters, 1 uppercase, 1 lowercase, 1 digit
   - Password never stored in plain text or returned in API responses

2. **Session Security**
   - 256-bit cryptographically random session tokens (secrets.token_hex(32))
   - HTTP-only cookies (not accessible via JavaScript)
   - Secure flag (HTTPS only in production)
   - SameSite=Lax (CSRF protection)
   - Automatic expiration and cleanup

3. **Database Security**
   - Connection pooling (min=5, max=20)
   - Context managers for safe connection handling
   - Foreign key constraints with CASCADE delete
   - Transaction support with automatic rollback on error

## API Endpoints

### Authentication Routes (`/auth`)

#### POST /auth/signup
Create a new user account (Step 1 of signup flow).

**Request:**
```json
{
  "email": "user@example.com",
  "password": "StrongPassword123"
}
```

**Response (201):**
```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "created_at": "2025-01-15T10:00:00Z",
  "updated_at": "2025-01-15T10:00:00Z"
}
```
+ Sets `session_token` cookie

**Errors:**
- 400: Invalid email format, weak password, or duplicate email
- 500: Server error

---

#### POST /auth/signin
Sign in with email and password.

**Request:**
```json
{
  "email": "user@example.com",
  "password": "StrongPassword123",
  "remember_me": false
}
```

**Response (200):**
```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "created_at": "2025-01-15T10:00:00Z",
  "updated_at": "2025-01-15T10:00:00Z"
}
```
+ Sets `session_token` cookie (24h or 30d if remember_me=true)

**Errors:**
- 401: Invalid credentials
- 500: Server error

---

#### GET /auth/session
Get current user session (requires authentication).

**Response (200):**
```json
{
  "id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "created_at": "2025-01-15T10:00:00Z",
  "updated_at": "2025-01-15T10:00:00Z"
}
```

**Errors:**
- 401: Not authenticated or session expired

---

#### POST /auth/signout
Sign out and invalidate current session.

**Response:** 204 No Content
+ Clears `session_token` cookie

**Errors:**
- 401: No active session

---

#### DELETE /auth/session
Delete all sessions for current user (logout from all devices).

**Authentication Required**

**Response:** 204 No Content

**Errors:**
- 401: Not authenticated

---

### Background Routes (`/background`)

#### POST /background
Create background data for authenticated user (Step 2 of signup flow).

**Authentication Required**

**Request:**
```json
{
  "software": {
    "experience_level": "intermediate",
    "preferred_languages": ["Python", "JavaScript", "Go"],
    "preferred_frameworks": ["FastAPI", "React"]
  },
  "hardware": {
    "experience_level": "beginner",
    "preferred_platforms": ["web", "desktop"],
    "device_types": ["laptop", "smartphone"]
  }
}
```

**Response (201):**
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "software": {
    "id": "456e7890-e89b-12d3-a456-426614174000",
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "experience_level": "intermediate",
    "preferred_languages": ["Python", "JavaScript", "Go"],
    "preferred_frameworks": ["FastAPI", "React"],
    "created_at": "2025-01-15T10:00:00Z",
    "updated_at": "2025-01-15T10:00:00Z"
  },
  "hardware": {
    "id": "789e0123-e89b-12d3-a456-426614174000",
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "experience_level": "beginner",
    "preferred_platforms": ["web", "desktop"],
    "device_types": ["laptop", "smartphone"],
    "created_at": "2025-01-15T10:00:00Z",
    "updated_at": "2025-01-15T10:00:00Z"
  }
}
```

**Errors:**
- 400: Validation error or background already exists
- 401: Not authenticated

---

#### GET /background
Get background data for authenticated user.

**Authentication Required**

**Response (200):** Same format as POST /background

**Errors:**
- 404: Background data not found
- 401: Not authenticated

---

#### PUT /background
Update background data for authenticated user.

**Authentication Required**

**Request:** Same format as POST /background

**Response (200):** Same format as POST /background

**Errors:**
- 400: Validation error or background doesn't exist
- 401: Not authenticated

---

#### DELETE /background
Delete background data for authenticated user.

**Authentication Required**

**Response:** 204 No Content

**Errors:**
- 404: Background data not found
- 401: Not authenticated

---

## Setup and Installation

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create `.env` file in `backend/` directory:

```env
# Neon DB PostgreSQL connection
NEON_DB_URL=postgresql://user:password@host:5432/database

# BetterAuth secret (generate with: openssl rand -base64 32)
BETTERAUTH_SECRET=your_32_character_secret_key_here

# Backend configuration (optional)
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
```

### 3. Run Database Migrations

```bash
cd backend/migrations
python run_migrations.py
```

Expected output:
```
[OK] 001_create_users.sql
[OK] 002_create_sessions.sql
[OK] 003_create_software_background.sql
[OK] 004_create_hardware_background.sql
[OK] 005_create_indexes.sql
[OK] 006_create_triggers.sql
All migrations completed successfully!
```

### 4. Start the Backend Server

```bash
cd backend
python -m src.main
```

Server will start on http://localhost:8000

### 5. Verify Installation

Check health endpoint:
```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "AI Book RAG API",
  "version": "1.0.0",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

## Testing

### Run All Tests

```bash
cd backend
pytest tests/test_auth_api.py -v
pytest tests/test_background_api.py -v
```

### Run Specific Test

```bash
pytest tests/test_auth_api.py::test_complete_auth_flow -v
```

### Test Coverage

The test suite includes:

**Authentication Tests** (`test_auth_api.py`):
- ✓ Successful signup with valid credentials
- ✓ Duplicate email prevention
- ✓ Password strength validation (uppercase, lowercase, digit, length)
- ✓ Email format validation
- ✓ Successful signin with correct credentials
- ✓ Invalid credentials rejection
- ✓ Session validation and retrieval
- ✓ Signout and session invalidation
- ✓ Complete authentication flow

**Background Tests** (`test_background_api.py`):
- ✓ Background creation with valid data
- ✓ Authentication requirement enforcement
- ✓ Duplicate background prevention
- ✓ Experience level validation
- ✓ Required array validation (languages, platforms)
- ✓ Background retrieval
- ✓ Background update
- ✓ Background deletion
- ✓ Complete background flow

## Usage Examples

### Example 1: Complete Signup Flow

```python
import httpx

BASE_URL = "http://localhost:8000"

# Step 1: Create account
signup_response = httpx.post(
    f"{BASE_URL}/auth/signup",
    json={
        "email": "alice@example.com",
        "password": "SecurePassword123"
    }
)
print(f"User created: {signup_response.json()['email']}")
session_cookie = signup_response.cookies.get("session_token")

# Step 2: Submit background data
background_response = httpx.post(
    f"{BASE_URL}/background",
    json={
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python", "Go"],
            "preferred_frameworks": ["FastAPI"]
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": ["laptop"]
        }
    },
    cookies={"session_token": session_cookie}
)
print(f"Background created for user: {background_response.json()['user_id']}")
```

### Example 2: Signin and Retrieve Profile

```python
import httpx

BASE_URL = "http://localhost:8000"

# Sign in
signin_response = httpx.post(
    f"{BASE_URL}/auth/signin",
    json={
        "email": "alice@example.com",
        "password": "SecurePassword123",
        "remember_me": True  # 30-day session
    }
)
session_cookie = signin_response.cookies.get("session_token")

# Get user profile
profile_response = httpx.get(
    f"{BASE_URL}/auth/session",
    cookies={"session_token": session_cookie}
)
print(f"User profile: {profile_response.json()}")

# Get user background
background_response = httpx.get(
    f"{BASE_URL}/background",
    cookies={"session_token": session_cookie}
)
print(f"User preferences: {background_response.json()}")
```

### Example 3: Update Preferences

```python
import httpx

BASE_URL = "http://localhost:8000"
session_cookie = "your_session_token_here"

# Update background
update_response = httpx.put(
    f"{BASE_URL}/background",
    json={
        "software": {
            "experience_level": "advanced",  # Upgraded!
            "preferred_languages": ["Python", "Rust", "Go"],  # Added Rust
            "preferred_frameworks": ["FastAPI", "Actix"]
        },
        "hardware": {
            "experience_level": "intermediate",  # Upgraded!
            "preferred_platforms": ["web", "embedded"],
            "device_types": ["laptop", "raspberry-pi"]
        }
    },
    cookies={"session_token": session_cookie}
)
print(f"Updated preferences: {update_response.json()}")
```

## Maintenance

### Cleanup Expired Sessions

Run periodically (e.g., via cron job):

```python
from src.services.session_service import SessionService

deleted_count = SessionService.cleanup_expired_sessions()
print(f"Deleted {deleted_count} expired sessions")
```

### Database Backups

Recommend daily backups of Neon DB:
```bash
pg_dump $NEON_DB_URL > backup_$(date +%Y%m%d).sql
```

## Troubleshooting

### Issue: "NEON_DB_URL environment variable is required"
**Solution:** Ensure `.env` file exists in `backend/` directory with valid `NEON_DB_URL`

### Issue: Migration fails with encoding error
**Solution:** Use `run_migrations.py` script which handles Windows encoding issues

### Issue: Session cookie not being set
**Solution:** Check CORS configuration in `main.py` - ensure `allow_credentials=True`

### Issue: Password validation failing unexpectedly
**Solution:** Verify password meets all requirements:
- Min 8 characters
- At least 1 uppercase letter (A-Z)
- At least 1 lowercase letter (a-z)
- At least 1 digit (0-9)

## Next Steps

- [ ] Implement frontend signup/signin forms
- [ ] Add email verification flow
- [ ] Implement password reset functionality
- [ ] Add rate limiting for auth endpoints
- [ ] Add OAuth providers (Google, GitHub)
- [ ] Implement 2FA (TOTP)

## References

- BetterAuth Documentation: https://better-auth.com/docs
- FastAPI Documentation: https://fastapi.tiangolo.com/
- Neon DB Documentation: https://neon.tech/docs
- Bcrypt Documentation: https://pypi.org/project/bcrypt/
