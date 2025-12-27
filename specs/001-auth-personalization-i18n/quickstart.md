# Quick Start: Authentication, Personalization, and Localization

**Feature**: 001-auth-personalization-i18n
**Date**: 2025-12-18
**Audience**: Developers implementing the feature

## Overview

This quick start guide helps developers understand and implement the authentication, personalization, and localization features. Most components already exist - this guide focuses on the remaining implementation work.

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
- Google API key (Gemini for personalization)
- OpenAI API key (GPT-4 for translation)
- Anthropic API key (fallback LLM, optional)

**Codebase Knowledge**:
- Familiarity with FastAPI framework
- Understanding of React and Docusaurus
- Basic knowledge of PostgreSQL and migrations

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend (Docusaurus + React)         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  AuthProvider (Context)                              │  │
│  │  - User authentication state                          │  │
│  │  - Session management                                │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  PersonalizationContext (Context)                    │  │
│  │  - User preferences                                   │  │
│  │  - Personalized content cache (localStorage)         │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  UI Components                                        │  │
│  │  - SignupForm / SigninForm                           │  │
│  │  - BackgroundQuestionsForm                           │  │
│  │  - PersonalizationButton (TO VERIFY/CREATE)          │  │
│  │  - TranslationButton (TO VERIFY/CREATE)              │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ HTTPS + Cookies
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      Backend (FastAPI)                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Authentication Middleware                            │  │
│  │  - Session validation                                 │  │
│  │  - Cookie parsing                                     │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Routes                                               │  │
│  │  - /auth/* (signup, signin, signout) ✅ EXISTS       │  │
│  │  - /background (GET/POST) ✅ EXISTS                  │  │
│  │  - /translate (POST) ✅ EXISTS                       │  │
│  │  - /personalize (POST) 🆕 TO CREATE                  │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Services                                             │  │
│  │  - UserService (CRUD) ✅ EXISTS                      │  │
│  │  - SessionService (mgmt) ✅ EXISTS                   │  │
│  │  - BackgroundService (profile) ✅ EXISTS             │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  AI Agents                                            │  │
│  │  - PersonalizationAgent ✅ EXISTS                    │  │
│  │  - TranslationAgent (via /translate) ✅ EXISTS       │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  Neon PostgreSQL Database                    │
│  - users                                                     │
│  - sessions                                                  │
│  - software_background                                       │
│  - hardware_background                                       │
└─────────────────────────────────────────────────────────────┘
```

---

## Setup Steps

### 1. Environment Configuration

Create or update `.env` file in the project root:

```bash
# Database
NEON_DB_URL=postgresql://user:password@host/database?sslmode=require

# Authentication
BETTERAUTH_SECRET=your-secret-key-min-32-characters-random-string

# AI Services
GOOGLE_API_KEY=your-google-api-key-for-gemini
OPENAI_API_KEY=your-openai-api-key-for-translation
ANTHROPIC_API_KEY=your-anthropic-api-key-optional

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

Run all migrations to set up the database schema:

```bash
cd backend
python migrations/run_migrations.py
```

**Verify Migrations**:
```sql
-- Connect to Neon DB
psql $NEON_DB_URL

-- Check tables exist
\dt

-- Expected tables:
-- - users
-- - sessions
-- - software_background
-- - hardware_background
```

### 3. Install Dependencies

**Backend**:
```bash
cd backend
pip install -r requirements.txt
```

**Frontend**:
```bash
cd frontend/my-book
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

### ✅ Already Implemented (No Action Needed)

**Backend**:
- [x] Authentication endpoints (/auth/signup, /auth/signin, /auth/signout, /auth/session)
- [x] User and session models
- [x] Password hashing with bcrypt
- [x] Session management with cookies
- [x] Authentication middleware
- [x] Database schema and migrations
- [x] Background endpoints (/background GET/POST)
- [x] Translation endpoint (/translate POST)
- [x] PersonalizationAgent class

**Frontend**:
- [x] AuthProvider context
- [x] PersonalizationContext
- [x] SignupForm component
- [x] SigninForm component
- [x] BackgroundQuestionsForm component
- [x] AuthNavbarItem component
- [x] Docusaurus i18n configuration (Urdu)

### 🆕 To Be Implemented

**Backend**:
- [ ] Create `/personalize` endpoint in `backend/src/routes/personalize.py`
- [ ] Register personalize route in `backend/src/main.py`
- [ ] Add authentication requirement to personalize endpoint
- [ ] Write tests: `backend/tests/test_auth.py`, `test_personalization.py`, `test_translation.py`

**Frontend**:
- [ ] Verify PersonalizationButton component exists (check `frontend/my-book/src/components/`)
- [ ] If not, create PersonalizationButton component
- [ ] Verify TranslationButton component exists
- [ ] If not, create TranslationButton component
- [ ] Integrate buttons at chapter level (docs MDX files or Docusaurus theme)
- [ ] Write component tests (Jest + React Testing Library)

**Integration**:
- [ ] End-to-end test: Signup → Background questions → Personalize chapter
- [ ] End-to-end test: Signin → Translate chapter
- [ ] Verify authentication state persists across book and chatbot
- [ ] Test personalized content caching in localStorage

---

## Key Implementation Tasks

### Task 1: Backend Personalize Endpoint

**File**: `backend/src/routes/personalize.py`

**Implementation**:
```python
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional
from ..middleware.auth_middleware import get_current_user
from ..services.background_service import get_user_background
from ..agents.personalization_agent import PersonalizationAgent
import time

router = APIRouter(prefix="/personalize", tags=["personalization"])

class PersonalizeRequest(BaseModel):
    chapter_content: str
    chapter_id: str
    learning_style: Optional[str] = "multimodal"

class PersonalizeResponse(BaseModel):
    personalized_content: str
    personalization_applied: dict
    processing_time_ms: float
    model_used: str = "gemini-pro"
    cached: bool = False

@router.post("/", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    current_user: dict = Depends(get_current_user)
):
    """
    Personalize chapter content based on user's background profile.
    Requires authentication.
    """
    start_time = time.time()

    # Get user background
    user_id = current_user["id"]
    background = await get_user_background(user_id)

    if not background:
        raise HTTPException(
            status_code=404,
            detail="User background profile not found. Please complete background questions first."
        )

    # Initialize personalization agent
    agent = PersonalizationAgent()

    # Prepare user preferences
    preferences = {
        "software_experience": background["software"]["experience_level"],
        "hardware_experience": background["hardware"]["experience_level"],
        "preferred_languages": background["software"].get("preferred_languages", []),
        "preferred_frameworks": background["software"].get("preferred_frameworks", []),
        "preferred_platforms": background["hardware"].get("preferred_platforms", []),
        "learning_style": request.learning_style
    }

    # Generate personalized content
    personalized = await agent.personalize(
        content=request.chapter_content,
        preferences=preferences
    )

    processing_time = (time.time() - start_time) * 1000

    return PersonalizeResponse(
        personalized_content=personalized,
        personalization_applied={
            "software_level": preferences["software_experience"],
            "hardware_level": preferences["hardware_experience"],
            "learning_style": request.learning_style,
            "preferred_languages": preferences["preferred_languages"],
            "preferred_frameworks": preferences["preferred_frameworks"]
        },
        processing_time_ms=processing_time,
        model_used="gemini-pro",
        cached=False
    )
```

**Register in `backend/src/main.py`**:
```python
from src.routes import personalize

app.include_router(personalize.router)
```

### Task 2: Frontend PersonalizationButton

**File**: `frontend/my-book/src/components/PersonalizationButton/index.tsx`

**Implementation**:
```typescript
import React, { useState } from 'react';
import { useAuth } from '@site/src/contexts/AuthProvider';
import { usePersonalization } from '@site/src/contexts/PersonalizationContext';
import styles from './styles.module.css';

interface PersonalizationButtonProps {
  chapterId: string;
  content: string;
  onPersonalized: (personalizedContent: string) => void;
}

export default function PersonalizationButton({
  chapterId,
  content,
  onPersonalized
}: PersonalizationButtonProps) {
  const { isAuthenticated } = useAuth();
  const { personalizeContent, isLoading } = usePersonalization();
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      alert('Please sign in to personalize content');
      return;
    }

    setError(null);
    try {
      const personalized = await personalizeContent(chapterId, content);
      onPersonalized(personalized);
    } catch (err) {
      setError(err.message || 'Personalization failed');
    }
  };

  if (!isAuthenticated) {
    return null; // Hide button for unauthenticated users
  }

  return (
    <div className={styles.personalizationButton}>
      <button
        onClick={handlePersonalize}
        disabled={isLoading}
        className="button button--primary"
      >
        {isLoading ? 'Personalizing...' : 'Personalize this chapter'}
      </button>
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
}
```

### Task 3: Frontend TranslationButton

**File**: `frontend/my-book/src/components/TranslationButton/index.tsx`

**Implementation**:
```typescript
import React, { useState } from 'react';
import styles from './styles.module.css';

interface TranslationButtonProps {
  chapterId: string;
  content: string;
  onTranslated: (translatedContent: string, language: string) => void;
}

export default function TranslationButton({
  chapterId,
  content,
  onTranslated
}: TranslationButtonProps) {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [currentLanguage, setCurrentLanguage] = useState<'en' | 'ur'>('en');

  const handleTranslate = async () => {
    const targetLanguage = currentLanguage === 'en' ? 'ur' : 'en';

    if (targetLanguage === 'en') {
      // Switch back to original
      onTranslated(content, 'en');
      setCurrentLanguage('en');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${process.env.REACT_APP_BACKEND_URL}/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          content,
          target_language: 'ur',
          chapter_id: chapterId
        })
      });

      if (!response.ok) throw new Error('Translation failed');

      const data = await response.json();
      onTranslated(data.translated_content, 'ur');
      setCurrentLanguage('ur');
    } catch (err) {
      setError(err.message || 'Translation failed');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.translationButton}>
      <button
        onClick={handleTranslate}
        disabled={isLoading}
        className="button button--secondary"
      >
        {isLoading
          ? 'Translating...'
          : currentLanguage === 'en'
            ? 'Translate to Urdu'
            : 'Show Original'}
      </button>
      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
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

def test_signup_success():
    response = client.post("/auth/signup", json={
        "email": "test@example.com",
        "password": "SecurePass123"
    })
    assert response.status_code == 201
    assert "user" in response.json()

def test_signin_success():
    # First signup
    client.post("/auth/signup", json={
        "email": "test2@example.com",
        "password": "SecurePass123"
    })

    # Then signin
    response = client.post("/auth/signin", json={
        "email": "test2@example.com",
        "password": "SecurePass123"
    })
    assert response.status_code == 200
    assert "session_token" in response.cookies

def test_personalize_requires_auth():
    response = client.post("/personalize", json={
        "chapter_content": "# Test Chapter",
        "chapter_id": "test-01"
    })
    assert response.status_code == 401
```

**Run Backend Tests**:
```bash
cd backend
pytest tests/ -v
```

### Frontend Tests

**File**: `frontend/my-book/src/components/PersonalizationButton/__tests__/index.test.tsx`

```typescript
import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import PersonalizationButton from '../index';

describe('PersonalizationButton', () => {
  it('renders button for authenticated users', () => {
    render(
      <PersonalizationButton
        chapterId="test-01"
        content="# Test"
        onPersonalized={jest.fn()}
      />
    );
    expect(screen.getByText('Personalize this chapter')).toBeInTheDocument();
  });

  it('hides button for unauthenticated users', () => {
    // Mock useAuth to return isAuthenticated=false
    // ...
  });
});
```

**Run Frontend Tests**:
```bash
cd frontend/my-book
npm test
```

---

## Verification Checklist

After implementation, verify:

**Authentication**:
- [ ] Can sign up with valid email/password
- [ ] Signup validation rejects weak passwords
- [ ] Signup prevents duplicate emails
- [ ] Can sign in with correct credentials
- [ ] Signin fails with incorrect credentials
- [ ] Session persists across page reloads
- [ ] Can sign out successfully
- [ ] Session cookie is HttpOnly and Secure

**Background Profile**:
- [ ] Can submit software experience level
- [ ] Can submit hardware experience level
- [ ] Background data saved to database
- [ ] Can retrieve background via GET /background

**Personalization**:
- [ ] PersonalizationButton appears on chapters (when authenticated)
- [ ] Clicking button triggers personalization
- [ ] Personalized content reflects user's software level
- [ ] Personalized content reflects user's hardware level
- [ ] Personalized content cached in localStorage
- [ ] Returning to chapter shows original content (not personalized)

**Translation**:
- [ ] TranslationButton appears on chapters
- [ ] Clicking button translates to Urdu
- [ ] Code blocks remain in English
- [ ] Technical terms remain in English
- [ ] Can toggle back to original language
- [ ] Translation completes within 10 seconds (per spec SC-005)

**RAG Chatbot Integration**:
- [ ] Chatbot still operates on original content
- [ ] Citations remain accurate after personalization
- [ ] Authentication state available in chatbot

---

## Deployment

### Environment Setup

**Production Environment Variables**:
```bash
NEON_DB_URL=<production-database-url>
BETTERAUTH_SECRET=<strong-production-secret>
GOOGLE_API_KEY=<production-api-key>
OPENAI_API_KEY=<production-api-key>
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
2. Test signup → background questions → personalization flow
3. Verify session persistence
4. Test translation functionality
5. Confirm RAG chatbot still works with citations

---

## Troubleshooting

### Common Issues

**Issue**: "Database connection failed"
**Solution**: Verify NEON_DB_URL is correct and includes `?sslmode=require`

**Issue**: "Session token not set"
**Solution**: Check CORS configuration allows credentials, ensure Secure flag matches HTTPS

**Issue**: "Personalization service unavailable"
**Solution**: Verify GOOGLE_API_KEY is valid and has Gemini API access

**Issue**: "Translation timeout"
**Solution**: Check OPENAI_API_KEY is valid, consider increasing timeout for large chapters

**Issue**: "Background profile not found"
**Solution**: Ensure user completed background questions after signup

---

## Next Steps

After completing this quick start:
1. Run `/sp.tasks` to generate detailed implementation tasks
2. Implement missing components (personalize endpoint, buttons)
3. Write comprehensive tests
4. Deploy to staging environment
5. Conduct user acceptance testing
6. Deploy to production

---

## Resources

**Documentation**:
- [Feature Specification](spec.md)
- [Implementation Plan](plan.md)
- [Data Model](data-model.md)
- [API Contracts](contracts/)

**External References**:
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [React Documentation](https://react.dev/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [Better-Auth Documentation](https://www.better-auth.com/docs)
- [Neon Documentation](https://neon.tech/docs)

**Support**:
- Create issues in project repository
- Review existing PHRs in `history/prompts/001-auth-personalization-i18n/`
