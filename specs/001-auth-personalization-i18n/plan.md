# Implementation Plan: Authentication, Personalization, and Localization

**Branch**: `001-auth-personalization-i18n` | **Date**: 2025-12-18 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-auth-personalization-i18n/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature integrates user authentication via Better-Auth, AI-driven content personalization based on user software and hardware backgrounds, and Urdu translation capabilities into the existing AI-driven book platform. The implementation leverages existing FastAPI backend and Docusaurus frontend infrastructure to deliver runtime-adaptive learning experiences without modifying source content. User profiles stored in Neon Postgres drive personalization through Gemini AI, while OpenAI powers translation, all while maintaining RAG chatbot citation accuracy by operating on original content.

## Technical Context

**Language/Version**: Python 3.8+ (Backend), TypeScript 5.2.2 (Frontend), Node.js >=18.0
**Primary Dependencies**:
- Backend: FastAPI 0.109.0, Uvicorn 0.27.0, Anthropic 0.18.0, Google Generative AI 0.3.2, OpenAI 1.10.0, Qdrant Client 1.7.3, psycopg2, python-dotenv, Pydantic 2.5.3, passlib (bcrypt), tiktoken 0.5.2
- Frontend: Docusaurus 3.9.0, React 18.0.0, TypeScript 5.2.2, MDX 3.0.0

**Storage**: Neon Serverless PostgreSQL (users, sessions, software_background, hardware_background tables with UUID primary keys and foreign key relationships), Qdrant Vector Database (book embeddings for RAG)

**Testing**: pytest (backend), Jest/React Testing Library (frontend), integration tests for auth flow and personalization

**Target Platform**: Web application (Linux server for backend, browser-based frontend), Vercel/Railway deployment, GitHub Pages for static frontend

**Project Type**: Web application with separate backend (FastAPI) and frontend (Docusaurus + React)

**Performance Goals**:
- Authentication: <500ms response for signup/signin
- Personalization: <10 seconds for chapter adaptation (per spec SC-004)
- Translation: <10 seconds for chapter translation (per spec SC-005)
- RAG queries: <2 seconds p95 latency (existing spec SC-002)
- Session persistence: 100% consistency across components (per spec SC-003)

**Constraints**:
- Runtime-only personalization and translation (MUST NOT modify source files per spec FR-013)
- RAG chatbot MUST operate on original content only (per spec FR-025, FR-026)
- Better-Auth compatible implementation required (per spec FR-001, FR-002)
- Session cookies: HttpOnly, Secure, SameSite=Lax
- Password requirements: min 8 chars, 1 uppercase, 1 lowercase, 1 digit
- Default session: 24 hours, remember me: 30 days

**Scale/Scope**:
- Multiple concurrent users with independent personalized content (per spec SC-011)
- Support for 3 software experience levels (Beginner/Intermediate/Advanced)
- Urdu language translation (primary target), extensible to other languages
- User profiles with software and hardware background data
- Chapter-level personalization and translation (not book-wide)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Specification-First Workflow
✅ **PASS** - Implementation is driven by spec.md in specs/001-auth-personalization-i18n/ with 27 functional requirements and 5 user stories.

### AI-Native Development
✅ **PASS** - Leverages existing AI infrastructure (Claude, Gemini, OpenAI) for personalization and translation agents. Extends current subagent architecture.

### Deterministic Structure
✅ **PASS** - Database schema is well-defined (4 new tables: users, sessions, software_background, hardware_background). Backend API contracts follow existing FastAPI patterns. Frontend extends Docusaurus structure without breaking existing layout.

### Reusability
✅ **PASS** - Personalization agent and translation agent are designed as reusable services that can be invoked for any chapter content. Authentication middleware can be applied to any protected route.

### Transparency
✅ **PASS** - RAG chatbot continues to operate on original content with citations preserved (FR-025, FR-026). Personalized/translated content does not interfere with citation accuracy.

### Dual QA Mode
✅ **PASS** - Existing global QA and selected-text QA modes are preserved. Authentication context is available but does not change RAG behavior per spec.

### Full Deployment
✅ **PASS** - Implementation extends existing deployment architecture: FastAPI backend to Vercel/Railway, Docusaurus frontend to GitHub Pages. No new deployment targets required.

### Performance
✅ **PASS** - Performance goals align with existing infrastructure: <500ms auth, <10s personalization/translation, <2s RAG queries. Existing Qdrant vector search and chunking strategies remain unchanged.

### Reliability
✅ **PASS** - Database migrations provide predictable schema evolution. Session management with expiration ensures reliable auth state. Error handling for AI agent failures included.

### Hackathon Compliance
✅ **PASS** - Feature enhances the existing hackathon deliverable (AI-driven book + RAG chatbot) by adding user-specific personalization without breaking core functionality.

**Overall Status**: ✅ ALL GATES PASS - No violations. Feature aligns with all constitutional principles and extends existing architecture cleanly.

## Project Structure

### Documentation (this feature)

```text
specs/001-auth-personalization-i18n/
├── spec.md              # Feature specification (created by /sp.specify)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── auth.openapi.yaml
│   ├── background.openapi.yaml
│   ├── personalization.openapi.yaml
│   └── translation.openapi.yaml
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                    # FastAPI app entry point (extend with new routes)
│   ├── config/
│   │   ├── auth_config.py         # Authentication configuration (✅ ALREADY EXISTS)
│   │   └── ai_config.py           # AI model configuration (extend for personalization/translation)
│   ├── database/
│   │   └── connection.py          # Neon Postgres connection pool (✅ ALREADY EXISTS)
│   ├── models/
│   │   ├── user.py                # User model (✅ ALREADY EXISTS)
│   │   ├── session.py             # Session model (✅ ALREADY EXISTS)
│   │   ├── background.py          # Combined background model (✅ ALREADY EXISTS)
│   │   ├── software_background.py # Software preferences (✅ ALREADY EXISTS)
│   │   └── hardware_background.py # Hardware preferences (✅ ALREADY EXISTS)
│   ├── routes/
│   │   ├── auth.py                # Auth endpoints: /signup, /signin, /signout (✅ ALREADY EXISTS)
│   │   ├── background.py          # Background endpoints: GET/POST /background (✅ ALREADY EXISTS)
│   │   ├── translate.py           # Translation endpoint: POST /translate (✅ ALREADY EXISTS)
│   │   └── personalize.py         # Personalization endpoint: POST /personalize (🆕 TO CREATE)
│   ├── services/
│   │   ├── user_service.py        # User CRUD operations (✅ ALREADY EXISTS)
│   │   ├── session_service.py     # Session management (✅ ALREADY EXISTS)
│   │   └── background_service.py  # Background data management (✅ ALREADY EXISTS)
│   ├── agents/
│   │   └── personalization_agent.py # Content personalization engine (✅ ALREADY EXISTS)
│   ├── middleware/
│   │   └── auth_middleware.py     # Request authentication (✅ ALREADY EXISTS)
│   └── utils/
│       └── validators.py          # Input validation utilities (✅ ALREADY EXISTS)
├── migrations/
│   ├── 001_create_users.sql       # ✅ ALREADY EXISTS
│   ├── 002_create_sessions.sql    # ✅ ALREADY EXISTS
│   ├── 003_create_software_background.sql # ✅ ALREADY EXISTS
│   ├── 004_create_hardware_background.sql # ✅ ALREADY EXISTS
│   └── run_migrations.py          # Migration runner (✅ ALREADY EXISTS)
├── tests/
│   ├── test_auth.py               # Auth flow tests (🆕 TO CREATE)
│   ├── test_personalization.py    # Personalization tests (🆕 TO CREATE)
│   └── test_translation.py        # Translation tests (🆕 TO CREATE)
└── requirements.txt               # ✅ ALREADY EXISTS (may need updates)

frontend/my-book/
├── src/
│   ├── contexts/
│   │   ├── AuthProvider.tsx       # Auth state management (✅ ALREADY EXISTS)
│   │   └── PersonalizationContext.tsx # Personalization state (✅ ALREADY EXISTS)
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupForm.tsx     # Signup form (✅ ALREADY EXISTS)
│   │   │   ├── SigninForm.tsx     # Signin form (✅ ALREADY EXISTS)
│   │   │   ├── BackgroundQuestionsForm.tsx # Background questions (✅ ALREADY EXISTS)
│   │   │   └── AuthNavbarItem.tsx # Auth navbar item (✅ ALREADY EXISTS)
│   │   ├── PersonalizationButton/  # Personalization button (🆕 TO VERIFY/CREATE)
│   │   └── TranslationButton/      # Translation button (🆕 TO VERIFY/CREATE)
│   └── utils/
│       ├── authClient.ts          # Auth API utilities (✅ ALREADY EXISTS)
│       └── config.ts              # Configuration utilities (✅ ALREADY EXISTS)
├── i18n/
│   └── ur/                       # Urdu translations (✅ ALREADY EXISTS)
├── docusaurus.config.ts           # ✅ ALREADY EXISTS (i18n configured)
└── package.json                   # ✅ ALREADY EXISTS
```

**Structure Decision**: Web application architecture with separate backend (FastAPI) and frontend (Docusaurus/React). Most components are already implemented based on exploration results. Primary implementation work will focus on:
1. **Backend**: Create personalization endpoint (POST /personalize) to connect PersonalizationAgent with REST API
2. **Frontend**: Verify and potentially create PersonalizationButton and TranslationButton components
3. **Testing**: Add comprehensive test coverage for auth flows, personalization, and translation
4. **Integration**: Ensure authentication state flows correctly between all components

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - No constitutional violations. All gates pass cleanly.
