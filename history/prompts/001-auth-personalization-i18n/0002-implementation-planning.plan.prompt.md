---
id: 0002
title: Authentication Personalization Localization Implementation Planning
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: 001-auth-personalization-i18n
branch: 001-auth-personalization-i18n
user: User
command: /sp.plan
labels: ["authentication", "personalization", "localization", "planning", "architecture", "better-auth", "gemini", "openai"]
links:
  spec: specs/001-auth-personalization-i18n/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-auth-personalization-i18n/plan.md
  - specs/001-auth-personalization-i18n/research.md
  - specs/001-auth-personalization-i18n/data-model.md
  - specs/001-auth-personalization-i18n/quickstart.md
  - specs/001-auth-personalization-i18n/contracts/auth.openapi.yaml
  - specs/001-auth-personalization-i18n/contracts/background.openapi.yaml
  - specs/001-auth-personalization-i18n/contracts/personalization.openapi.yaml
  - specs/001-auth-personalization-i18n/contracts/translation.openapi.yaml
tests:
  - Constitution Check: All 10 gates passed (no violations)
  - Technical Context validation: All fields populated with concrete values
---

## Prompt

(No additional arguments provided - standard /sp.plan workflow executed)

## Response snapshot

Completed comprehensive implementation planning for authentication, personalization, and localization features:

**Phase 0 - Research (Completed)**:
Generated research.md with 10 major research areas:
1. Authentication Architecture: Custom Better-Auth compatible backend with session-based auth (bcrypt, HTTP-only cookies)
2. Database Schema Design: Separate tables for users, sessions, software_background, hardware_background (all with UUID primary keys, proper foreign keys)
3. AI Personalization Strategy: Google Gemini for content personalization with fallback to Claude/OpenAI
4. Translation Architecture: OpenAI GPT-4 for Urdu translation (migrated from Claude for quality)
5. Frontend State Management: React Context API for auth and personalization (no Redux needed)
6. Performance Optimization: Multi-layer caching (localStorage frontend, optional backend cache)
7. Testing Strategy: Pytest (backend) + Jest/React Testing Library (frontend)
8. Security Considerations: Defense-in-depth with password hashing, session tokens, SQL injection prevention
9. Deployment Strategy: Extends existing Vercel/Railway + GitHub Pages architecture
10. Gap Analysis: ~80% implementation complete, primary gaps are /personalize endpoint and frontend buttons

**Phase 1 - Design & Contracts (Completed)**:

1. **Data Model** (data-model.md):
   - Documented 4 persistent entities: User, Session, SoftwareBackground, HardwareBackground
   - Documented 4 runtime entities: PersonalizationRequest, PersonalizedContent, TranslationRequest, TranslatedContent
   - Entity Relationship Diagram with 1:N (user:sessions) and 1:1 (user:backgrounds) relationships
   - Migration strategy with 6 SQL files (001-006)
   - Query patterns for authentication, session validation, profile retrieval
   - Performance indexes on foreign keys, email, token, expires_at

2. **API Contracts** (contracts/*.openapi.yaml):
   - **auth.openapi.yaml**: 5 endpoints (POST /auth/signup, POST /auth/signin, POST /auth/signout, GET /auth/session, DELETE /auth/session)
   - **background.openapi.yaml**: 2 endpoints (GET /background, POST /background)
   - **personalization.openapi.yaml**: 1 endpoint (POST /personalize) - NEW, needs implementation
   - **translation.openapi.yaml**: 1 endpoint (POST /translate) - already exists
   - All contracts include request/response schemas, error responses, security schemes (cookie auth)

3. **Quick Start Guide** (quickstart.md):
   - Architecture overview with frontend/backend component diagram
   - Environment setup instructions (.env configuration, secret generation)
   - Database migration steps (run_migrations.py)
   - Development server startup commands
   - Implementation checklist: ✅ Already implemented (18 items), 🆕 To be implemented (8 items)
   - Code examples for key implementation tasks:
     * Backend /personalize endpoint (routes/personalize.py)
     * Frontend PersonalizationButton component
     * Frontend TranslationButton component
   - Testing guide with pytest and Jest examples
   - Verification checklist (authentication, background profile, personalization, translation, RAG integration)
   - Deployment commands and troubleshooting

4. **Implementation Plan** (plan.md):
   - **Summary**: Integrates auth via Better-Auth, AI personalization via Gemini, Urdu translation via OpenAI
   - **Technical Context**: Python 3.8+/TypeScript 5.2.2, FastAPI 0.109.0/Docusaurus 3.9.0, Neon Postgres/Qdrant, performance goals (<500ms auth, <10s personalization/translation)
   - **Constitution Check**: ✅ ALL 10 GATES PASS - Specification-first, AI-native, deterministic structure, reusability, transparency, dual QA mode, full deployment, performance, reliability, hackathon compliance
   - **Project Structure**: Web application with backend/ and frontend/my-book/ directories, most components already exist
   - **Complexity Tracking**: N/A - No constitutional violations

**Implementation Status**:
- **Existing (80%)**: Auth backend, database schema, migrations, personalization agent, translation endpoint, frontend auth components, i18n config
- **Gaps (20%)**: /personalize endpoint, PersonalizationButton component, TranslationButton component, test coverage

**Key Architectural Decisions**:
1. Better-Auth compatible backend (not full Better-Auth npm package) for session-based auth
2. Separate database tables for software/hardware backgrounds (vs JSONB column)
3. Google Gemini for personalization (cost-effective for long-context) with fallback
4. OpenAI GPT-4 for translation (migrated from Claude for quality)
5. React Context API for state management (no Redux)
6. Multi-layer caching (frontend localStorage + optional backend)
7. Runtime-only personalization/translation (does NOT modify source files)
8. RAG chatbot operates exclusively on original content (preserves citation accuracy)

**Next Steps**: Run /sp.tasks to generate detailed implementation tasks for the remaining 20% of work.

## Outcome

- ✅ Impact: Comprehensive implementation plan with research, data model, API contracts, and quick start guide covering all aspects of authentication, personalization, and localization
- 🧪 Tests: Constitution Check validated all 10 gates pass with no violations; technical context fully populated with concrete technologies and versions
- 📁 Files: Created plan.md (183 lines), research.md (comprehensive 10-section analysis), data-model.md (detailed schema with ERD), quickstart.md (step-by-step implementation guide), 4 OpenAPI contract files
- 🔁 Next prompts: /sp.tasks (generate detailed implementation tasks), /sp.implement (execute implementation), /sp.clarify (if business questions arise)
- 🧠 Reflection: Feature is ~80% complete per exploration results; planning identified precise gaps (/personalize endpoint, 2 frontend components, tests); all architectural decisions align with existing infrastructure; no refactoring or new technologies required

## Evaluation notes (flywheel)

- Failure modes observed: None - all phases completed successfully; initial script parameter error self-corrected by branch creation
- Graders run and results (PASS/FAIL): PASS - Constitution Check all gates passed, Technical Context fully specified, API contracts complete with proper OpenAPI 3.0 format, data model includes ERD and relationships
- Prompt variant (if applicable): Standard /sp.plan workflow with codebase exploration via Task tool (subagent_type=Explore)
- Next experiment (smallest change to try): Consider breaking quickstart into separate deployment guide if implementation complexity increases (currently manageable as single document)
