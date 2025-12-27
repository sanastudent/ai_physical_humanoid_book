---
id: 0028
title: BetterAuth Completion Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-betterauth-completion
branch: 003-betterauth-completion
user: User
command: /sp.plan
labels: [feature, plan, auth, betterauth, security]
links:
  spec: specs/003-betterauth-completion/spec.md
  plan: specs/003-betterauth-completion/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-betterauth-completion/plan.md
  - specs/003-betterauth-completion/research.md
  - specs/003-betterauth-completion/data-model.md
  - specs/003-betterauth-completion/quickstart.md
  - specs/003-betterauth-completion/contracts/auth.openapi.yaml
tests:
  - backend/tests/test_auth.py
  - frontend/tests/auth.test.tsx
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/setup-plan.ps1 -Json` from repo root and parse JSON for FEATURE_SPEC, IMPL_PLAN, SPECS_DIR, BRANCH. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load context**: Read FEATURE_SPEC and `.specify/memory/constitution.md`. Load IMPL_PLAN template (already copied).

3. **Execute plan workflow**: Follow the structure in IMPL_PLAN template to:
   - Fill Technical Context (mark unknowns as "NEEDS CLARIFICATION")
   - Fill Constitution Check section from constitution
   - Evaluate gates (ERROR if violations unjustified)
   - Phase 0: Generate research.md (resolve all NEEDS CLARIFICATION)
   - Phase 1: Generate data-model.md, contracts/, quickstart.md
   - Phase 1: Update agent context by running the agent script
   - Re-evaluate Constitution Check post-design

4. **Stop and report**: Command ends after Phase 2 planning. Report branch, IMPL_PLAN path, and generated artifacts.

## Phases

### Phase 0: Outline & Research

1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:

   ```text
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

### Phase 1: Design & Contracts

**Prerequisites:** `research.md` complete

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Agent context update**:
   - Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`
   - These scripts detect which AI agent is in use
   - Update the appropriate agent-specific context file
   - Add only new technology from current plan
   - Preserve manual additions between markers

**Output**: data-model.md, /contracts/*, quickstart.md, agent-specific file

## Key rules

- Use absolute paths
- ERROR on gate failures or unresolved clarifications

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

# Implementation Plan: BetterAuth Integration Completion

**Branch**: `003-betterauth-completion` | **Date**: 2025-12-18 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/003-betterauth-completion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This feature completes the BetterAuth integration to address gaps identified in analysis. The implementation focuses on creating a backend that is fully compatible with BetterAuth frontend library, implementing proper session management according to BetterAuth standards, and ensuring all authentication flows work correctly with BetterAuth patterns. This addresses the requirement gap identified where the system was using custom auth implementation instead of the required BetterAuth integration.

## Technical Context

**Language/Version**: Python 3.8+ (Backend), TypeScript 5.2.2 (Frontend), Node.js >=18.0
**Primary Dependencies**:
- Backend: FastAPI 0.109.0, Uvicorn 0.27.0, Better-Auth 0.8.0+, python-dotenv, Pydantic 2.5.3, passlib (bcrypt), psycopg2
- Frontend: Docusaurus 3.9.0, React 18.0.0, TypeScript 5.2.2, Better-Auth client library
**Storage**: Neon Serverless PostgreSQL (users, sessions, software_background, hardware_background tables with UUID primary keys and foreign key relationships)
**Testing**: pytest (backend), Jest/React Testing Library (frontend), integration tests for auth flow
**Target Platform**: Web application (Linux server for backend, browser-based frontend), Vercel/Railway deployment, GitHub Pages for static frontend
**Project Type**: Web application with separate backend (FastAPI) and frontend (Docusaurus + React)
**Performance Goals**:
- Authentication: <500ms response for signup/signin
- Session validation: <250ms for token verification
- Session persistence: 100% consistency across components (per spec SC-014)
**Constraints**:
- Must use BetterAuth as authentication provider (per spec FR-039, FR-041)
- All user data must be stored in Neon DB (per spec FR-042)
- Session tokens: HttpOnly, Secure, SameSite=Lax (per spec FR-040)
- Password requirements: min 8 chars, 1 uppercase, 1 lowercase, 1 digit
- Default session: 24 hours, remember me: 30 days
- Maintain backward compatibility for existing accounts (per spec FR-047)
**Scale/Scope**:
- Multiple concurrent users with independent sessions (per spec SC-018)
- Support for existing user migration from custom auth to BetterAuth
- User sessions with proper timeout and refresh mechanisms
- Session management following BetterAuth's security protocols

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Specification-First Workflow
✅ **PASS** - Implementation is driven by spec.md in specs/003-betterauth-completion/ with 12 functional requirements and 4 user stories.

### AI-Native Development
✅ **PASS** - Leverages existing AI infrastructure (Claude, Gemini, OpenAI) for any personalization features that use authentication context. Extends current subagent architecture.

### Deterministic Structure
✅ **PASS** - Database schema is well-defined (4 existing tables: users, sessions, software_background, hardware_background). Backend API contracts follow BetterAuth-compatible patterns.

### Reusability
✅ **PASS** - BetterAuth integration provides reusable authentication service that can be used across all protected routes and features.

### Transparency
✅ **PASS** - Authentication flows maintain transparency with proper session handling and user verification mechanisms.

### Dual QA Mode
✅ **PASS** - Existing global QA and selected-text QA modes are preserved. Authentication context is available but does not change RAG behavior per spec.

### Full Deployment
✅ **PASS** - Implementation extends existing deployment architecture: FastAPI backend to Vercel/Railway, Docusaurus frontend to GitHub Pages. No new deployment targets required.

### Performance
✅ **PASS** - Performance goals align with existing infrastructure: <500ms auth, <250ms session validation. Existing Qdrant vector search and chunking strategies remain unchanged.

### Reliability
✅ **PASS** - Proper session management with expiration ensures reliable auth state. Error handling for authentication failures included.

### Hackathon Compliance
✅ **PASS** - Feature enhances the existing hackathon deliverable (AI-driven book + RAG chatbot) by completing the required authentication integration without breaking core functionality.

**Overall Status**: ✅ ALL GATES PASS - No violations. Feature aligns with all constitutional principles and extends existing architecture cleanly.

## Project Structure

### Documentation (this feature)

```text
specs/003-betterauth-completion/
├── spec.md              # Feature specification (created by /sp.specify)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py                    # FastAPI app entry point (extend with BetterAuth routes)
│   ├── config/
│   │   ├── auth_config.py         # BetterAuth configuration (NEW)
│   │   └── database_config.py     # Neon Postgres configuration
│   ├── database/
│   │   └── connection.py          # Neon Postgres connection pool (✅ ALREADY EXISTS)
│   ├── models/
│   │   ├── user.py                # User model (update for BetterAuth compatibility)
│   │   ├── session.py             # Session model (update for BetterAuth compatibility)
│   │   ├── background.py          # Combined background model (✅ ALREADY EXISTS)
│   │   ├── software_background.py # Software preferences (✅ ALREADY EXISTS)
│   │   └── hardware_background.py # Hardware preferences (✅ ALREADY EXISTS)
│   ├── routes/
│   │   ├── auth.py                # BetterAuth-compatible endpoints: /auth/* (NEW)
│   │   ├── background.py          # Background endpoints: GET/POST /background (✅ ALREADY EXISTS)
│   │   ├── translate.py           # Translation endpoint: POST /translate (✅ ALREADY EXISTS)
│   │   └── personalize.py         # Personalization endpoint: POST /personalize (✅ ALREADY EXISTS)
│   ├── services/
│   │   ├── user_service.py        # User CRUD operations (update for BetterAuth)
│   │   ├── session_service.py     # Session management (update for BetterAuth)
│   │   └── background_service.py  # Background data management (✅ ALREADY EXISTS)
│   ├── auth/
│   │   └── better_auth_adapter.py # BetterAuth backend adapter (NEW)
│   └── utils/
│       └── validators.py          # Input validation utilities (✅ ALREADY EXISTS)
├── migrations/
│   ├── 001_create_users.sql       # ✅ ALREADY EXISTS
│   ├── 002_create_sessions.sql    # ✅ ALREADY EXISTS
│   ├── 003_create_software_background.sql # ✅ ALREADY EXISTS
│   ├── 004_create_hardware_background.sql # ✅ ALREADY EXISTS
│   └── run_migrations.py          # Migration runner (✅ ALREADY EXISTS)
├── tests/
│   ├── test_auth.py               # BetterAuth flow tests (NEW)
│   ├── test_session.py            # Session management tests (NEW)
│   └── test_user_migration.py     # User migration tests (NEW)
└── requirements.txt               # ✅ ALREADY EXISTS (may need BetterAuth updates)

frontend/my-book/
├── src/
│   ├── contexts/
│   │   ├── AuthProvider.tsx       # BetterAuth context (update from custom auth)
│   │   └── PersonalizationContext.tsx # Personalization state (✅ ALREADY EXISTS)
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupForm.tsx     # BetterAuth signup form (update from custom)
│   │   │   ├── SigninForm.tsx     # BetterAuth signin form (update from custom)
│   │   │   ├── BackgroundQuestionsForm.tsx # Background questions (✅ ALREADY EXISTS)
│   │   │   └── AuthNavbarItem.tsx # Auth navbar item (update from custom)
│   │   ├── PersonalizationButton/ # Personalization button (✅ ALREADY EXISTS)
│   │   └── TranslationButton/     # Translation button (✅ ALREADY EXISTS)
│   └── utils/
│       ├── authClient.ts          # BetterAuth client utilities (update from custom)
│       └── config.ts              # Configuration utilities (✅ ALREADY EXISTS)
├── i18n/
│   └── ur/                       # Urdu translations (✅ ALREADY EXISTS)
├── docusaurus.config.ts           # ✅ ALREADY EXISTS (i18n configured)
└── package.json                   # ✅ ALREADY EXISTS (may need BetterAuth updates)
```

**Structure Decision**: Web application architecture with separate backend (FastAPI) and frontend (Docusaurus/React). Implementation work will focus on:
1. **Backend**: Create BetterAuth-compatible backend adapter and authentication endpoints
2. **Frontend**: Update authentication context and components to use BetterAuth
3. **Migration**: Handle existing users transitioning from custom auth to BetterAuth
4. **Testing**: Add comprehensive tests for BetterAuth flows

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - No constitutional violations. All gates pass cleanly.

## Outcome

- ✅ Impact: Created comprehensive implementation plan for BetterAuth integration completion addressing gaps identified in analysis
- 🧪 Tests: Defined testing approach for BetterAuth flows and migration
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and auth.openapi.yaml
- 🔁 Next prompts: Ready for task generation with clear architecture and implementation approach
- 🧠 Reflection: Plan addresses BetterAuth requirement gap while maintaining backward compatibility

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation phase