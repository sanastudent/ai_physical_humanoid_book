---
id: 0029
title: BetterAuth Completion Tasks
stage: tasks
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-betterauth-completion
branch: 003-betterauth-completion
user: User
command: /sp.tasks
labels: [feature, tasks, auth, betterauth, security]
links:
  spec: specs/003-betterauth-completion/spec.md
  plan: specs/003-betterauth-completion/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-betterauth-completion/tasks.md
tests:
  - backend/tests/test_auth.py
  - backend/tests/test_session.py
  - backend/tests/test_user_migration.py
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: $ARGUMENTS

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

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

# Implementation Tasks: BetterAuth Integration Completion

**Feature**: 003-betterauth-completion
**Generated**: 2025-12-18
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)
**Status**: Ready for implementation

## Overview

This document outlines implementation tasks for completing the BetterAuth integration. Based on the plan and existing codebase analysis, most infrastructure already exists with primary gaps in the BetterAuth-compatible backend adapter, authentication endpoints, and frontend integration.

## Phase 1: Setup Tasks

- [ ] T001 Install BetterAuth dependencies in backend requirements.txt
- [ ] T002 Install BetterAuth client dependencies in frontend package.json
- [ ] T003 Update environment variables with BetterAuth configuration (BETTERAUTH_SECRET, etc.)

## Phase 2: Foundational Tasks

- [ ] T004 Update users table schema to include BetterAuth fields (email_verified, email_verified_at)
- [ ] T005 Update sessions table schema to include BetterAuth fields (session_type, provider_id)
- [ ] T006 Create database migration scripts for BetterAuth compatibility
- [ ] T007 Verify existing authentication infrastructure is functional

## Phase 3: User Story 1 - BetterAuth-Compatible Authentication Flow (P1)

**Goal**: Enable users to interact with the authentication system expecting standard BetterAuth behavior, with the backend properly supporting BetterAuth's session management, user verification, and authentication flows.

**Independent Test Criteria**: Can be fully tested by using BetterAuth frontend components to signup, signin, and verify sessions, confirming that all authentication flows work as expected with BetterAuth standards.

### Story-Specific Tasks:

- [ ] T008 [US1] Create BetterAuth backend adapter in backend/src/auth/better_auth_adapter.py
- [ ] T009 [US1] [P] Implement user creation method in BetterAuth adapter
- [ ] T010 [US1] [P] Implement user lookup by email method in BetterAuth adapter
- [ ] T011 [US1] [P] Implement session creation method in BetterAuth adapter
- [ ] T012 [US1] [P] Implement session lookup by token method in BetterAuth adapter
- [ ] T013 [US1] Create BetterAuth-compatible auth endpoints in backend/src/routes/auth.py
- [ ] T014 [US1] [P] Implement registration endpoint (/auth/register) with BetterAuth patterns
- [ ] T015 [US1] [P] Implement login endpoint (/auth/login) with BetterAuth patterns
- [ ] T016 [US1] [P] Implement session endpoint (/auth/session) for validation
- [ ] T017 [US1] [P] Implement user endpoint (/auth/user) for user info retrieval
- [ ] T018 [US1] [P] Implement signout endpoint (/auth/session DELETE) for session termination
- [ ] T019 [US1] Register BetterAuth-compatible routes in backend/src/main.py
- [ ] T020 [US1] Update User model for BetterAuth compatibility in backend/src/models/user.py
- [ ] T021 [US1] Update Session model for BetterAuth compatibility in backend/src/models/session.py
- [ ] T022 [US1] Update UserService for BetterAuth patterns in backend/src/services/user_service.py
- [ ] T023 [US1] Update SessionService for BetterAuth patterns in backend/src/services/session_service.py

## Phase 4: User Story 2 - Session Management and Security (P1)

**Goal**: Ensure logged-in users have sessions managed securely according to BetterAuth standards, with proper token handling, expiration, and security measures implemented.

**Independent Test Criteria**: Can be tested by creating a session, verifying token security, testing session expiration, and confirming secure session handling according to BetterAuth security standards.

### Story-Specific Tasks:

- [ ] T024 [US2] Implement secure session token generation with 256-bit random tokens
- [ ] T025 [US2] [P] Configure session cookies with HttpOnly, Secure, SameSite=Lax flags
- [ ] T026 [US2] [P] Implement proper session expiration (24h default, 30d with remember me)
- [ ] T027 [US2] [P] Create session validation middleware for BetterAuth compatibility
- [ ] T028 [US2] [P] Implement session cleanup for expired sessions
- [ ] T029 [US2] [P] Add session termination capability for admin/security events
- [ ] T030 [US2] [P] Implement password verification using bcrypt in validators.py

## Phase 5: User Story 3 - API Endpoint Compatibility (P2)

**Goal**: Ensure frontend applications using BetterAuth can communicate with backend endpoints that follow BetterAuth's API contract standards for authentication operations.

**Independent Test Criteria**: Can be tested by making API calls using BetterAuth's standard endpoints and verifying that the backend responds with the expected data structures and follows BetterAuth's API contract.

### Story-Specific Tasks:

- [ ] T031 [US3] [P] Implement BetterAuth-compatible response format for auth endpoints
- [ ] T032 [US3] [P] Implement BetterAuth-compatible error response patterns
- [ ] T033 [US3] [P] Create proper API documentation for BetterAuth endpoints
- [ ] T034 [US3] [P] Implement request validation for BetterAuth-compatible inputs
- [ ] T035 [US3] [P] Ensure responses follow BetterAuth data structures (user, session objects)

## Phase 6: User Story 4 - Migration from Custom Auth to BetterAuth (P2)

**Goal**: Ensure existing users with accounts created through custom authentication have their accounts properly integrated with BetterAuth's user management system.

**Independent Test Criteria**: Can be tested by verifying existing accounts are properly recognized by BetterAuth systems and that user data is accessible through BetterAuth's user management interfaces.

### Story-Specific Tasks:

- [ ] T036 [US4] [P] Implement migration tracking for existing users (legacy vs BetterAuth sessions)
- [ ] T037 [US4] [P] Create migration logic for existing users during first BetterAuth login
- [ ] T038 [US4] [P] Ensure password hash compatibility (bcrypt) for existing users
- [ ] T039 [US4] [P] Update existing user records with BetterAuth-compatible fields
- [ ] T040 [US4] [P] Create migration scripts for existing user accounts
- [ ] T041 [US4] [P] Implement transparent migration during user login process

## Phase 7: Frontend Integration Tasks

- [ ] T042 [P] Update AuthProvider to use BetterAuth client in frontend/my-book/src/contexts/AuthProvider.tsx
- [ ] T043 [P] Update SignupForm to use BetterAuth methods in frontend/my-book/src/components/Auth/SignupForm.tsx
- [ ] T044 [P] Update SigninForm to use BetterAuth methods in frontend/my-book/src/components/Auth/SigninForm.tsx
- [ ] T045 [P] Update AuthNavbarItem for BetterAuth state in frontend/my-book/src/components/Auth/AuthNavbarItem.tsx
- [ ] T046 [P] Update authClient utilities for BetterAuth in frontend/my-book/src/utils/authClient.ts

## Phase 8: Testing Tasks

- [ ] T047 [P] Create BetterAuth authentication tests in backend/tests/test_auth.py
- [ ] T048 [P] Create session management tests in backend/tests/test_session.py
- [ ] T049 [P] Create user migration tests in backend/tests/test_user_migration.py
- [ ] T050 [P] Update frontend component tests for BetterAuth integration

## Phase 9: Polish & Cross-Cutting Concerns

- [ ] T051 [P] Update BetterAuth configuration in backend/src/config/auth_config.py
- [ ] T052 [P] Add error handling for BetterAuth API rate limits and service outages
- [ ] T053 [P] Implement fallback mechanisms for BetterAuth configuration issues
- [ ] T054 [P] Add monitoring and logging for BetterAuth authentication flows
- [ ] T055 [P] Update deployment configuration for BetterAuth environment variables
- [ ] T056 [P] Conduct end-to-end testing of BetterAuth integration
- [ ] T057 [P] Verify 100% existing user account accessibility after integration (SC-015)
- [ ] T058 [P] Verify authentication performance meets <500ms requirement (SC-013)
- [ ] T059 [P] Verify session validation performance meets <250ms requirement (SC-014)

## Dependencies

- Foundational Tasks (Phase 2) must complete before any user story implementation
- User Story 1 (Authentication Flow) must complete before User Story 4 (Migration) can be fully tested
- Backend BetterAuth adapter (US1-T008) must be completed before auth endpoints (US1-T013)

## Parallel Execution Opportunities

- User Story 1 tasks can run in parallel for different components (adapter, routes, models)
- User Story 2 security tasks can run in parallel with User Story 3 API compatibility tasks
- Frontend integration tasks (Phase 7) can run in parallel with backend tasks
- Testing tasks (Phase 8) can run in parallel with implementation tasks

## Implementation Strategy

**MVP Scope**: Focus on User Story 1 (BetterAuth Authentication Flow) to deliver core functionality. Implement backend adapter, auth endpoints, and basic session management first.

**Incremental Delivery**:
1. Complete Phase 1-2 (Setup and Foundational)
2. Complete User Story 1 (Authentication Flow)
3. Complete User Story 2 (Session Management)
4. Complete User Story 3 (API Compatibility)
5. Complete User Story 4 (Migration)
6. Complete Frontend Integration
7. Complete Testing and Polish

## Success Criteria Verification

Each task should verify compliance with the success criteria from spec.md:
- SC-012: BetterAuth frontend components can successfully authenticate users with 100% compatibility
- SC-013: Authentication requests complete within 1.5 seconds under normal load conditions
- SC-014: Session validation occurs in under 500ms with 99.9% success rate
- SC-015: 100% of existing user accounts remain accessible after BetterAuth integration
- SC-018: 99% of authentication requests result in successful user sessions
- SC-020: Error handling provides clear messages to users without exposing system internals
- SC-021: Migration from custom auth to BetterAuth completes successfully for all existing accounts

## Outcome

- ✅ Impact: Generated comprehensive tasks.md with 59 specific, actionable tasks organized by user stories for completing BetterAuth integration
- 🧪 Tests: Identified test tasks for authentication, session management, and user migration (4 test files)
- 📁 Files: Created specs/003-betterauth-completion/tasks.md with structured task breakdown by user stories
- 🔁 Next prompts: Implementation of individual tasks from the generated list
- 🧠 Reflection: Tasks are well-organized by user stories with proper dependencies and parallel execution opportunities identified

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation of critical path tasks (T001-T007 in foundational phase)