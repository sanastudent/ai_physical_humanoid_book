# Tasks: BetterAuth Signup & Signin with User Background Collection

**Input**: Design documents from `/specs/002-betterauth-signup/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-spec.yaml

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are excluded from this implementation plan.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/my-book/src/`
- Backend: FastAPI (Python)
- Frontend: Docusaurus/React (TypeScript)
- Database: Neon DB (PostgreSQL)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for authentication system

- [X] T001 Configure Neon DB connection string in backend/.env (NEON_DB_URL)
- [X] T002 [P] Install BetterAuth Python library in backend/requirements.txt or pyproject.toml
- [X] T003 [P] Install BetterAuth React library in frontend/my-book/package.json
- [X] T004 [P] Install email-validator library in backend for email validation
- [X] T005 [P] Install React Hook Form and Zod in frontend/my-book/package.json for form validation
- [X] T006 Create .env.example files documenting required environment variables (BETTERAUTH_SECRET, NEON_DB_URL, REACT_APP_BACKEND_URL)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Schema & Migrations

- [X] T007 Create database migration script for users table in backend/migrations/001_create_users.sql
- [X] T008 [P] Create database migration script for sessions table in backend/migrations/002_create_sessions.sql
- [X] T009 [P] Create database migration script for software_background table in backend/migrations/003_create_software_background.sql
- [X] T010 [P] Create database migration script for hardware_background table in backend/migrations/004_create_hardware_background.sql
- [X] T011 Create database migration script for indexes in backend/migrations/005_create_indexes.sql
- [X] T012 Create database migration script for triggers (updated_at) in backend/migrations/006_create_triggers.sql
- [X] T013 Run all database migrations on Neon DB instance

### Backend Data Models

- [X] T014 [P] Create User Pydantic model in backend/src/models/user.py
- [X] T015 [P] Create Session Pydantic model in backend/src/models/session.py
- [X] T016 [P] Create SoftwareBackground Pydantic model in backend/src/models/software_background.py
- [X] T017 [P] Create HardwareBackground Pydantic model in backend/src/models/hardware_background.py
- [X] T018 Create BackgroundInput and BackgroundData request/response models in backend/src/models/background.py

### Backend Core Services

- [X] T019 Configure BetterAuth with Neon DB connection in backend/src/config/auth_config.py
- [X] T020 Create database connection pool manager in backend/src/database/connection.py
- [X] T021 Create session validation middleware in backend/src/middleware/auth_middleware.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Account Creation with Background Information (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with email/password and provide software/hardware background information during signup

**Independent Test**: Complete the signup form with valid credentials and background questions, verify account is created in database, confirm user is auto-logged in, verify background data is stored correctly

### Backend Implementation for User Story 1

- [X] T022 [P] [US1] Create POST /auth/signup endpoint handler in backend/src/routes/auth.py
- [X] T023 [P] [US1] Create POST /background endpoint handler in backend/src/routes/background.py
- [X] T024 [US1] Implement UserService.create_user() method in backend/src/services/user_service.py (depends on T022)
- [X] T025 [US1] Implement BackgroundService.create_background() method in backend/src/services/background_service.py (depends on T023)
- [X] T026 [US1] Add email validation (RFC 5322 format) in backend/src/utils/validators.py
- [X] T027 [US1] Add password strength validation (min 8 chars, 1 upper, 1 lower, 1 number) in backend/src/utils/validators.py
- [X] T028 [US1] Add background data validation (experience level, language/platform lists) in backend/src/utils/validators.py
- [X] T029 [US1] Implement duplicate email check in UserService in backend/src/services/user_service.py
- [X] T030 [US1] Add error handling for 409 Conflict (email exists) in POST /auth/signup endpoint
- [X] T031 [US1] Add transaction handling for signup (user + background creation must be atomic) in backend/src/services/user_service.py

### Frontend Implementation for User Story 1

- [X] T032 [P] [US1] Create SignupForm component (Step 1 - credentials) in frontend/my-book/src/components/Auth/SignupForm.tsx
- [X] T033 [P] [US1] Create BackgroundQuestionsForm component (Step 2) in frontend/my-book/src/components/Auth/BackgroundQuestionsForm.tsx
- [X] T034 [US1] Add multi-step form state management in SignupForm component (Step 1 → Step 2 flow)
- [X] T035 [US1] Add client-side email validation in SignupForm using Zod schema
- [X] T036 [US1] Add client-side password strength validation in SignupForm using Zod schema
- [X] T037 [US1] Add software background question fields in BackgroundQuestionsForm (experience level dropdown, languages multi-select, frameworks multi-select)
- [X] T038 [US1] Add hardware background question fields in BackgroundQuestionsForm (experience level dropdown, platforms multi-select, devices multi-select)
- [X] T039 [US1] Integrate SignupForm with POST /auth/signup endpoint
- [X] T040 [US1] Integrate BackgroundQuestionsForm with POST /background endpoint
- [X] T041 [US1] Add inline validation error display in SignupForm for email/password
- [X] T042 [US1] Add validation for required background fields (at least one language, one platform)
- [X] T043 [US1] Add error handling for duplicate email (display "Email already registered" message)
- [X] T044 [US1] Add network error handling with toast notifications in signup flow
- [X] T045 [US1] Create Signup page route in frontend/my-book/src/pages/signup.tsx
- [X] T046 [US1] Add auto-login after successful signup (redirect to background questions)
- [X] T047 [US1] Add auto-redirect to main content after completing background questions

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Users can create accounts, provide background information, and be automatically logged in.

---

## Phase 4: User Story 2 - Secure Sign In (Priority: P1)

**Goal**: Enable existing users to sign in with email/password and maintain authenticated sessions

**Independent Test**: Create an account via User Story 1, log out, then sign back in with correct credentials, verify session persists across page navigation, verify "remember me" extends session duration

### Backend Implementation for User Story 2

- [X] T048 [P] [US2] Create POST /auth/signin endpoint handler in backend/src/routes/auth.py
- [X] T049 [P] [US2] Create POST /auth/signout endpoint handler in backend/src/routes/auth.py
- [X] T050 [P] [US2] Create GET /auth/session endpoint handler in backend/src/routes/auth.py
- [X] T051 [US2] Implement UserService.authenticate_user() method in backend/src/services/user_service.py (password verification with bcrypt)
- [X] T052 [US2] Implement SessionService.create_session() method in backend/src/services/session_service.py (with remember me support - 24h vs 30 days)
- [X] T053 [US2] Implement SessionService.invalidate_session() method in backend/src/services/session_service.py
- [X] T054 [US2] Implement SessionService.get_current_session() method in backend/src/services/session_service.py
- [X] T055 [US2] Add session cookie configuration (HttpOnly, Secure, SameSite=Lax) in POST /auth/signin endpoint
- [X] T056 [US2] Add error handling for 401 Unauthorized (invalid credentials) in POST /auth/signin endpoint
- [X] T057 [US2] Add session expiration check in session validation middleware

### Frontend Implementation for User Story 2

- [X] T058 [P] [US2] Create SigninForm component in frontend/my-book/src/components/Auth/SigninForm.tsx
- [X] T059 [US2] Add email/password fields with validation in SigninForm
- [X] T060 [US2] Add "remember me" checkbox in SigninForm
- [X] T061 [US2] Integrate SigninForm with POST /auth/signin endpoint
- [X] T062 [US2] Add error handling for invalid credentials (display "Invalid email or password")
- [X] T063 [US2] Add network error handling with toast notifications in signin flow
- [X] T064 [US2] Create Signin page route in frontend/my-book/src/pages/signin.tsx
- [X] T065 [US2] Add redirect to main content after successful signin
- [X] T066 [US2] Implement signout functionality (call POST /auth/signout and clear local state)
- [X] T067 [US2] Add signout button to navigation bar or user menu in frontend/my-book/src/components/Auth/

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Users can create accounts, sign in, sign out, and have persistent sessions.

---

## Phase 5: User Story 3 - Access Personalized Content (Priority: P2)

**Goal**: Enable logged-in users with background data to access personalized chapter content via the existing personalization button

**Independent Test**: Log in with an account that has background data, navigate to any chapter, click the personalization button, verify content is adapted based on stored background; test that non-logged-in users see "Sign in to personalize"

### Backend Implementation for User Story 3

- [X] T068 [US3] Create GET /background endpoint handler in backend/src/routes/background.py
- [X] T069 [US3] Implement BackgroundService.get_user_background() method in backend/src/services/background_service.py
- [X] T070 [US3] Add ownership validation (verify session user_id matches background user_id) in GET /background endpoint
- [X] T071 [US3] Add 404 error handling for missing background data in GET /background endpoint
- [X] T072 [US3] Modify existing POST /personalize endpoint to accept user background from session in backend/src/routes/personalization.py

### Frontend Implementation for User Story 3

- [X] T073 [US3] Extend PersonalizationContext to include BetterAuth useSession() hook in frontend/my-book/src/contexts/PersonalizationContext.tsx
- [X] T074 [US3] Add userBackground state to PersonalizationContext in frontend/my-book/src/contexts/PersonalizationContext.tsx
- [X] T075 [US3] Add fetchUserBackground() function that calls GET /background in PersonalizationContext
- [X] T076 [US3] Add useEffect to fetch background data when user session is detected in PersonalizationContext
- [X] T077 [US3] Cache fetched background data in PersonalizationContext state
- [X] T078 [US3] Update PersonalizationButton to check authentication state (show "Sign in to personalize" for anonymous users) in frontend/my-book/src/components/PersonalizationButton/index.tsx
- [X] T079 [US3] Update PersonalizationButton to use background data from PersonalizationContext when personalizing in frontend/my-book/src/components/PersonalizationButton/index.tsx
- [X] T080 [US3] Add prompt to complete background questions if logged-in user has no background data
- [X] T081 [US3] Test personalization works correctly with user's stored background data

**Checkpoint**: All P1 and P2 user stories should now be independently functional. Logged-in users can access personalized content based on their background.

---

## Phase 6: User Story 4 - Update Background Information (Priority: P3)

**Goal**: Enable logged-in users to update their software and hardware background information after initial signup

**Independent Test**: Log in with an existing account, navigate to preferences/profile page, modify background question responses, verify changes are saved, verify future personalized content reflects updated profile

### Backend Implementation for User Story 4

- [ ] T082 [US4] Create PUT /background endpoint handler in backend/src/routes/background.py
- [ ] T083 [US4] Implement BackgroundService.update_background() method in backend/src/services/background_service.py
- [ ] T084 [US4] Add ownership validation (verify session user_id matches background user_id) in PUT /background endpoint
- [ ] T085 [US4] Add 404 error handling for missing background data (prompt user to create via POST) in PUT /background endpoint
- [ ] T086 [US4] Add transaction handling for updating both software and hardware background atomically

### Frontend Implementation for User Story 4

- [ ] T087 [P] [US4] Create PreferencesPage component in frontend/my-book/src/pages/preferences.tsx
- [ ] T088 [P] [US4] Create UpdateBackgroundForm component in frontend/my-book/src/components/Preferences/UpdateBackgroundForm.tsx
- [ ] T089 [US4] Pre-populate UpdateBackgroundForm with current background data from PersonalizationContext
- [ ] T090 [US4] Add software background edit fields (experience level, languages, frameworks) in UpdateBackgroundForm
- [ ] T091 [US4] Add hardware background edit fields (experience level, platforms, devices) in UpdateBackgroundForm
- [ ] T092 [US4] Integrate UpdateBackgroundForm with PUT /background endpoint
- [ ] T093 [US4] Add success notification after background update ("Background updated successfully")
- [ ] T094 [US4] Update PersonalizationContext cache with new background data after successful update
- [ ] T095 [US4] Add validation for required fields (at least one language, one platform)
- [ ] T096 [US4] Add link to preferences page from user menu/navigation bar

**Checkpoint**: All user stories (P1, P2, P3) should now be independently functional. Users can update their profiles and see updated personalization.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final production readiness

### Security Hardening

- [ ] T097 [P] Add rate limiting configuration (optional, future enhancement) in backend/src/middleware/rate_limiter.py
- [ ] T098 [P] Add SQL injection prevention verification (ensure all queries use parameterized statements)
- [ ] T099 [P] Add XSS prevention verification (ensure no dangerouslySetInnerHTML for user input)
- [ ] T100 [P] Verify password hashing uses bcrypt with salt rounds=10 (BetterAuth default)
- [ ] T101 [P] Verify session tokens never logged or exposed in API responses

### Performance Optimization

- [ ] T102 [P] Add database connection pooling configuration (max 20 connections) in backend/src/database/connection.py
- [ ] T103 [P] Verify database indexes are created (email, token, user_id, arrays)
- [ ] T104 [P] Add React.memo optimization for PersonalizationButton to prevent unnecessary re-renders
- [ ] T105 [P] Add useMemo for derived state in PersonalizationContext

### Observability

- [ ] T106 [P] Add structured logging for authentication events (signup, signin, signout) in backend/src/utils/logger.py
- [ ] T107 [P] Add logging for validation errors and database errors
- [ ] T108 [P] Add logging for session errors (expired, invalid tokens)
- [ ] T109 [P] Configure log format as structured JSON with timestamp, level, event, user_id

### Documentation & Environment Setup

- [ ] T110 [P] Create API documentation from contracts/api-spec.yaml (generate with Swagger/Redoc)
- [ ] T111 [P] Update README.md with setup instructions for authentication system
- [ ] T112 [P] Document environment variables in .env.example (BETTERAUTH_SECRET, NEON_DB_URL, REACT_APP_BACKEND_URL)
- [ ] T113 [P] Create deployment runbook in docs/runbooks/authentication-deployment.md

### Final Validation

- [ ] T114 Verify User Story 1 acceptance scenarios (signup with background, duplicate email handling)
- [ ] T115 Verify User Story 2 acceptance scenarios (signin, signout, session persistence, remember me)
- [ ] T116 Verify User Story 3 acceptance scenarios (personalization for logged-in users, anonymous user handling)
- [ ] T117 Verify User Story 4 acceptance scenarios (background update, personalization reflects changes)
- [ ] T118 Run database validation queries (check for orphaned records, verify CASCADE delete)
- [ ] T119 Verify all Success Criteria from spec.md (SC-001 through SC-008)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion
- **User Story 2 (Phase 4)**: Depends on Foundational phase completion - Can run in parallel with US1
- **User Story 3 (Phase 5)**: Depends on Foundational + US1 + US2 completion (needs signup and signin to work first)
- **User Story 4 (Phase 6)**: Depends on Foundational + US1 completion (needs background creation to work first)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies on other stories - Can start after Foundational
- **User Story 2 (P1)**: No dependencies on other stories - Can start after Foundational (parallel with US1)
- **User Story 3 (P2)**: Depends on US1 and US2 (needs accounts and signin to work)
- **User Story 4 (P3)**: Depends on US1 (needs background creation endpoint)

### Within Each User Story

- Backend models/services before endpoints
- Backend endpoints before frontend integration
- Frontend components before page routes
- Core implementation before error handling
- Story complete before moving to next priority

### Parallel Opportunities

**Setup Phase**:
- T002, T003, T004, T005 (all package installations) can run in parallel

**Foundational Phase**:
- T008, T009, T010 (database migrations for separate tables) can run in parallel
- T014, T015, T016, T017 (data models for separate entities) can run in parallel

**User Story 1**:
- T022, T023 (separate endpoint handlers) can run in parallel
- T026, T027, T028 (separate validation functions) can run in parallel
- T032, T033 (separate React components) can run in parallel

**User Story 2**:
- T048, T049, T050 (separate endpoint handlers) can run in parallel
- T058 (frontend component can start while backend is being worked on by different developer)

**User Story 3**:
- T073, T074, T075, T076, T077 (PersonalizationContext changes) are sequential but can be done by one developer while another works on backend

**User Story 4**:
- T087, T088 (separate frontend components) can run in parallel

**Polish Phase**:
- T097, T098, T099, T100, T101 (all security tasks) can run in parallel
- T102, T103, T104, T105 (all performance tasks) can run in parallel
- T106, T107, T108, T109 (all logging tasks) can run in parallel
- T110, T111, T112, T113 (all documentation tasks) can run in parallel

---

## Parallel Example: User Story 1

```bash
# Backend developers can work in parallel on:
Task T022: "Create POST /auth/signup endpoint handler"
Task T023: "Create POST /background endpoint handler"

# Validation functions can be built in parallel:
Task T026: "Add email validation"
Task T027: "Add password strength validation"
Task T028: "Add background data validation"

# Frontend developers can work in parallel on:
Task T032: "Create SignupForm component (Step 1)"
Task T033: "Create BackgroundQuestionsForm component (Step 2)"
```

---

## Parallel Example: Foundational Phase

```bash
# Database migrations for different tables:
Task T008: "Create sessions table migration"
Task T009: "Create software_background table migration"
Task T010: "Create hardware_background table migration"

# Data models for different entities:
Task T014: "Create User model"
Task T015: "Create Session model"
Task T016: "Create SoftwareBackground model"
Task T017: "Create HardwareBackground model"
```

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 2 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T021) - CRITICAL blocker
3. Complete Phase 3: User Story 1 (T022-T047) - Account creation with background
4. Complete Phase 4: User Story 2 (T048-T067) - Signin and session management
5. **STOP and VALIDATE**: Test both stories independently
   - Can users sign up and provide background? ✓
   - Can users sign in and stay logged in? ✓
   - Do sessions persist across navigation? ✓
6. Deploy/demo if ready - **This is a functional MVP!**

### Incremental Delivery (Recommended)

1. **Foundation**: Complete Setup + Foundational (T001-T021)
2. **MVP Release**: Add User Story 1 + 2 (T022-T067) → Test → Deploy
   - Value: Users can create accounts and sign in
3. **Enhancement Release**: Add User Story 3 (T068-T081) → Test → Deploy
   - Value: Logged-in users get personalized content
4. **Polish Release**: Add User Story 4 (T082-T096) → Test → Deploy
   - Value: Users can update their profiles
5. **Production Release**: Complete Polish phase (T097-T119) → Final validation → Production deploy

### Parallel Team Strategy

With multiple developers after Foundational phase completes:

**Team A (Backend)**:
- User Story 1 backend (T022-T031)
- User Story 2 backend (T048-T057)

**Team B (Frontend)**:
- User Story 1 frontend (T032-T047)
- User Story 2 frontend (T058-T067)

**Team C (Feature Enhancement)** - After MVP:
- User Story 3 (T068-T081)
- User Story 4 (T082-T096)

---

## Notes

- **[P]** tasks = different files, no dependencies, can run in parallel
- **[Story]** label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- **Critical path**: Setup → Foundational → US1+US2 (MVP) → US3 → US4 → Polish
- **Tests**: Not included per specification (no explicit test requirements)
- **Validation**: Use acceptance scenarios from spec.md to manually test each story
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Summary

- **Total tasks**: 119 tasks
- **Setup phase**: 6 tasks (T001-T006)
- **Foundational phase**: 15 tasks (T007-T021) - BLOCKS all stories
- **User Story 1**: 26 tasks (T022-T047) - Priority P1 🎯 MVP
- **User Story 2**: 20 tasks (T048-T067) - Priority P1 🎯 MVP
- **User Story 3**: 14 tasks (T068-T081) - Priority P2
- **User Story 4**: 15 tasks (T082-T096) - Priority P3
- **Polish phase**: 23 tasks (T097-T119)
- **Parallel opportunities**: 45 tasks marked [P]
- **MVP scope**: Phases 1-4 (User Stories 1 + 2) = 67 tasks for full authentication system
