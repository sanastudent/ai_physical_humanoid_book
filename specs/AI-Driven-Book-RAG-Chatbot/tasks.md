---

description: "Task list for AI-Driven Book + RAG Chatbot feature implementation (Updated for OpenAI Agents SDK)"
---

# Tasks: AI-Driven-Book-RAG-Chatbot

**Input**: Design documents from `/specs/AI-Driven-Book-RAG-Chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), SPEC_UPDATE_2025-12-04.md (OpenAI Agents SDK changes)
**Version**: 1.1.0 (Updated 2025-12-04 for OpenAI Agents SDK)

**Tests**: Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Paths shown below follow the project structure from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend/` and `frontend/` root directories
- [X] T002 Initialize Python environment in `backend/` with `requirements.txt`
- [X] T003 Initialize Node.js environment in `frontend/` with `package.json`
- [X] T004 Create `.env` file in project root with ANTHROPIC_API_KEY and OPENAI_API_KEY configuration

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Configure Docusaurus `frontend/docusaurus.config.js` with base URL, organization, project name, GitHub Pages settings
- [X] T006 Create Docusaurus `frontend/sidebars.js` with initial book structure
- [X] T007 Create base FastAPI application in `backend/src/main.py` with CORS configuration
- [X] T008 [P] Add dependencies to `backend/requirements.txt`: fastapi, uvicorn, qdrant-client, anthropic, openai, python-dotenv, tiktoken
- [X] T009 [P] Add dependencies to `frontend/package.json`: @docusaurus/core, @docusaurus/preset-classic, react, react-dom
- [X] T010 Create Qdrant schema definition in `backend/src/schema.py` with BookChunk, EmbedRequest, QueryRequest, QueryResponse models

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Generate and Deploy Book (Priority: P1) 🎯 MVP

**Goal**: Automatically generate a multi-chapter book using AI and deploy it as a static website.

**Independent Test**: The deployed Docusaurus site should be accessible, display the generated book structure, and contain content within its chapters, summary, glossary, and references.

### Implementation for User Story 1

- [X] T011 [P] [US1] Create `frontend/docs/introduction.md` with book overview and module descriptions
- [X] T012 [P] [US1] Create `frontend/docs/summary.md` with key takeaways and conclusion
- [X] T013 [P] [US1] Create `frontend/docs/glossary.md` with technical terms and definitions
- [X] T014 [P] [US1] Create `frontend/docs/references.md` with citations and resources
- [X] T015 [US1] Create `frontend/docs/chapters/` directory for chapter content
- [X] T016 [US1] Generate Module 1 chapters in `frontend/docs/chapters/module1-*.md` (4 chapters on ROS 2)
- [X] T017 [US1] Generate Module 2 chapters in `frontend/docs/chapters/module2-*.md` (4 chapters on Digital Twins)
- [X] T018 [US1] Generate Module 3 chapters in `frontend/docs/chapters/module3-*.md` (4 chapters on NVIDIA Isaac)
- [X] T019 [US1] Generate Module 4 chapters in `frontend/docs/chapters/module4-*.md` (4 chapters on VLA Models)
- [X] T020 [US1] Update `frontend/sidebars.js` with complete book navigation structure
- [X] T021 [US1] Configure GitHub Pages deployment settings in `frontend/docusaurus.config.js` (deploymentBranch, trailingSlash)
- [ ] T022 [US1] Test local Docusaurus build with `npm run build` in frontend directory
- [X] T023 [US1] Create deployment script `scripts/deploy_docs.sh` for GitHub Pages deployment

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interact with RAG Chatbot (Global QA) (Priority: P1)

**Goal**: Ask questions about the entire book content via a chatbot and receive answers with inline citations.

**Independent Test**: The chatbot UI should load, allow text input, and provide relevant, cited answers to general questions about the book.

### Implementation for User Story 2

#### Backend RAG Components

- [X] T024 [P] [US2] Create Qdrant client in `backend/src/qdrant_client.py` with collection management and search methods
- [X] T025 [P] [US2] Implement text chunking in `backend/src/embed.py` (500 tokens, 50 overlap) using tiktoken
- [X] T026 [P] [US2] Implement embedding generation in `backend/src/embed.py` using OpenAI embeddings API
- [X] T027 [US2] Implement RAG engine in `backend/src/rag.py` with OpenAI Agents SDK integration
- [X] T028 [US2] Implement context retrieval method in `backend/src/rag.py` using Qdrant similarity search
- [X] T029 [US2] Implement answer generation in `backend/src/rag.py` using OpenAI Agents SDK with gpt-4o-mini model
- [X] T030 [US2] Implement citation extraction in `backend/src/rag.py` to parse inline citations from responses
- [X] T031 [US2] Add `/embed` endpoint in `backend/src/main.py` for single chapter embedding
- [X] T032 [US2] Add `/embed-book` endpoint in `backend/src/main.py` for full book embedding
- [X] T033 [US2] Add `/query` endpoint in `backend/src/main.py` for Global QA using OpenAI Agents SDK
- [X] T034 [US2] Add `/health` endpoint in `backend/src/main.py` for service health checks
- [X] T035 [US2] Configure OpenAI Agents SDK client initialization in `backend/src/rag.py` with API key from environment

#### Frontend Chatbot UI

- [X] T036 [US2] Create custom React ChatUI component in `frontend/static/chatbot/ChatUI.jsx`
- [X] T037 [US2] Implement message display and user input in `frontend/static/chatbot/ChatUI.jsx`
- [X] T038 [US2] Implement loading indicator in `frontend/static/chatbot/ChatUI.jsx`
- [X] T039 [US2] Implement citation rendering in `frontend/static/chatbot/ChatUI.jsx` with formatted display
- [X] T040 [US2] Add fetch-based backend communication in `frontend/static/chatbot/ChatUI.jsx` for `/query` endpoint
- [X] T041 [US2] Add error handling in `frontend/static/chatbot/ChatUI.jsx` for failed requests
- [X] T042 [US2] Create CSS styling in `frontend/static/chatbot/chatui.css` with dark mode support
- [X] T043 [US2] Add chatbot toggle button in `frontend/static/chatbot/ChatUI.jsx` for show/hide functionality

#### Integration and Testing

- [X] T044 [US2] Create embedding script in `scripts/embed_book.py` to process all book markdown files
- [X] T045 [US2] Create RAG testing script in `scripts/test_rag.py` for endpoint validation
- [ ] T046 [US2] Test complete RAG pipeline: embed book → query chatbot → verify citations
  - **Status**: Ready for execution (requires OPENAI_API_KEY in .env)
  - **Test Script**: `scripts/test_rag_pipeline_T046.py` (uses in-memory Qdrant, no Docker needed)
  - **Guide**: See `TEST_EXECUTION_GUIDE_T046.md` for detailed instructions
  - **Blocker**: User must add valid OpenAI API key to `.env` file to execute test

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interact with RAG Chatbot (Selected-Text QA) (Priority: P2)

**Goal**: Highlight specific text within the book and ask questions related only to that selected text.

**Independent Test**: Highlighting text should automatically open the chatbot with the selected text as context, and subsequent questions should be answered based *only* on that context.

### Implementation for User Story 3

- [X] T047 [US3] Add `/select` endpoint in `backend/src/main.py` for Selected-text QA using OpenAI Agents SDK
- [X] T048 [US3] Implement selected-text query handler in `backend/src/rag.py` with context-aware prompting
- [X] T049 [US3] Create Docusaurus theme override in `frontend/src/theme/Root.js` for text selection detection
- [X] T050 [US3] Implement text selection event handlers in `frontend/src/theme/Root.js` (mouseup, touchend)
- [X] T051 [US3] Add auto-open chatbot functionality in `frontend/src/theme/Root.js` when text is selected
- [X] T052 [US3] Update `frontend/static/chatbot/ChatUI.jsx` to accept selectedText prop and display context
- [X] T053 [US3] Add selected-text mode indicator in `frontend/static/chatbot/ChatUI.jsx` UI
- [X] T054 [US3] Implement context-based query submission in `frontend/static/chatbot/ChatUI.jsx` to `/select` endpoint
- [ ] T055 [US3] Test selected-text QA workflow: highlight text → chatbot opens → ask question → receive contextual answer

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T056 Add comprehensive error handling in `backend/src/main.py` for LLM failures and Qdrant unavailability
- [X] T057 Implement request validation in `backend/src/main.py` using Pydantic models
- [ ] T058 Add logging configuration in `backend/src/main.py` with structured logging
- [ ] T059 Optimize chunking strategy in `backend/src/embed.py` for better retrieval quality
- [ ] T060 Add rate limiting middleware in `backend/src/main.py` for API protection
- [X] T061 Create setup automation script in `scripts/setup.sh` for project initialization
- [X] T062 Create README.md in project root with quick start guide and architecture overview
- [X] T063 Create IMPLEMENTATION_GUIDE.md with detailed setup, deployment, and troubleshooting instructions
- [ ] T064 Configure Render deployment in `render.yaml` with environment variables and build commands
- [X] T065 Add responsive mobile styling in `frontend/static/chatbot/chatui.css`
- [ ] T066 Implement WebSocket streaming support (optional) in `backend/src/main.py` for real-time responses
- [ ] T067 Add analytics tracking (optional) in `frontend/static/chatbot/ChatUI.jsx` for usage metrics

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Integrates with US1 by using the generated book content
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Extends US2 with text selection capability

### Within Each User Story

- Backend components before endpoints
- Frontend UI before integration
- Core implementation before testing
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Within US1: All documentation files (T011-T014) can be created in parallel
- Within US2: Backend components (T024-T026) can be built in parallel; Frontend components can be built in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 2

```bash
# Launch all backend RAG components in parallel:
Task: "Create Qdrant client in backend/src/qdrant_client.py"
Task: "Implement text chunking in backend/src/embed.py"
Task: "Implement RAG engine in backend/src/rag.py"

# Launch all frontend UI components in parallel (after backend is ready):
Task: "Create ChatUI component in frontend/static/chatbot/ChatUI.jsx"
Task: "Create CSS styling in frontend/static/chatbot/chatui.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Book generation and deployment)
   - Developer B: User Story 2 (RAG chatbot backend + frontend)
   - Developer C: User Story 3 (Text selection integration)
3. Stories complete and integrate independently

---

## Key Changes from Version 1.0.0

**Updated for OpenAI Agents SDK** (Version 1.1.0):

1. **Removed ChatKit dependencies**:
   - No @openai/chatkit package installation
   - No ChatKit SDK integration tasks

2. **Added OpenAI Agents SDK tasks**:
   - T027: RAG engine with OpenAI Agents SDK integration
   - T029: Answer generation using OpenAI Agents SDK with gpt-4o-mini
   - T033: `/query` endpoint implementation with OpenAI Agents SDK
   - T035: OpenAI Agents SDK client initialization
   - T047: `/select` endpoint with OpenAI Agents SDK

3. **Updated frontend tasks**:
   - T036-T043: Custom React ChatUI component (no ChatKit)
   - T040: Fetch-based backend communication instead of ChatKit streaming

4. **Updated configuration**:
   - T004: Updated to include both ANTHROPIC_API_KEY and OPENAI_API_KEY
   - T008: Updated dependencies to include openai package for Agents SDK

5. **Architecture changes**:
   - Backend now uses OpenAI Agents SDK for RAG responses
   - Frontend uses simple fetch API for backend communication
   - No external chat SDK dependencies
   - Direct control over UI and UX

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- OpenAI Agents SDK provides better control and integration for RAG use cases
- Custom ChatUI offers full flexibility for UI customization
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
