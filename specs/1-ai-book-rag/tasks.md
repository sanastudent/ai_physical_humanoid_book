---
description: "Task list for AI-Driven Book + RAG Chatbot feature implementation"
---

# Tasks: AI-Driven-Book-RAG-Chatbot

**Input**: Design documents from `/specs/1-ai-book-rag/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend/` and `frontend/` root directories
- [X] T002 Initialize Python environment in `backend/` with `requirements.txt`
- [X] T003 Initialize Node.js environment in `frontend/` with `package.json`
- [X] T004 Create basic `.env` file in project root for configuration
- [X] T005 [P] Create directory structure for backend agents (`backend/src/agents/`)
- [X] T006 [P] Set up configuration management for AI model routing (primary/fallback)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Configure static site generator (e.g., Docusaurus) in `frontend/`
- [X] T008 Create static site navigation configuration (e.g., `frontend/sidebars.ts`)
- [X] T009 Create base backend service (e.g., FastAPI) in `backend/src/main.py`
- [X] T010 [P] Add dependencies for backend and frontend (e.g., FastAPI and Docusaurus) to `requirements.txt` and `package.json` respectively
- [X] T011 Create vector database schema file (e.g., `backend/src/schema.py`)
- [X] T012 Create vector database client file (e.g., `backend/src/qdrant_manager.py`)
- [X] T013 Create data model files based on `data-model.md` in `backend/src/schema.py`
- [X] T014 Set up API contract validation middleware in `backend/src/middleware/performance.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Generate and Deploy Book (Priority: P1) 🎯 MVP

**Goal**: Automatically generate a multi-chapter book using AI and deploy it as a static website.

**Independent Test**: The deployed website should be accessible, display the generated book structure, and contain content within its chapters, summary, glossary, and references.

### Implementation for User Story 1

- [X] T015 [US1] Create introduction content file in documentation directory (e.g., `frontend/docs/introduction.md`)
- [X] T016 [US1] Create summary content file in documentation directory (e.g., `frontend/docs/summary.md`)
- [X] T017 [US1] Create glossary content file in documentation directory (e.g., `frontend/docs/glossary.md`)
- [X] T018 [US1] Create references content file in documentation directory (e.g., `frontend/docs/references.md`)
- [X] T019 [US1] Create chapters directory in documentation (e.g., `frontend/docs/chapters/`)
- [X] T020 [US1] Implement BookOutlineAgent capability to generate outline from course syllabus in `backend/src/agents/book_outline_agent.py`
- [X] T021 [US1] Implement ChapterWriterAgent capability to generate Markdown chapters with placeholders in `backend/src/agents/chapter_writer_agent.py`
- [X] T022 [US1] Implement capability to write generated Markdown content to documentation directory
- [X] T023 [US1] Implement capability to update static site navigation configuration (e.g., `frontend/sidebars.ts`)
- [X] T024 [US1] Configure static web hosting deployment (e.g., GitHub Pages) in `frontend/`
- [X] T025 [US1] Implement capability to build static site (e.g., Docusaurus build process)
- [X] T026 [US1] Implement error handling for LLM failures in book generation (Edge Case #104)
- [X] T027 [US1] Implement token limit handling for large books in `backend/src/agents/chapter_writer_agent.py` (Edge Case #105)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interact with RAG Chatbot (Global QA) (Priority: P1)

**Goal**: Ask questions about the entire book content via a chatbot and receive answers with inline citations.

**Independent Test**: The chatbot UI should load, allow text input, and provide relevant, cited answers to general questions about the book.

### Implementation for User Story 2

- [X] T028 [P] [US2] Implement content embedding logic (chunking, vectorization) in `backend/src/embed.py`
- [X] T029 [P] [US2] Implement RAG logic in `backend/src/rag.py`
- [X] T030 [US2] Implement content processing endpoint (e.g., `/embed`) in `backend/src/main.py`
- [X] T031 [US2] Implement global QA endpoint (e.g., `/query`) in `backend/src/main.py`
- [X] T032 [US2] Create chatbot UI component (e.g., `frontend/my-book/static/chatbot/ChatUI.jsx`)
- [X] T033 [US2] Integrate frontend SDK for chatbot UI in `frontend/my-book/static/chatbot/ChatUI.jsx`
- [X] T034 [US2] Implement loading indicator in chatbot UI (e.g., `frontend/my-book/static/chatbot/ChatUI.jsx`)
- [X] T035 [US2] Enable streaming answers from backend to chatbot UI (e.g., `frontend/my-book/static/chatbot/ChatUI.jsx`)
- [X] T036 [US2] Display inline citations in chatbot UI (e.g., `frontend/my-book/static/chatbot/ChatUI.jsx`)
- [X] T037 [US2] Implement RAGAgent capability to process and embed book content in `backend/src/agents/rag_agent.py`
- [X] T038 [US2] Implement RAGAgent capability to handle global QA queries in `backend/src/agents/rag_agent.py`
- [X] T039 [US2] Implement error handling for Qdrant database unavailability in `backend/src/rag.py` (Edge Case #106)
- [X] T040 [US2] Implement fallback behavior when no citations are available in `backend/src/rag.py`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interact with RAG Chatbot (Selected-Text QA) (Priority: P2)

**Goal**: Highlight specific text within the book and ask questions related only to that selected text.

**Independent Test**: Highlighting text should automatically open the chatbot with the selected text as context, and subsequent questions should be answered based *only* on that context.

### Implementation for User Story 3

- [X] T041 [US3] Implement selected-text QA endpoint (e.g., `/select`) in `backend/src/main.py`
- [X] T042 [US3] Create static site theme customization file (e.g., `frontend/my-book/src/theme/Root.js`)
- [X] T043 [US3] Implement text selection auto-opening chatbot with context in `frontend/my-book/src/theme/Root.js` and `frontend/my-book/static/chatbot/ChatUI.jsx`
- [X] T044 [US3] Implement selected-text QA logic in `backend/src/rag.py`
- [X] T045 [US3] Add citation validation for selected-text context in `backend/src/rag.py` (Edge Case #107)

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T046 Configure AI model integration with primary/fallback routing in `backend/src/config/ai_config.py`
- [X] T047 Implement backend service deployment configuration for cloud hosting platform (Render)
- [X] T048 Add health check endpoint (e.g., `/health`) in `backend/src/main.py`
- [X] T049 Review and refine content processing rules (e.g., chunking) in `backend/src/embed.py`
- [X] T050 Implement error handling for AI model failures, vector database unavailability in `backend/src/rag.py` and `backend/src/main.py`
- [X] T051 Implement performance monitoring for 2-second response time requirement (SC-002)
- [X] T052 Add comprehensive logging for debugging and monitoring
- [X] T053 Implement graceful degradation when vector database is slow (Edge Case #106 follow-up)
- [X] T054 Create deployment scripts for GitHub Pages and Render
- [X] T055 Add comprehensive tests for all API endpoints and error scenarios
- [X] T056 Document the API using the OpenAPI contract from `contracts/api-contract.yaml`
- [X] T057 Set up CI/CD pipeline for automated testing and deployment

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
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Integrates with US1 by using the generated book content.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 and US2 for text selection and QA.

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all model/component creation for User Story 1 together:
Task: "Create introduction content file in documentation directory (e.g., frontend/docs/introduction.md)"
Task: "Create summary content file in documentation directory (e.g., frontend/docs/summary.md)"
Task: "Create glossary content file in documentation directory (e.g., frontend/docs/glossary.md)"
Task: "Create references content file in documentation directory (e.g., frontend/docs/references.md)"
Task: "Create chapters directory in documentation (e.g., frontend/docs/chapters/)"
Task: "Implement BookOutlineAgent capability to generate outline from course syllabus in backend/src/agents/book_outline_agent.py"
Task: "Implement ChapterWriterAgent capability to generate Markdown chapters with placeholders in backend/src/agents/chapter_writer_agent.py"
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
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence