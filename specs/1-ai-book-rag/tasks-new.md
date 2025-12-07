---
description: "Task list for AI-Driven Book + RAG Chatbot feature implementation - Remaining Work"
---

# Tasks: AI-Driven-Book-RAG-Chatbot

**Input**: Design documents from `/specs/1-ai-book-rag/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Focus**: Generate remaining tasks to reach 100% spec coverage
**Context**:
- Book generation mostly done
- Backend partially done
- Frontend partially done
- Subagents partially registered
- Docusaurus partially built

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

- [ ] T098 Update project structure with missing directories if needed
- [ ] T099 Configure additional dependencies for remaining functionality
- [ ] T100 Set up configuration for AI model routing (primary/fallback) in `backend/src/config/ai_config.py`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T101 Complete vector database schema implementation in `backend/src/schema.py` (FR-008)
- [ ] T102 Complete Qdrant client with proper error handling in `backend/src/qdrant_client.py`
- [ ] T103 Set up reusable skills framework in `backend/src/skills/` (FR-019)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Generate and Deploy Book (Priority: P1) 🎯 MVP

**Goal**: Automatically generate a multi-chapter book using AI and deploy it as a static website.

**Independent Test**: The deployed website should be accessible, display the generated book structure, and contain content within its chapters, summary, glossary, and references.

### Implementation for User Story 1

- [ ] T104 [US1] Complete BookOutlineAgent with all learning outcomes in `backend/src/agents/book_outline_agent.py` (FR-001)
- [ ] T105 [US1] Complete ChapterWriterAgent with exercises, image placeholders, citation placeholders in `backend/src/agents/chapter_writer_agent.py` (FR-002)
- [ ] T106 [US1] Complete glossary and references generation in `backend/src/agents/book_outline_agent.py` (FR-003)
- [ ] T107 [US1] Implement error handling for LLM failures with retry logic in `backend/src/agents/book_outline_agent.py` (Edge Case #104)
- [ ] T108 [US1] Implement token limit handling for large books in `backend/src/agents/chapter_writer_agent.py` (Edge Case #105)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interact with RAG Chatbot (Global QA) (Priority: P1)

**Goal**: Ask questions about the entire book content via a chatbot and receive answers with inline citations.

**Independent Test**: The chatbot UI should load, allow text input, and provide relevant, cited answers to general questions about the book.

### Implementation for User Story 2

- [ ] T109 [P] [US2] Complete content embedding with proper chunking (500 tokens) and overlap (50 tokens) in `backend/src/embed.py` (FR-009)
- [ ] T110 [US2] Complete global QA endpoint with proper answer accuracy in `backend/src/main.py` (FR-007, FR-010)
- [ ] T111 [US2] Complete chatbot UI with proper SDK integration in `frontend/src/components/ChatUI.jsx` (FR-011)
- [ ] T112 [US2] Complete loading indicator implementation in `frontend/src/components/ChatUI.jsx` (FR-012)
- [ ] T113 [US2] Complete answer streaming functionality in `frontend/src/components/ChatUI.jsx` (FR-013)
- [ ] T114 [US2] Complete inline citations display in format "[Chapter X: Paragraph Y]" in `frontend/src/components/ChatUI.jsx` (FR-014)
- [ ] T115 [US2] Complete RAGAgent with global QA capabilities in `backend/src/agents/rag_agent.py` (FR-010)
- [ ] T116 [US2] Implement error handling for Qdrant database unavailability in `backend/src/rag.py` (Edge Case #106)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interact with RAG Chatbot (Selected-Text QA) (Priority: P2)

**Goal**: Highlight specific text within the book and ask questions related only to that selected text.

**Independent Test**: Highlighting text should automatically open the chatbot with the selected text as context, and subsequent questions should be answered based *only* on that context.

### Implementation for User Story 3

- [ ] T117 [US3] Complete selected-text QA endpoint with context limitation in `backend/src/main.py` (FR-007, FR-010, FR-015)
- [ ] T118 [US3] Complete text selection auto-open functionality for chatbot in `frontend/src/theme/Root.js` (FR-015)
- [ ] T119 [US3] Complete selected-text context handling in `frontend/src/components/ChatUI.jsx` (FR-015)
- [ ] T120 [US3] Complete RAGAgent with selected-text QA capabilities in `backend/src/agents/rag_agent.py` (FR-010, FR-015)
- [ ] T121 [US3] Implement citation validation when no clear citations exist in `backend/src/rag.py` (Edge Case #107)

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T122 Complete APIIntegrationAgent with /embed, /query, /select endpoints in `backend/src/agents/api_integration_agent.py` (FR-019)
- [ ] T123 Complete AI model routing with primary/fallback configuration in `backend/src/config/ai_config.py` (FR-016, FR-017)
- [ ] T124 Complete subagent registration and initialization in `backend/src/main.py` (FR-019)
- [ ] T125 Create reusable content processing skill in `backend/src/skills/content_processing.py` (FR-019)
- [ ] T126 Create reusable RAG skill in `backend/src/skills/rag.py` (FR-019)
- [ ] T127 Complete Docusaurus build process optimization in `frontend/docusaurus.config.js` (FR-006)
- [ ] T128 Complete GitHub Pages deployment configuration in `frontend/.github/workflows/deploy.yml` (FR-006)
- [ ] T129 Complete Render backend deployment configuration in `backend/requirements.txt` and `backend/Dockerfile` (FR-018)
- [ ] T130 Add comprehensive backend tests for all endpoints in `backend/tests/` (SC-002, SC-003, SC-004)
- [ ] T131 Add performance monitoring for 2-second response time requirement in `backend/src/middleware/performance.py` (SC-002)
- [ ] T132 Complete end-to-end integration tests in `tests/e2e/` (SC-003, SC-004)
- [ ] T133 Validate book generation process completes without manual intervention (SC-005)
- [ ] T134 Test global QA answers with 90% relevant, cited information rate (SC-003)
- [ ] T135 Test selected-text QA answers with 90% relevant, cited information rate (SC-004)
- [ ] T136 Verify backend endpoints respond within 2 seconds (p95 latency) (SC-002)
- [ ] T137 Verify deployed book is publicly accessible via web hosting (SC-001)

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
# Launch all remaining components for User Story 1 together:
Task: "Complete BookOutlineAgent with all learning outcomes in backend/src/agents/book_outline_agent.py (FR-001)"
Task: "Complete ChapterWriterAgent with exercises, image placeholders, citation placeholders in backend/src/agents/chapter_writer_agent.py (FR-002)"
Task: "Complete glossary and references generation in backend/src/agents/book_outline_agent.py (FR-003)"
Task: "Implement error handling for LLM failures with retry logic in backend/src/agents/book_outline_agent.py (Edge Case #104)"
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