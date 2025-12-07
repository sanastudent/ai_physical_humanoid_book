---
id: 004
title: Generate tasks for AI-Driven Book + RAG Chatbot
stage: tasks
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: AI-Driven-Book-RAG-Chatbot
branch: feature/1-ai-book-rag
user: User
command: /sp.tasks # Task Generation Phase: AI-Driven Book + RAG Chatbot...
labels: [tasks, generation, planning]
links:
  spec: specs/1-ai-book-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/1-ai-book-rag/tasks.md
tests:

---

## Prompt

```text
# Task Generation Phase: AI-Driven Book + RAG Chatbot

Feature: AI-Driven Book + RAG Chatbot
Spec File: specs/1-ai-book-rag/spec.md
Plan File: specs/1-ai-book-rag/plan.md
Branch: feature/1-ai-book-rag

## Goal
Generate a detailed, executable task list from the validated plan:
- Break each high-level task in the plan into subtasks where necessary.
- Assign dependencies, priority, and estimated effort (optional).
- Tasks must be clear, actionable, and measurable.

## Task Categories (derived from plan)
1. **BookOutlineAgent Tasks**
   - Generate chapter outlines from course syllabus.
   - Define learning outcomes and subtopics for each chapter.

2. **ChapterWriterAgent Tasks**
   - Convert outlines into Markdown chapters.
   - Include code blocks, exercises, image placeholders, citation placeholders.

3. **RAGAgent Tasks**
   - Chunk text content.
   - Generate embeddings.
   - Store in vector database schema.
   - Implement QA modes: Global QA and Selected-text QA.

4. **APIIntegrationAgent Tasks**
   - Create backend endpoints: /embed, /query, /select.
   - Connect RAGAgent outputs to API.
   - Ensure responses include citations and structured JSON.

5. **Deployment & Testing Tasks**
   - Deploy book content to static hosting.
   - Test QA chatbot functionality.
   - Validate all success criteria from spec.

## Output Requirements
- Format: JSON or Markdown.
- Each task should include:
  - Task ID
  - Title
  - Description
  - Dependencies (if any)
  - Subtasks (optional)
  - Priority (high/medium/low)
  - Optional: effort estimate (hours)
- Tasks should be grouped by agent or functional category for clarity.

## Instructions for Claude
- Use the plan (`plan.md`) as source of truth.
- Focus on executable, high-level tasks; do not include implementation specifics.
- Ensure tasks can be assigned and tracked individually.
- Output a **complete task list read
```

## Response snapshot

I have successfully generated the `tasks.md` file.

**Tasks Summary for AI-Driven Book + RAG Chatbot:**
- **File**: `specs/1-ai-book-rag/tasks.md`
- **Total Tasks**: 40
- **User Story Tasks**:
    - User Story 1 (P1 - Generate and Deploy Book): 11 tasks
    - User Story 2 (P1 - Interact with RAG Chatbot Global QA): 13 tasks
    - User Story 3 (P2 - Interact with RAG Chatbot Selected-Text QA): 3 tasks
- **Other Phases**:
    - Setup: 4 tasks
    - Foundational: 4 tasks
    - Polish & Cross-Cutting: 5 tasks
- **Parallel Opportunities**: Identified with `[P]` flag for concurrent execution within phases and across user stories after foundational setup.
- **Independent Test Criteria**: Clearly defined for each user story in the `tasks.md` file.
- **Suggested MVP Scope**: Focus on User Story 1 for initial delivery.
- **Format Validation**: All tasks adhere to the specified checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`.

## Outcome

- ✅ Impact: Successfully generated tasks.md based on spec and plan.
- 🧪 Tests: N/A (no tests run in this phase)
- 📁 Files: Created specs/1-ai-book-rag/tasks.md
- 🔁 Next prompts: Implement tasks from tasks.md
- 🧠 Reflection: Successfully adapted to missing bash script by using agent-native PHR creation. Confirmed project structure and generated comprehensive tasks based on provided spec and plan.

## Evaluation notes (flywheel)

- Failure modes observed: Shell script execution failed (pwsh and create-phr.sh not found).
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve environment detection for shell capabilities.
