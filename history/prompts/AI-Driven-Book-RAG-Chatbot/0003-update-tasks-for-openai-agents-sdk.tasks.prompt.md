---
id: 0003
title: Update tasks for OpenAI Agents SDK
stage: tasks
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: AI-Driven-Book-RAG-Chatbot
branch: feature/1-ai-book-rag
user: User
command: /sp.tasks
labels: ["tasks", "openai-agents-sdk", "specification-update", "rag-chatbot"]
links:
  spec: specs/AI-Driven-Book-RAG-Chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/AI-Driven-Book-RAG-Chatbot/tasks.md
  - specs/AI-Driven-Book-RAG-Chatbot/SPEC_UPDATE_2025-12-04.md
tests:
  - None (task generation, no code tests)
---

## Prompt

User requested task generation via /sp.tasks command after updating the specification to replace ChatKit SDK with OpenAI Agents SDK.

Context: The specification was previously updated (SPEC_UPDATE_2025-12-04.md) to:
1. Remove all ChatKit SDK dependencies
2. Replace with OpenAI Agents SDK (Python) for backend RAG responses
3. Use custom React-based ChatUI with simple fetch-based communication
4. Update configuration to include both ANTHROPIC_API_KEY and OPENAI_API_KEY

The tasks.md needed regeneration to reflect these architectural changes while maintaining the same user story structure and organization.

## Response snapshot

Generated updated tasks.md (Version 1.1.0) with 67 tasks organized across 5 phases:

**Phase 1: Setup** (4 tasks)
- T001-T004: Basic project initialization with updated .env configuration

**Phase 2: Foundational** (6 tasks)
- T005-T010: Core infrastructure including updated dependencies

**Phase 3: User Story 1 - Book Generation** (13 tasks)
- T011-T023: Book content creation and Docusaurus deployment (unchanged from v1.0)

**Phase 4: User Story 2 - RAG Chatbot Global QA** (23 tasks)
- T024-T046: Backend RAG with OpenAI Agents SDK + Custom ChatUI frontend
  - Key changes:
    - T027: RAG engine with OpenAI Agents SDK integration
    - T029: Answer generation using gpt-4o-mini model
    - T033: /query endpoint with OpenAI Agents SDK
    - T035: OpenAI SDK client initialization
    - T036-T043: Custom React ChatUI (no ChatKit)

**Phase 5: User Story 3 - Selected-Text QA** (9 tasks)
- T047-T055: Text selection integration with OpenAI Agents SDK backend

**Phase N: Polish** (12 tasks)
- T056-T067: Cross-cutting concerns and optional enhancements

**Key improvements:**
- Removed all ChatKit dependencies
- Added OpenAI Agents SDK integration tasks
- Updated configuration tasks for dual API keys
- Maintained user story independence
- Preserved parallel execution opportunities
- Added detailed "Key Changes from Version 1.0.0" section

## Outcome

- ✅ Impact: Successfully updated tasks to reflect OpenAI Agents SDK architecture, removing 0 deprecated tasks and updating 15+ tasks with new implementation details
- 🧪 Tests: No test code generated (task planning only)
- 📁 Files: Updated 1 file (tasks.md v1.1.0), referenced 1 supporting file (SPEC_UPDATE_2025-12-04.md)
- 🔁 Next prompts: Implementation of updated tasks, particularly Phase 4 (User Story 2) backend RAG components
- 🧠 Reflection: Task reorganization maintains clarity while adapting to architectural change; OpenAI Agents SDK integration is now explicit in task descriptions

## Evaluation notes (flywheel)

- Failure modes observed: None - successful task generation
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checkbox format), Path validation PASS (all file paths specified), Story mapping PASS (all US tasks labeled)
- Prompt variant (if applicable): Standard /sp.tasks with specification update context
- Next experiment (smallest change to try): Monitor implementation to validate task granularity is appropriate for OpenAI Agents SDK integration
