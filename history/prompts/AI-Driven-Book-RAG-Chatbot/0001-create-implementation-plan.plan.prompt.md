---
id: 0001
title: Create implementation plan
stage: plan
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: AI-Driven-Book-RAG-Chatbot
branch: master
user: User
command: "/sp.plan Phase 1: Setup - Install Claude Code Router\n- Add Gemini free API key\n- Install Spec-Kit Plus\n- Create GitHub repo\n- Initialize project root\n\nPhase 2: Pre-Title Book Logic\n- Build BookOutlineAgent + ChapterWriterAgent\n- Test sample outlines\n\nPhase 3: Title Integration\n- Add book title via .env: BOOK_TITLE=\"Physical AI & Humanoid Robotics: A Textbook\"\n- Regenerate outline using BOOK_TITLE\n- Regenerate all chapters\n- Generate summary + glossary\n- (Note: Title can be updated anytime in .env before running this phase)\n\nPhase 4: Build Docusaurus Book\n- Auto-generate docs folder\n- Generate sidebar.js\n- Add Docusaurus config\n- Local build test\n- Deploy to GitHub Pages\n\nPhase 5: Backend (FastAPI + Qdrant)\n- Create /embed\n- Create /query\n- Create /select\n- Connect Qdrant Cloud\n- Test full RAG pipeline\n\nPhase 6: Chatbot Integration\n- Add ChatKit with @openai/chatkit\n- Add ChatUI.jsx\n- Add Root.js override\n- Implement selected-text flow\n- Add streaming + citation format\n\nPhase 7: Bonus Intelligence\n- Add reusable skills\n- Add Subagents\n\nPhase 8: Final Deployment\n- Docusaurus → GitHub Pages\n- Backend → Render\n- RAG Final Tests\n- Record YouTube demo\n- Submit project"
labels: ["planning", "implementation-plan"]
links:
  spec: specs/AI-Driven-Book-RAG-Chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/AI-Driven-Book-RAG-Chatbot/plan.md
tests: null
---

## Prompt

/sp. plan Phase 1: Setup
- Install Claude Code Router
- Add Gemini free API key
- Install Spec-Kit Plus
- Create GitHub repo
- Initialize project root

Phase 2: Pre-Title Book Logic
- Build BookOutlineAgent + ChapterWriterAgent
- Test sample outlines

Phase 3: Title Integration
- Add book title via .env: BOOK_TITLE="Physical AI & Humanoid Robotics: A Textbook"
- Regenerate outline using BOOK_TITLE
- Regenerate all chapters
- Generate summary + glossary
- (Note: Title can be updated anytime in .env before running this phase)

Phase 4: Build Docusaurus Book
- Auto-generate docs folder
- Generate sidebar.js
- Add Docusaurus config
- Local build test
- Deploy to GitHub Pages

Phase 5: Backend (FastAPI + Qdrant)
- Create /embed
- Create /query
- Create /select
- Connect Qdrant Cloud
- Test full RAG pipeline

Phase 6: Chatbot Integration
- Add ChatKit with @openai/chatkit
- Add ChatUI.jsx
- Add Root.js override
- Implement selected-text flow
- Add streaming + citation format

Phase 7: Bonus Intelligence
- Add reusable skills
- Add Subagents

Phase 8: Final Deployment
- Docusaurus → GitHub Pages
- Backend → Render
- RAG Final Tests
- Record YouTube demo
- Submit project

## Response snapshot

The implementation plan for "AI-Driven Book + RAG Chatbot (Hackathon Project)" has been successfully created and saved to `specs/AI-Driven-Book-RAG-Chatbot/plan.md`.

**Summary:** The plan outlines the technical context, project structure, and ensures alignment with the project's core principles as defined in the constitution. It details the use of Docusaurus for the frontend and FastAPI with Qdrant for the backend, along with LLM integration via Claude Code Router.

**Next Steps:**
- Generate tasks based on this plan using `/sp.tasks`.

**Suggested Commit Message:**
`feat: initial implementation plan for AI-Driven Book + RAG Chatbot
`

## Outcome

- ✅ Impact: Implementation plan drafted and saved.
- 🧪 Tests: No tests run in this phase.
- 📁 Files: 1 file created: specs/AI-Driven-Book-RAG-Chatbot/plan.md
- 🔁 Next prompts: Generate tasks based on this plan using `/sp.tasks`.
- 🧠 Reflection: Successfully drafted the implementation plan based on user specification and project constitution.

## Evaluation notes (flywheel)

- Failure modes observed: PHR creation script failed, resorted to agent-native tools.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why `create-phr.sh` failed.
