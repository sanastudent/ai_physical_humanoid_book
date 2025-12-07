# AI-Driven Book + RAG Chatbot (Hackathon Project) Constitution
<!--
Version change: 1.0.0 → 1.0.1
List of modified principles:
- None (only updating status and amendment date)
Added sections:
- None
Removed sections:
- None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (Constitution Check section needs explicit principles)
- .specify/templates/spec-template.md: ⚠ pending (Implicit alignment with principles review needed)
- .specify/templates/tasks-template.md: ⚠ pending (Implicit alignment with principles review needed)
- .specify/templates/commands/*.md: ⚠ pending (No files found to check)
- README.md: ⚠ pending (File not found)
Follow-up TODOs:
- RATIFICATION_DATE: Needs to be set at official adoption.
-->

## Core Principles

### Specification-First Workflow
All code must be generated from Spec-Kit documents.

### AI-Native Development
Claude Code + Subagents + Agent Skills handle book generation and RAG.

### Deterministic Structure
Book layout, backend API, vector schemas remain stable.

### Reusability
Subagents + skills act as reusable intelligence.

### Transparency
Chatbot answers must provide citations from book text.

### Dual QA Mode
Global book QA + selected-text QA.

### Full Deployment
Docusaurus → GitHub Pages, FastAPI → Render.

### Performance
Optimized chunking and Qdrant vector search.

### Reliability
Predictable builds using structured specs.

### Hackathon Compliance
Must match all official deliverables and deadlines.

## Implementation Status

### Completed Components
- ✅ Docusaurus frontend with custom chatbot integration
- ✅ FastAPI backend with RAG functionality
- ✅ Qdrant vector database integration
- ✅ Dual QA modes (global and selected-text)
- ✅ Text selection workflow for contextual queries
- ✅ Citation system with source tracking

### Critical Issues Resolved
- ✅ Backend import naming conflict (qdrant_client.py → qdrant_manager.py)
- ✅ Relative import fixes across all modules
- ✅ Docusaurus build and serve functionality verified

### Current Status
- Frontend: Production-ready build system
- Backend: RAG pipeline fully functional
- Integration: Selected-text QA workflow operational
- Deployment: Ready for GitHub Pages + backend hosting

## Governance
This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.0.1 | **Ratified**: TODO(RATIFICATION_DATE): Needs to be set at official adoption. | **Last Amended**: 2025-12-05
