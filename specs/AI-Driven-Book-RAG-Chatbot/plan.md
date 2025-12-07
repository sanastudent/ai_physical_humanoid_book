# Implementation Plan: AI-Driven Book + RAG Chatbot (Hackathon Project)

**Branch**: `feature/book-rag-chatbot` | **Date**: 2025-12-03 | **Spec**: specs/AI-Driven-Book-RAG-Chatbot/spec.md
**Input**: Feature specification from `/specs/AI-Driven-Book-RAG-Chatbot/spec.md`

## Summary

This project aims to automatically generate a multi-chapter book using AI and deploy it as a static website (Docusaurus on GitHub Pages). Concurrently, it will implement a RAG chatbot using FastAPI as a backend and ChatKit on the frontend, capable of answering questions about the book content with inline citations. The system will support both global QA and selected-text QA modes.

## Technical Context

**Language/Version**: Python 3.10+, JavaScript/React
**Primary Dependencies**: FastAPI, Docusaurus 3.9, Qdrant, @openai/chatkit@latest
**Storage**: Qdrant (vector database)
**Testing**: Local Docusaurus build tests, FastAPI endpoint tests, RAG pipeline integration tests.
**Target Platform**: GitHub Pages (frontend), Render (backend)
**Project Type**: Web application (Docusaurus frontend, FastAPI backend)
**Performance Goals**: FastAPI endpoints respond within 2 seconds (p95 latency); Optimized chunking for Qdrant vector search.
**Constraints**: Render free tier (autoscaling off), GitHub Pages deployment limitations.
**Scale/Scope**: Hackathon project scope, single book generation, RAG for one book.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Specification-First Workflow**: All code will be generated from Spec-Kit documents (this plan, associated tasks).
- **AI-Native Development**: Claude Code, Subagents, and Agent Skills will handle book generation and RAG.
- **Deterministic Structure**: Book layout, backend API, and Qdrant vector schemas will remain stable as defined in the spec.
- **Reusability**: Subagents and skills will be designed for reusability.
- **Transparency**: Chatbot answers will provide inline citations from book text.
- **Dual QA Mode**: Both Global QA and Selected-text QA modes will be implemented as per the spec.
- **Full Deployment**: Docusaurus will deploy to GitHub Pages, and FastAPI to Render.
- **Performance**: Optimized chunking and Qdrant vector search are explicitly mentioned as goals.
- **Reliability**: Predictable builds will be achieved through structured specs.
- **Hackathon Compliance**: The project aims to meet all official deliverables and deadlines.

All constitution principles are adhered to and addressed within the technical context and plan.

## Project Structure

### Documentation (this feature)

```text
specs/AI-Driven-Book-RAG-Chatbot/
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
│   ├── main.py
│   ├── embed.py
│   ├── rag.py
│   ├── qdrant_client.py
│   └── schema.py
└── tests/

frontend/
├── docusaurus.config.js
├── sidebar.js
├── docs/
│   ├── introduction.md
│   ├── chapters/
│   ├── summary.md
│   ├── glossary.md
│   └── references.md
├── static/chatbot/
│   └── ChatUI.jsx
└── src/theme/
    └── Root.js
```

**Structure Decision**: The project will utilize a web application structure with a `backend/` directory for the FastAPI application and a `frontend/` directory for the Docusaurus static site, aligned with the user's specification.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |
