# Feature Specification: AI-Driven Book + RAG Chatbot (Hackathon Project)

**Feature Branch**: `feature/book-rag-chatbot`
**Created**: 2025-12-03
**Status**: Draft
**Input**: User description: "Create the full technical specification for the project.

──────────────────────────────────────────
1. Book System Specification
──────────────────────────────────────────
Framework: Docusaurus 3.9

Deployment: GitHub Pages

Required Files:
- docusaurus.config.js
- sidebar.js
- docs/*.md
- static/chatbot/ChatUI.jsx
- src/theme/Root.js

Required Configs:
- url: "https://<username>.github.io"
- baseUrl: "/<repo-name>/"
- organizationName: "<github-username>"
- projectName: "<repo-name>"
- trailingSlash: false

Book Structure:
- introduction.md
- chapters/ (auto-generated)
- summary.md
- glossary.md
- references.md

──────────────────────────────────────────
2. RAG Chatbot Specification
──────────────────────────────────────────
Frontend:
- Custom React-based ChatUI component
- Two modes:
    A. Global QA (answers about full book)
    B. Selected-text QA (user highlights text → sent to backend)

Backend Integration:
- HTTP POST → /query  (global QA)
- HTTP POST → /select (selected-text QA)

Features:
- Streaming or non-streaming responses (configurable)
- Inline citations e.g. "[Chapter 2: Paragraph 4]"
- Loading indicator
- Selecting text auto-opens chatbot with context

──────────────────────────────────────────
3. Backend Specification (FastAPI)
──────────────────────────────────────────
Files:
- main.py
- embed.py
- rag.py
- qdrant_client.py
- schema.py

Endpoints:
- /embed  → store embeddings
- /query  → full-book QA (uses OpenAI Agents SDK)
- /select → selected-text QA (uses OpenAI Agents SDK)
- /health → server status

AI Integration:
- OpenAI Agents SDK (Python) for RAG responses
- Model: gpt-4o-mini or compatible
- Qdrant context retrieval before agent call
- Agent receives context chunks and user query

Chunking rules:
- Chunk size: 500 tokens
- Overlap: 50 tokens

Qdrant Schema:
collection_name: "book_embeddings"
vector_size: 1536
distance: "Cosine"
payload_fields:
- id: string
- chapter: string
- text: string
- token_count: int

──────────────────────────────────────────
4. LLM Integration

──────────────────────────────────────────
**Book Generation**:
- Claude Code Router → Claude (paid, requires API key)
- Fallback: Gemini (free tier, optional testing only)

**RAG Chatbot**:
- OpenAI Agents SDK (Python)
- Model: gpt-4o-mini or compatible
- Integrated with Qdrant for context retrieval

LLM Uses:

Create outline (Claude)

Generate chapters (Claude)

Create glossary + summary (Claude)

Generate embeddings (OpenAI)

Generate RAG answers (OpenAI Agents SDK)

Configuration / Notes:

Set environment variables:

export ANTHROPIC_API_KEY="sk-XXXX-XXXX-XXXX"
export OPENAI_API_KEY="sk-XXXX-XXXX-XXXX"


Subagents use Claude via Router for book generation

OpenAI Agents SDK handles RAG chatbot responses


──────────────────────────────────────────
5. Subagents + Skills
──────────────────────────────────────────
Subagents:
- BookOutlineAgent
- ChapterWriterAgent
- RAGAgent
- APIIntegrationAgent

Skills:
- generate_outline()
- generate_chapter()
- create_markdown()
- embed_book()
- query_rag()
- build_sidebar()
- build_docusaurus()

──────────────────────────────────────────
6. Deployment Specification
──────────────────────────────────────────
Docusaurus:
- Deploy via GitHub Pages → `/docs` branch or `/gh-pages`.

FastAPI:
- Deploy to Render (free tier)
- Autoscaling off
- Python 3.10+
"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Generate and Deploy Book (Priority: P1)

As a user, I want to automatically generate a multi-chapter book using AI and deploy it as a static website so that I can easily share AI-generated content.

**Why this priority**: This is the core functionality of the "AI-Driven Book" aspect and a fundamental deliverable for the hackathon.

**Independent Test**: The deployed Docusaurus site should be accessible, display the generated book structure, and contain content within its chapters, summary, glossary, and references.

**Acceptance Scenarios**:

1.  **Given** the project is initialized with a `BOOK_TITLE` in `.env`, **When** I run the book generation process, **Then** an outline, chapters, summary, glossary, and references are generated as Markdown files within the `docs/` directory.
2.  **Given** the Markdown files are generated, **When** I deploy the Docusaurus project to GitHub Pages, **Then** the book is publicly accessible at the specified GitHub Pages URL.

---

### User Story 2 - Interact with RAG Chatbot (Global QA) (Priority: P1)

As a user, I want to ask questions about the entire book content via a chatbot and receive answers with inline citations so that I can quickly find information and verify sources.

**Why this priority**: This is the core functionality of the "RAG Chatbot" aspect and a fundamental deliverable for the hackathon.

**Independent Test**: The chatbot UI should load, allow text input, and provide relevant, cited answers to general questions about the book.

**Acceptance Scenarios**:

1.  **Given** the Docusaurus book is built and the FastAPI backend is deployed, **When** I type a question into the chatbot UI and submit it, **Then** the chatbot displays a loading indicator, shows an answer (streaming or non-streaming), and includes inline citations (e.g., "[Chapter 2: Paragraph 4]") from the book text.
2.  **Given** a question about the book content, **When** the chatbot provides an answer, **Then** the answer accurately reflects the content of the book.

---

### User Story 3 - Interact with RAG Chatbot (Selected-Text QA) (Priority: P2)

As a user, I want to highlight specific text within the book and ask questions related only to that selected text so that I can get context-specific answers.

**Why this priority**: This enhances the RAG chatbot's usability by providing more granular querying capabilities.

**Independent Test**: Highlighting text should automatically open the chatbot with the selected text as context, and subsequent questions should be answered based *only* on that context.

**Acceptance Scenarios**:

1.  **Given** I am viewing a chapter of the book, **When** I highlight a section of text, **Then** the chatbot UI automatically appears, pre-populated with the highlighted text as context for a query.
2.  **Given** the chatbot is open with selected text context, **When** I ask a question, **Then** the answer provided is limited to the information within the highlighted text and includes appropriate inline citations.

---

### Edge Cases

- What happens when the LLM fails to generate a chapter or outline? The system should handle errors gracefully and potentially retry or flag for manual intervention.
- How does the system handle very large books that exceed token limits during chunking? Chunking rules should ensure content fits within LLM context windows.
- What if the Qdrant database is unavailable or returns an error during a RAG query? The backend should return an appropriate error message to the frontend.
- What if a user highlights text that does not have clear citations or context within the book? The chatbot should indicate that it cannot provide a precise citation or answer.

## Requirements *(mandatory)*


### Modules
## Module 1: ROS 2
description: "Robotic Nervous System basics"

## Module 2: Digital Twin
description: "Gazebo & Unity simulations"

## Module 3: NVIDIA Isaac
description: "AI perception and control"

## Module 4: VLA
description: "Voice-to-Action, Cognitive Planning"


### Functional Requirements

-   **FR-001**: The system MUST generate a book outline.
-   **FR-002**: The system MUST generate chapters based on the outline.
-   **FR-003**: The system MUST generate a summary and glossary for the book.
-   **FR-004**: The system MUST configure Docusaurus for static site generation.
-   **FR-005**: The system MUST generate `docusaurus.config.js`, `sidebar.js`, `docs/*.md`, `static/chatbot/ChatUI.jsx`, and `src/theme/Root.js`.
-   **FR-006**: The system MUST deploy the Docusaurus book to GitHub Pages.
-   **FR-007**: The system MUST provide a FastAPI backend with `/embed`, `/query`, `/select`, and `/health` endpoints.
-   **FR-008**: The backend MUST store embeddings using Qdrant with the specified schema (`book_embeddings`, vector_size: 1536, distance: "Cosine", payload_fields: id, chapter, text, token_count).
-   **FR-009**: The backend MUST implement chunking rules (500 tokens chunk size, 50 tokens overlap).
-   **FR-010**: The RAG chatbot MUST support two modes: Global QA and Selected-text QA.
-   **FR-011**: The chatbot MUST use a custom React-based ChatUI component for the frontend.
-   **FR-012**: The chatbot MUST display a loading indicator during queries.
-   **FR-013**: The chatbot MUST support streaming or non-streaming answers to the frontend.
-   **FR-014**: The chatbot MUST provide inline citations in the format "[Chapter X: Paragraph Y]".
-   **FR-015**: The chatbot MUST automatically open with context when text is selected in the book.
-   **FR-016**: The system MUST use Claude (paid API) for book generation via Claude Code Router.
-   **FR-017**: The system SHOULD allow fallback to Gemini (free tier) if Claude API key is missing for testing purposes.
-   **FR-020**: The RAG chatbot MUST use OpenAI Agents SDK (Python) for generating responses with context from Qdrant.
-   **FR-018**: The system MUST deploy the FastAPI backend to Render.
-   **FR-019**: Subagents and Skills MUST be utilized for book generation, RAG, and API integration.

### Key Entities

-   **Book**: Composed of `introduction.md`, `chapters/*.md`, `summary.md`, `glossary.md`, `references.md`.
-   **BookChunk**: A segment of book text (500 tokens) with associated metadata (id, chapter, text, token_count) for embedding and RAG.
-   **Embedding**: A vector representation of a `BookChunk` stored in Qdrant.
-   **Query**: User input to the RAG chatbot.
-   **Citation**: A reference to a specific part of the book text (e.g., "[Chapter 2: Paragraph 4]").

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The generated Docusaurus book is successfully deployed and publicly accessible via GitHub Pages.
-   **SC-002**: The FastAPI backend is deployed to Render and all specified endpoints are operational and respond within 2 seconds (p95 latency).
-   **SC-003**: The RAG chatbot successfully answers global QA queries about the entire book with relevant, cited information 90% of the time.
-   **SC-004**: The RAG chatbot successfully answers selected-text QA queries about highlighted book content with relevant, cited information 90% of the time, constrained to the selected text.
-   **SC-005**: The book generation process (outline, chapters, summary, glossary) completes successfully without manual intervention.
-   **SC-006**: All project deliverables and deadlines for the hackathon are met.
