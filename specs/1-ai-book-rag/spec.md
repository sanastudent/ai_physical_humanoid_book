# Feature Specification: AI-Driven Book + RAG Chatbot

**Feature Branch**: `feature/1-ai-book-rag`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "# Subagent Instructions (Hackathon Bonus)
Create these Subagents using Claude Router (paid API):
1. BookOutlineAgent: Generate book outline from course syllabus; output chapters array with titles, subtopics, learning outcomes.
2. ChapterWriterAgent: Take chapter outline → generate full Markdown with code blocks, exercises, image placeholders, citation placeholders.
3. RAGAgent: Chunk text, generate embeddings, store in Qdrant schema; support global QA and selected-text QA.
4. APIIntegrationAgent: Connect /embed, /query, /select to RAGAgent; return JSON answers with citations.
Save outputs in `docs/` for Docusaurus; agents callable independently.

# Goal
Generate full book content (outline, chapters, glossary, summary, references) for the course: ${BOOK_TITLE} (from .env), designed for learners with basic AI knowledge.

# Audience
Students/professionals learning Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac, VLA integration. Step-by-step guidance with practical exercises and examples.

# Book Requirements
- Introduction: Foundations of Physical AI, embodied intelligence, future AI in robotics.
- Modules 1–4 + Capstone Project.
- Weekly breakdown 12–13 weeks, exercises, mini-projects.
- Glossary, summary, references.
- Code examples in Python (ROS 2), Isaac, Gazebo, Unity.
- Include simulation tips, edge devices (Jetson), robot deployment notes.
- Chapters end with: Learning outcomes, exercises, real-world notes.
- RAG-ready Markdown structure, chunkable for embeddings.

# Modules / Chapters
Module 1: Robotic Nervous System (ROS 2) — ROS 2 architecture, nodes/topics/services, URDF for humanoids, Python (rclpy) examples; exercises: create nodes, publish/subscribe.

Module 2: Digital Twin (Gazebo & Unity) — Physics simulation, gravity, collisions, sensors (LiDAR, Depth, IMU), Unity visualization; exercises: simulate humanoid environment, visualize movement.

Module 3: AI-Robot Brain (NVIDIA Isaac) — Isaac Sim & ROS, hardware-accelerated perception (VSLAM), navigation (Nav2), synthetic data; exercises: train perception, simulate navigation.

Module 4: Vision-Language-Action (VLA) — Voice commands (Whisper), LLM → ROS actions, autonomous humanoid workflow; exercises: implement simple voice-to-action tasks.

Capstone Project — Autonomous Humanoid: Voice command → plan → navigate → manipulate object; integrate ROS 2, Isaac, VLA, sensors; final assessment: full pipeline simulation.

# Output Format
- Markdown files `docs/*.md` for Docusaurus.
- Include headings, subheadings, code blocks, image placeholders, citations placeholders.
- Chapters self-contained; cross-reference chapters.
- Include `glossary.md`, `summary.md`, `references.md`.

# Instructions for Claude Code
1. Run BookOutlineAgent → generate outline.
2. Run ChapterWriterAgent → generate chapters in Markdown.
3. Generate glossary, summary, references.
4. Ensure content is Hackathon compliant: RAG-ready, chunkable, structured, includes exercises/code/citations.

# End specify"

## Clarifications
### Session 2025-12-07
- Q: Which AI model should be used for processing? → A: Use Claude Opus (most powerful) for all processing
- Q: What embedding model should be used for 1536-dim vectors? → A: OpenAI Ada 2
- Q: For which operations should 2-second p95 latency be maintained? → A: RAG query operations (both global and selected-text QA)
- Q: How should Qdrant be hosted? → A: Self-hosted Qdrant in cloud
- Q: How should the system handle failures in book generation? → A: Implement retry logic with exponential backoff and manual review queue

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Generate and Deploy Book (Priority: P1)

As a user, I want to automatically generate a multi-chapter book using AI and deploy it as a static website so that I can easily share AI-generated content.

**Why this priority**: This is the core functionality of the "AI-Driven Book" aspect and a fundamental deliverable for the hackathon.

**Independent Test**: The deployed website should be accessible, display the generated book structure, and contain content within its chapters, summary, glossary, and references.

**Acceptance Scenarios**:

1. **Given** the project is initialized with a `BOOK_TITLE` in `.env`, **When** I run the book generation process, **Then** an outline, chapters, summary, glossary, and references are generated as Markdown files within the designated documentation directory.
2. **Given** the Markdown files are generated, **When** I deploy the project to a web hosting service, **Then** the book is publicly accessible at the specified URL.

---

### User Story 2 - Interact with RAG Chatbot (Global QA) (Priority: P1)

As a user, I want to ask questions about the entire book content via a chatbot and receive answers with inline citations so that I can quickly find information and verify sources.

**Why this priority**: This is the core functionality of the "RAG Chatbot" aspect and a fundamental deliverable for the hackathon.

**Independent Test**: The chatbot UI should load, allow text input, and provide relevant, cited answers to general questions about the book.

**Acceptance Scenarios**:

1. **Given** the Docusaurus book is built and the FastAPI backend is deployed, **When** I type a question into the chatbot UI and submit it, **Then** the chatbot displays a loading indicator, streams an answer, and includes inline citations (e.g., "[Chapter 2: Paragraph 4]") from the book text.
2. **Given** a question about the book content, **When** the chatbot provides an answer, **Then** the answer accurately reflects the content of the book.

---

### User Story 3 - Interact with RAG Chatbot (Selected-Text QA) (Priority: P2)

As a user, I want to highlight specific text within the book and ask questions related only to that selected text so that I can get context-specific answers.

**Why this priority**: This enhances the RAG chatbot's usability by providing more granular querying capabilities.

**Independent Test**: Highlighting text should automatically open the chatbot with the selected text as context, and subsequent questions should be answered based *only* on that context.

**Acceptance Scenarios**:

1. **Given** I am viewing a chapter of the book, **When** I highlight a section of text, **Then** the chatbot UI automatically appears, pre-populated with the highlighted text as context for a query.
2. **Given** the chatbot is open with selected text context, **When** I ask a question, **Then** the answer provided is limited to the information within the highlighted text and includes appropriate inline citations.

---

### Edge Cases

- What happens when the LLM fails to generate a chapter or outline? The system MUST implement retry logic with exponential backoff and maintain a manual review queue for persistent failures.
- How does the system handle very large books that exceed token limits during chunking? Chunking rules should ensure content fits within LLM context windows.
- What if the Qdrant database is unavailable or returns an error during a RAG query? The backend should return an appropriate error message to the frontend.
- What if a user highlights text that does not have clear citations or context within the book? The chatbot should indicate that it cannot provide a precise citation or answer.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST generate a book outline.
- **FR-002**: The system MUST generate chapters based on the outline.
- **FR-003**: The system MUST generate a summary and glossary for the book.
- **FR-004**: The system MUST configure a static site generator for publishing the book.
- **FR-005**: The system MUST generate configuration files for the static site, sidebar navigation, and book content (e.g., introduction, chapters, summary, glossary, references) within the designated documentation directory, as well as components for the chatbot UI and theme customization.
- **FR-006**: The system MUST deploy the generated book to a static web hosting service.
- **FR-007**: The system MUST provide a backend service with endpoints for content processing, global QA, selected-text QA, and a health check.
- **FR-008**: The backend MUST store processed content in a self-hosted Qdrant vector database in the cloud with a defined schema (e.g., collection_name: "book_embeddings", vector_size: 1536, distance: "Cosine", payload_fields: id, chapter, text, token_count).
- **FR-009**: The backend MUST implement content processing rules (e.g., chunk size: 500 tokens, overlap: 50 tokens).
- **FR-010**: The RAG chatbot MUST support two modes: Global QA and Selected-text QA.
- **FR-011**: The chatbot MUST integrate a frontend SDK for its user interface.
- **FR-012**: The chatbot MUST display a loading indicator during queries.
- **FR-013**: The chatbot MUST stream answers to the frontend.
- **FR-014**: The chatbot MUST provide inline citations in the format "[Chapter X: Paragraph Y]".
- **FR-015**: The chatbot MUST automatically open with context when text is selected in the book.
- **FR-016**: The system MUST use Claude Opus for text generation and RAG integration via a routing mechanism.
- **FR-017**: The system SHOULD allow fallback to a free-tier AI model if the primary AI model access is unavailable for testing purposes.
- **FR-018**: The system MUST use OpenAI Ada 2 embeddings with 1536-dimensional vectors for RAG operations.
- **FR-019**: The system MUST deploy the backend service to a cloud hosting platform.
- **FR-020**: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration.

### Key Entities *(include if feature involves data)*

- **Book**: Composed of `introduction.md`, `chapters/*.md`, `summary.md`, `glossary.md`, `references.md`.
- **BookChunk**: A segment of book text (500 tokens) with associated metadata (id, chapter, text, token_count) for embedding and RAG.
- **Embedding**: A vector representation of a `BookChunk` stored in Qdrant.
- **Query**: User input to the RAG chatbot.
- **Citation**: A reference to a specific part of the book text (e.g., "[Chapter 2: Paragraph 4]").

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The generated book is successfully deployed and publicly accessible via a web hosting service.
- **SC-002**: The backend service is deployed to a cloud hosting platform and RAG query endpoints (global QA and selected-text QA) respond within 2 seconds (p95 latency).
- **SC-003**: The RAG chatbot successfully answers global QA queries about the entire book with relevant, cited information 90% of the time.
- **SC-004**: The RAG chatbot successfully answers selected-text QA queries about highlighted book content with relevant, cited information 90% of the time, constrained to the selected text.
- **SC-005**: The book generation process (outline, chapters, summary, glossary) completes successfully without manual intervention.
- **SC-006**: All project deliverables and deadlines for the hackathon are met.
