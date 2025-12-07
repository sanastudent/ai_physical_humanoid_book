---
id: 002
title: Create spec for AI-Driven Book + RAG Chatbot
stage: spec
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: AI-Driven-Book-RAG-Chatbot
branch: feature/1-ai-book-rag
user: User
command: /sp.specify # Subagent Instructions (Hackathon Bonus)...
labels: [spec, creation, requirements]
links:
  spec: specs/1-ai-book-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/1-ai-book-rag/spec.md
- specs/1-ai-book-rag/checklists/requirements.md
tests:

---

## Prompt

```text
# Subagent Instructions (Hackathon Bonus)
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

# End specify
```

## Response snapshot

The feature specification for "AI-Driven Book + RAG Chatbot" has been created and validated.
- **Branch Name**: `feature/1-ai-book-rag`
- **Spec File Path**: `specs/1-ai-book-rag/spec.md`
- **Checklist Status**: All items passed in `specs/1-ai-book-rag/checklists/requirements.md`

The spec is now ready for the planning phase.

## Outcome

- ✅ Impact: Successfully created and validated the feature specification, including a new branch and a requirements checklist.
- 🧪 Tests: N/A (no tests run in this phase)
- 📁 Files: Created `specs/1-ai-book-rag/spec.md` and `specs/1-ai-book-rag/checklists/requirements.md`.
- 🔁 Next prompts: Proceed to planning (`/sp.plan`).
- 🧠 Reflection: Successfully created the branch and spec file, then iteratively refined the spec to remove implementation details and pass the quality checklist. Adapted to the lack of shell script for PHR creation by generating it natively.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec failed quality checks due to implementation details. Successfully rectified in a single iteration.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve initial spec generation to be more technology-agnostic to reduce refinement iterations.
