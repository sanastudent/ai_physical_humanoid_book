# Data Model: AI-Driven Book + RAG Chatbot

## Overview
Data model derived from the feature specification, specifically the "Key Entities" section in spec.md and functional requirements.

## Core Entities

### Book
**Description:** The main book entity containing all content
**Fields:**
- `id` (string): Unique identifier for the book
- `title` (string): Title of the book from .env configuration
- `chapters` (array): Collection of Chapter entities
- `summary` (string): Book summary content
- `glossary` (object): Glossary terms and definitions
- `references` (array): List of references
- `createdAt` (timestamp): Creation timestamp
- `updatedAt` (timestamp): Last update timestamp

**Validation Rules:**
- Title must be provided from BOOK_TITLE environment variable (FR-001)
- Must contain at least one chapter (FR-002)
- Must include summary, glossary, and references (FR-003)

### Chapter
**Description:** Individual chapter within the book
**Fields:**
- `id` (string): Unique identifier for the chapter
- `title` (string): Chapter title
- `content` (string): Full chapter content in Markdown format
- `subtopics` (array): List of subtopics covered
- `learningOutcomes` (array): Learning outcomes for the chapter
- `exercises` (array): Exercises included in the chapter
- `bookId` (string): Reference to parent Book entity
- `order` (integer): Chapter order in the book sequence

**Validation Rules:**
- Content must be RAG-ready Markdown structure (Book Requirements #28)
- Must include learning outcomes, exercises, and real-world notes (Book Requirements #27)
- Must be self-contained but allow cross-references (Output Format #44)

### BookChunk
**Description:** A segment of book text for embedding and RAG operations
**Fields:**
- `id` (string): Unique identifier for the chunk
- `chapterId` (string): Reference to parent Chapter entity
- `text` (string): Chunked text content (max 500 tokens)
- `tokenCount` (integer): Number of tokens in the chunk
- `metadata` (object): Additional metadata for RAG
- `embeddingId` (string): Reference to associated Embedding entity (when processed)

**Validation Rules:**
- Must be exactly 500 tokens maximum (FR-009)
- Must have 50 token overlap when adjacent (FR-009)
- Must preserve context from original chapter

### Embedding
**Description:** Vector representation of a BookChunk stored in Qdrant
**Fields:**
- `id` (string): Unique identifier for the embedding
- `chunkId` (string): Reference to parent BookChunk entity
- `vector` (array): Vector representation (1536 dimensions)
- `collectionName` (string): Collection name in Qdrant ("book_embeddings")
- `distance` (string): Distance metric ("Cosine")
- `payload` (object): Additional data including chapter, text, token_count

**Validation Rules:**
- Vector size must be 1536 (FR-008)
- Distance metric must be "Cosine" (FR-008)
- Payload must include id, chapter, text, token_count (FR-008)

### Query
**Description:** User input to the RAG chatbot
**Fields:**
- `id` (string): Unique identifier for the query
- `userId` (string): User identifier (optional for anonymous)
- `question` (string): The question text from user
- `context` (string): Optional context (for selected-text QA)
- `mode` (string): "global" or "selected-text" QA mode
- `createdAt` (timestamp): Query creation timestamp
- `bookId` (string): Reference to book being queried

**Validation Rules:**
- Question must not be empty
- Mode must be one of the supported QA modes (FR-010)

### Citation
**Description:** Reference to a specific part of the book text
**Fields:**
- `id` (string): Unique identifier for the citation
- `queryId` (string): Reference to parent Query entity
- `format` (string): Citation format "[Chapter X: Paragraph Y]" (FR-014)
- `chapter` (string): Referenced chapter
- `paragraph` (string): Referenced paragraph or section
- `text` (string): The cited text
- `confidence` (number): Confidence score of the citation

**Validation Rules:**
- Must follow format "[Chapter X: Paragraph Y]" (FR-014)
- Must reference actual content in the book
- Must be provided with all answers (FR-014)

## Relationships

```
Book 1-* Chapter 1-* BookChunk 1-* Embedding
Query 1-* Citation
Chapter -- BookChunk (one chapter can have many chunks)
Query -- Citation (one query can have many citations)
BookChunk -- Embedding (one chunk maps to one embedding)
```

## State Transitions

### Book State
- `draft` → `generating` → `complete` → `deployed`
- Transitions triggered by FR-001-003 completion

### Chapter State
- `outline` → `writing` → `complete`
- Transitions triggered by FR-002 completion

### BookChunk State
- `created` → `embedding` → `embedded` → `indexed`
- Transitions triggered by FR-008, FR-009 implementation

### Query State
- `received` → `processing` → `answering` → `complete`
- Transitions triggered by FR-007 endpoints

## Indexes and Constraints

### Primary Keys
- Each entity has a unique `id` field
- Composite keys where appropriate (e.g., book+chapter for ordering)

### Foreign Key Constraints
- Chapter.bookId → Book.id
- BookChunk.chapterId → Chapter.id
- Citation.queryId → Query.id
- Embedding.chunkId → BookChunk.id

### Unique Constraints
- Book title within system (if multiple books supported)
- Embedding.collectionName for vector database organization

## API Contract Implications

The data model supports the following API requirements:
- FR-007: Backend service with content processing endpoints
- FR-008: Vector database schema with specified structure
- FR-010: Support for both global and selected-text QA modes
- FR-014: Citation format and structure