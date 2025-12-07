# Quickstart Guide: AI-Driven Book + RAG Chatbot

## Overview
This guide provides quick setup and usage instructions for the AI-Driven Book + RAG Chatbot system, following specification-first principles.

## Prerequisites
- Python 3.10+
- Node.js 16+
- Access to OpenAI API (or fallback model for testing)
- Qdrant vector database access
- Git

## Setup

### 1. Clone and Initialize
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Environment
```bash
# Create .env file with required variables
cp .env.example .env
# Edit .env to include:
# BOOK_TITLE="Your Book Title"
# OPENAI_API_KEY="your-api-key"
# QDRANT_URL="your-qdrant-url"
```

### 3. Install Dependencies
```bash
# Backend setup
cd backend
pip install -r requirements.txt

# Frontend setup
cd ../frontend
npm install
```

## Usage

### 1. Generate a Book
```bash
# Using the BookOutlineAgent and ChapterWriterAgent as specified in the spec
cd backend
python -m src.book_generator --title "${BOOK_TITLE}"
```

### 2. Process Content for RAG
```bash
# Embed book content using the RAGAgent
python -m src.rag_processor --book-id "book-identifier"
```

### 3. Start the Backend API
```bash
cd backend
uvicorn src.main:app --reload
```

### 4. Start the Frontend
```bash
cd frontend
npm start
```

### 5. Use the RAG Chatbot
1. Navigate to the book in the frontend
2. Ask questions in global QA mode or select text for selected-text QA
3. View answers with inline citations

## API Endpoints

### Content Processing
- `POST /embed` - Process and embed book content
- `GET /health` - Health check

### RAG Chatbot
- `POST /query` - Global QA mode (entire book)
- `POST /select` - Selected-text QA mode

## Configuration

### Environment Variables
- `BOOK_TITLE` - Title of the book to generate (required)
- `OPENAI_API_KEY` - API key for OpenAI (required for primary model)
- `OPENAI_MODEL` - Model to use for generation (default: gpt-4)
- `QDRANT_URL` - URL for Qdrant vector database (required)
- `QDRANT_API_KEY` - API key for Qdrant (if required)
- `COLLECTION_NAME` - Name of the vector collection (default: book_embeddings)

### Content Processing Settings
- `CHUNK_SIZE` - Size of text chunks in tokens (default: 500) (FR-009)
- `CHUNK_OVERLAP` - Overlap between chunks in tokens (default: 50) (FR-009)

## Development

### Specification-First Workflow
All code changes should be driven by specification updates:
1. Update spec.md with new requirements
2. Run code generation tools based on updated spec
3. Validate generated code against specification

### Running Tests
```bash
# Backend tests
cd backend
pytest

# Frontend tests
cd frontend
npm test
```

### Local Development
1. Make changes to specification documents
2. Generate code from updated specifications
3. Test functionality against original requirements
4. Update documentation as needed

## Troubleshooting

### Common Issues
- **API Key Errors**: Verify OPENAI_API_KEY is set correctly
- **Qdrant Connection**: Check QDRANT_URL and API key
- **Chunk Size Issues**: Ensure content fits within token limits

### Performance
- Responses should be within 2 seconds (p95 latency) (SC-002)
- If performance is slow, check vector database connection
- Monitor token usage for large books (Edge Case #105)

## Next Steps
1. Deploy frontend to GitHub Pages
2. Deploy backend to Render
3. Configure production environment variables
4. Monitor usage and performance metrics