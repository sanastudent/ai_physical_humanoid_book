# Implementation Guide

## Quick Start (Step-by-Step)

### 1. Prerequisites

Install required software:
- Python 3.10 or higher
- Node.js 18 or higher
- Docker (for Qdrant)
- Git

### 2. Clone and Configure

```bash
# Clone the repository
git clone https://github.com/your-username/book.git
cd book

# Create and configure .env file
cp .env .env.example  # Use as template
nano .env  # Edit with your API keys
```

Required API Keys:
- `ANTHROPIC_API_KEY` - Get from https://console.anthropic.com/
- `OPENAI_API_KEY` - Get from https://platform.openai.com/api-keys (for embeddings)

Optional:
- `GOOGLE_API_KEY` - Get from https://makersuite.google.com/app/apikey

### 3. Start Qdrant Vector Database

```bash
docker run -d -p 6333:6333 --name qdrant-book qdrant/qdrant
```

Verify it's running:
```bash
curl http://localhost:6333/collections
```

### 4. Setup Backend

```bash
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Start backend server
cd src
python main.py
```

Backend runs on `http://localhost:8000`

Test health:
```bash
curl http://localhost:8000/health
```

### 5. Setup Frontend

Open a new terminal:

```bash
cd frontend/my-book

# Install dependencies
npm install

# Create environment file
echo "REACT_APP_BACKEND_URL=http://localhost:8000" > .env.local

# Start development server
npm start
```

Frontend runs on `http://localhost:3000`

### 6. Embed Book Content

Open another terminal:

```bash
# Make sure backend is running first!
python scripts/embed_book.py
```

This reads all markdown files from `frontend/my-book/docs/` and creates embeddings.

### 7. Test the System

```bash
# Test RAG queries
python scripts/test_rag.py

# Or test manually in browser
# Navigate to http://localhost:3000
# Click the chatbot button and ask questions!
```

## Architecture Overview

```
┌─────────────┐
│   User      │
└──────┬──────┘
       │
       v
┌─────────────────────────────────────┐
│  Docusaurus Frontend                │
│  - Book content (Markdown)          │
│  - ChatUI component                 │
│  - Text selection handler           │
└──────────┬──────────────────────────┘
           │ HTTP POST
           v
┌─────────────────────────────────────┐
│  FastAPI Backend                    │
│  - /query (Global QA)               │
│  - /select (Selected-text QA)       │
│  - /embed (Embedding endpoint)      │
└──────────┬──────────────────────────┘
           │
           ├───> Anthropic Claude (LLM)
           │
           └───> ┌─────────────────────┐
                 │  Qdrant              │
                 │  Vector Database     │
                 │  - Book embeddings   │
                 │  - Similarity search │
                 └─────────────────────┘
```

## Development Workflow

### Adding New Book Content

1. Create markdown file in `frontend/my-book/docs/chapters/`
2. Update `frontend/my-book/sidebars.ts` to include new chapter
3. Re-embed: `python scripts/embed_book.py`
4. Restart frontend: `npm start`

### Modifying RAG Behavior

Edit `backend/src/rag.py`:
- `retrieve_context()` - Change search parameters
- `generate_answer()` - Modify prompt templates
- `_extract_citations()` - Customize citation format

### Customizing Chat UI

Edit `frontend/my-book/static/chatbot/`:
- `ChatUI.jsx` - React component logic
- `chatui.css` - Styling and layout

## Testing

### Backend Unit Tests

```bash
cd backend
pytest tests/ -v
```

### Frontend Testing

```bash
cd frontend/my-book
npm test
```

### Integration Testing

```bash
# Start all services, then:
python scripts/test_rag.py
```

## Deployment

### Backend to Render

1. Create `render.yaml` in project root:

```yaml
services:
  - type: web
    name: book-rag-api
    env: python
    plan: free
    buildCommand: pip install -r backend/requirements.txt
    startCommand: cd backend/src && python main.py
    envVars:
      - key: ANTHROPIC_API_KEY
        sync: false
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_HOST
        value: your-qdrant-host
```

2. Push to GitHub
3. Connect repository to Render
4. Add environment variables in Render dashboard

### Frontend to GitHub Pages

1. Update `frontend/my-book/docusaurus.config.ts`:

```typescript
url: 'https://your-username.github.io',
baseUrl: '/book/',
organizationName: 'your-username',
projectName: 'book',
```

2. Deploy:

```bash
cd frontend/my-book
npm run build
GIT_USER=your-username npm run deploy
```

3. Enable GitHub Pages in repository settings:
   - Settings → Pages → Source: gh-pages branch

### Qdrant Cloud (Production)

1. Sign up at https://cloud.qdrant.io
2. Create a cluster
3. Update `.env`:

```
QDRANT_HOST=your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-api-key
```

## Troubleshooting

### Backend won't start

**Error**: "No API key found"
**Solution**: Check `.env` file has `ANTHROPIC_API_KEY` set

**Error**: "Connection refused to Qdrant"
**Solution**: Ensure Qdrant is running on port 6333

### Frontend issues

**Error**: "Cannot connect to backend"
**Solution**: Check `.env.local` has correct `REACT_APP_BACKEND_URL`

**Error**: CORS errors
**Solution**: Backend's CORS settings may need adjustment in `backend/src/main.py`

### Embedding fails

**Error**: "OpenAI API key required"
**Solution**: Embeddings need OpenAI key. Add `OPENAI_API_KEY` to `.env`

**Error**: "Context length exceeded"
**Solution**: Reduce chunk size in `backend/src/embed.py` (CHUNK_SIZE variable)

### RAG returns poor results

**Issue**: Irrelevant answers
**Solution**:
- Increase number of retrieved chunks (limit parameter)
- Improve prompt in `backend/src/rag.py`
- Re-embed with better chunk strategy

## Performance Optimization

### Backend

1. **Caching**: Add Redis for repeated queries
2. **Batch Processing**: Process multiple chunks in parallel
3. **Model Selection**: Use faster models for simple queries

### Frontend

1. **Code Splitting**: Lazy load chat component
2. **Debouncing**: Delay queries while user types
3. **Pagination**: Load book chapters on demand

## Security Considerations

1. **API Keys**: Never commit `.env` to Git
2. **Rate Limiting**: Add to backend endpoints
3. **Input Validation**: Sanitize user queries
4. **CORS**: Configure appropriately for production
5. **HTTPS**: Use SSL in production

## Next Steps

1. Expand book content with more chapters
2. Improve citation accuracy
3. Add user authentication
4. Implement feedback mechanism
5. Add analytics and monitoring

## Support

For issues and questions:
- Check existing GitHub issues
- Create new issue with detailed description
- Include logs and error messages
