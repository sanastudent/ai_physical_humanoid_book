# AI-Driven Book + RAG Chatbot

An intelligent book generation and RAG-powered chatbot system for the hackathon project.

## Project Structure

```
book/
├── backend/                 # FastAPI backend
│   ├── src/
│   │   ├── main.py         # FastAPI endpoints
│   │   ├── embed.py        # Embedding generation
│   │   ├── rag.py          # RAG engine
│   │   ├── qdrant_client.py # Vector database
│   │   └── schema.py       # Data models
│   ├── requirements.txt
│   └── tests/
├── frontend/               # Docusaurus book
│   └── my-book/
│       ├── docs/           # Book content
│       ├── static/chatbot/ # Chat UI
│       └── src/theme/      # Text selection
├── .env                    # Configuration
└── README.md
```

## Setup Instructions

### Prerequisites

- Python 3.10+
- Node.js 18+
- Qdrant (running locally or cloud)
- API Keys:
  - Anthropic API key (required)
  - OpenAI API key (for embeddings)
  - Google API key (optional fallback)

### Backend Setup

1. **Install Qdrant**:
   ```bash
   docker run -p 6333:6333 qdrant/qdrant
   ```

2. **Configure Environment**:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

3. **Install Dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

4. **Run Backend**:
   ```bash
   cd backend/src
   python main.py
   ```

   Backend will be available at `http://localhost:8000`

### Frontend Setup

1. **Install Dependencies**:
   ```bash
   cd frontend/my-book
   npm install
   ```

2. **Configure Backend URL**:
   Create `frontend/my-book/.env.local`:
   ```
   REACT_APP_BACKEND_URL=http://localhost:8000
   ```

3. **Run Development Server**:
   ```bash
   npm start
   ```

   Frontend will be available at `http://localhost:3000`

## Usage

### Embedding Book Content

Send POST request to `/embed-book` with book chapters:

```bash
curl -X POST http://localhost:8000/embed-book \
  -H "Content-Type: application/json" \
  -d '{
    "introduction": "Content here...",
    "chapter1": "Content here..."
  }'
```

### Querying the Chatbot

#### Global QA Mode
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "mode": "global"
  }'
```

#### Selected Text QA Mode
```bash
curl -X POST http://localhost:8000/select \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept",
    "mode": "selected",
    "context": "Selected text from the book..."
  }'
```

## Deployment

### Backend (Render)

1. Create `render.yaml`:
   ```yaml
   services:
     - type: web
       name: book-rag-api
       env: python
       buildCommand: pip install -r requirements.txt
       startCommand: cd src && python main.py
   ```

2. Push to GitHub and connect to Render

### Frontend (GitHub Pages)

1. **Update Configuration**:
   Edit `frontend/my-book/docusaurus.config.ts`:
   - Set `url` to your GitHub Pages URL
   - Set `organizationName` to your GitHub username
   - Set `projectName` to your repo name

2. **Build and Deploy**:
   ```bash
   cd frontend/my-book
   npm run build
   GIT_USER=<your-username> npm run deploy
   ```

## API Endpoints

### Health Check
- **GET** `/health` - Server status

### Embedding
- **POST** `/embed` - Embed single chapter
- **POST** `/embed-book` - Embed entire book

### RAG Queries
- **POST** `/query` - Global book QA
- **POST** `/select` - Selected text QA

## Features

- ✓ AI-generated book content (Docusaurus)
- ✓ RAG-powered chatbot with citations
- ✓ Global and selected-text QA modes
- ✓ Streaming responses
- ✓ Vector search with Qdrant
- ✓ GitHub Pages deployment
- ✓ Dark mode support

## Tech Stack

- **Backend**: FastAPI, Qdrant, Anthropic Claude
- **Frontend**: Docusaurus, React
- **AI**: Claude for generation, OpenAI for embeddings
- **Deployment**: Render (backend), GitHub Pages (frontend)

## Development

### Running Tests
```bash
cd backend
pytest tests/
```

### Linting
```bash
cd frontend/my-book
npm run lint
```

### Building for Production
```bash
cd frontend/my-book
npm run build
```

## License

MIT

## Credits

Built for the AI Hackathon using Claude Code and Spec-Driven Development.
