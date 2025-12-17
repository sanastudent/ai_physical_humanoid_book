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

### 🚀 Quick Deploy (Recommended)

**Frontend → Vercel | Backend → Railway**

See [QUICK_DEPLOY.md](./QUICK_DEPLOY.md) for 10-minute deployment guide.

### Deployment Options

#### Option 1: Vercel + Railway (Recommended)
- **Frontend (Docusaurus)** → Deploy to Vercel for global CDN
- **Backend (FastAPI)** → Deploy to Railway for persistent connections
- **Best for**: Production apps with RAG chatbot

📖 **Full Guide**: [VERCEL_DEPLOYMENT_GUIDE.md](./VERCEL_DEPLOYMENT_GUIDE.md)

#### Option 2: GitHub Pages (Static Only)
- **Frontend only** → Deploy static Docusaurus site
- **No backend features**: Chatbot, authentication, personalization won't work
- **Best for**: Documentation-only projects

```bash
cd frontend/my-book
npm run build
GIT_USER=<your-username> npm run deploy
```

#### Platform Comparison
See [DEPLOYMENT_PLATFORMS_COMPARISON.md](./DEPLOYMENT_PLATFORMS_COMPARISON.md) for detailed feature comparison.

### Environment Variables

Copy `.env.example` to `.env` and configure:
```bash
cp .env.example .env
# Add your API keys and configuration
```

Required:
- `GOOGLE_API_KEY` - Google Gemini API
- `ANTHROPIC_API_KEY` - Claude API
- `QDRANT_URL` and `QDRANT_API_KEY` - Qdrant Cloud
- `NEON_DB_URL` - Neon Postgres
- `BETTERAUTH_SECRET` - Random 32+ char string

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

### Core Features
- ✅ AI-generated book content (Docusaurus)
- ✅ RAG-powered chatbot with Gemini
- ✅ Global and selected-text QA modes
- ✅ Vector search with Qdrant Cloud
- ✅ Dark mode support

### Advanced Features
- ✅ User authentication (BetterAuth + Neon)
- ✅ Content personalization (based on user background)
- ✅ Translation to Urdu (RTL support)
- ✅ Multilingual i18n support

### Deployment
- ✅ Vercel deployment (frontend)
- ✅ Railway deployment (backend)
- ✅ GitHub Pages option (static only)

## Tech Stack

- **Backend**: FastAPI, Qdrant Cloud, Google Gemini
- **Frontend**: Docusaurus, React 19, TypeScript
- **AI**: Google Gemini for RAG, Anthropic Claude for personalization
- **Database**: Neon Serverless Postgres
- **Authentication**: BetterAuth
- **Deployment**: Vercel (frontend), Railway (backend)

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
