# AI Book RAG Backend

FastAPI backend for AI-powered book with RAG chatbot, authentication, and personalization features.

## Features

- 🤖 RAG chatbot using Google Gemini
- 🔐 User authentication with BetterAuth
- 👤 Content personalization based on user background
- 💾 Qdrant vector database for embeddings
- 🗄️ Neon Postgres for user data
- 🚀 Ready for Vercel/Railway deployment

## Tech Stack

- **Framework**: FastAPI
- **AI**: Google Gemini, Anthropic Claude
- **Vector DB**: Qdrant Cloud
- **SQL DB**: Neon Serverless Postgres
- **Deployment**: Railway (recommended), Vercel (optional)

## Quick Start

### Prerequisites

- Python 3.11+
- API keys: Google AI, Anthropic, Qdrant, Neon

### Installation

```bash
cd backend
pip install -r requirements.txt
```

### Configuration

Create `.env` file in backend directory:

```bash
GOOGLE_API_KEY=your-google-api-key
ANTHROPIC_API_KEY=your-anthropic-api-key
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536
NEON_DB_URL=postgresql://user:pass@host/db?sslmode=require
BETTERAUTH_SECRET=your-32-char-secret
```

### Run Locally

```bash
cd src
python main.py
```

API available at: `http://localhost:8000`
API docs: `http://localhost:8000/docs`

## Deployment

### Railway (Recommended)

1. Connect GitHub repository to Railway
2. Set root directory to `backend`
3. Add environment variables
4. Railway auto-detects `Procfile` and deploys

### Vercel (Alternative)

1. Install Vercel CLI: `npm i -g vercel`
2. Deploy: `cd backend && vercel --prod`
3. Add environment variables in Vercel dashboard

See `RAILWAY_DEPLOYMENT_GUIDE.md` for detailed instructions.

## API Endpoints

### Health
- `GET /health` - Health check

### Authentication
- `POST /api/auth/signup` - User signup
- `POST /api/auth/signin` - User signin
- `GET /api/auth/me` - Get current user

### RAG Chatbot
- `POST /api/rag/query` - Query book content
- `POST /api/rag/selected` - Query selected text

### Personalization
- `POST /api/personalize` - Get personalized content
- `POST /api/preferences` - Update user preferences

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app
│   ├── qdrant_manager.py    # Vector database
│   ├── embed.py             # Embeddings
│   ├── rag.py               # RAG engine
│   ├── schema.py            # Data models
│   ├── agents/              # AI agents
│   ├── health/              # Health checks
│   ├── middleware/          # Auth & performance
│   ├── models/              # Database models
│   └── routes/              # API routes
├── requirements.txt
├── Procfile                 # Railway config
├── vercel.json              # Vercel config
└── README.md
```

## Environment Variables

All required environment variables are documented in the root `.env.example` file.

## Testing

```bash
# Run tests
pytest tests/

# Test health endpoint
python verify_health_simple.py

# Test RAG functionality
python test_gemini_rag.py
```

## License

MIT
