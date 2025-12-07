"""
FastAPI backend for AI-Driven Book + RAG Chatbot
"""
import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from dotenv import load_dotenv
from typing import Dict
import json

from .schema import EmbedRequest, QueryRequest, QueryResponse
from .qdrant_manager import QdrantManager
from .embed import EmbeddingGenerator
from .rag import RAGEngine
from .agents.book_outline_agent import BookOutlineAgent
from .agents.chapter_writer_agent import ChapterWriterAgent
from .agents.rag_agent import RAGAgent
from .agents.api_integration_agent import APIIntegrationAgent
from .middleware.performance import PerformanceMiddleware, get_performance_stats, get_slow_requests, is_within_performance_threshold

load_dotenv()

app = FastAPI(
    title="AI Book RAG API",
    description="Backend API for AI-generated book with RAG chatbot",
    version="1.0.0"
)

# Add performance monitoring middleware
app.add_middleware(PerformanceMiddleware)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize components
qdrant_manager = QdrantManager()
embedding_generator = EmbeddingGenerator()
rag_engine = RAGEngine()

# Initialize AI agents for FR-019
book_outline_agent = BookOutlineAgent()
chapter_writer_agent = ChapterWriterAgent()
rag_agent = RAGAgent()
api_integration_agent = APIIntegrationAgent()


@app.on_event("startup")
async def startup_event():
    """Initialize Qdrant collection and AI agents on startup"""
    try:
        # Initialize Qdrant collection
        qdrant_manager.create_collection()
        print("Qdrant collection ready")

        # Health check for AI agents
        agents_health = await api_integration_agent.health_check()
        print(f"API Integration Agent health: {agents_health['agent_status']}")

        print("All AI agents initialized and ready")
    except Exception as e:
        print(f"Error during startup: {e}")


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "AI Book RAG API",
        "version": "1.0.0"
    }


@app.get("/performance")
async def performance_metrics():
    """Performance metrics endpoint - tracks response times and verifies 2-second requirement"""
    stats = get_performance_stats()
    return {
        "status": "success",
        "metrics": stats,
        "threshold_met": is_within_performance_threshold(),
        "requirement": "Response time should be under 2 seconds (p95 latency)"
    }


@app.get("/performance/slow-requests")
async def slow_requests_list():
    """List of requests that exceeded the 2-second threshold"""
    slow_reqs = get_slow_requests()
    return {
        "slow_requests": slow_reqs,
        "count": len(slow_reqs)
    }


@app.post("/embed")
async def embed_content(request: EmbedRequest):
    """Embed book content and store in Qdrant

    Args:
        request: EmbedRequest with content and chapter

    Returns:
        Success message with embedding count
    """
    try:
        # Chunk the content
        chunks = embedding_generator.chunk_text(request.content, request.chapter)

        # Generate embeddings
        embeddings = []
        for chunk in chunks:
            vector = embedding_generator.generate_embedding(chunk["text"])
            embeddings.append({
                "id": chunk["id"],
                "vector": vector,
                "chapter": chunk["chapter"],
                "text": chunk["text"],
                "token_count": chunk["token_count"]
            })

        # Store in Qdrant
        qdrant_manager.insert_embeddings(embeddings)

        return {
            "status": "success",
            "chapter": request.chapter,
            "chunks_created": len(embeddings)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/query")
async def query_global(request: QueryRequest):
    """Global QA endpoint - answer questions about the entire book

    Args:
        request: QueryRequest with user query

    Returns:
        Streaming response with answer, citations, and sources
    """
    try:
        if request.mode == "selected" and request.context:
            # Handle selected-text QA mode
            result = rag_engine.query_selected(request.query, request.context)
        else:
            # Handle global QA mode
            result = rag_engine.query_global(request.query)

        # Return as JSON response for compatibility
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/select")
async def query_selected(request: QueryRequest):
    """Selected-text QA endpoint - answer questions about highlighted text

    Args:
        request: QueryRequest with query and selected text context

    Returns:
        Response with answer, citations, and sources
    """
    try:
        if not request.context:
            raise HTTPException(
                status_code=400,
                detail="Context (selected text) is required for /select endpoint"
            )

        result = rag_engine.query_selected(request.query, request.context)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/embed-book")
async def embed_entire_book(book_content: Dict[str, str]):
    """Embed entire book at once

    Args:
        book_content: Dict mapping chapter names to content

    Returns:
        Success message
    """
    try:
        embeddings = embedding_generator.process_book(book_content)
        qdrant_manager.insert_embeddings(embeddings)

        return {
            "status": "success",
            "total_chunks": len(embeddings),
            "chapters": list(book_content.keys())
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    host = os.getenv("BACKEND_HOST", "0.0.0.0")
    port = int(os.getenv("BACKEND_PORT", "8000"))
    uvicorn.run(app, host=host, port=port)
