#!/bin/bash

# Setup script for AI-Driven Book + RAG Chatbot

set -e

echo "===================================="
echo "AI-Driven Book + RAG Chatbot Setup"
echo "===================================="

# Check prerequisites
command -v python3 >/dev/null 2>&1 || { echo "Error: Python 3 is required but not installed."; exit 1; }
command -v node >/dev/null 2>&1 || { echo "Error: Node.js is required but not installed."; exit 1; }
command -v docker >/dev/null 2>&1 || { echo "Warning: Docker is recommended for Qdrant."; }

# Create virtual environment for backend
echo "\n[1/5] Setting up Python virtual environment..."
cd backend
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
cd ..

# Install frontend dependencies
echo "\n[2/5] Installing frontend dependencies..."
cd frontend/my-book
npm install
cd ../..

# Check for .env file
echo "\n[3/5] Checking configuration..."
if [ ! -f .env ]; then
    echo "Warning: .env file not found. Creating from template..."
    cp .env .env.example 2>/dev/null || echo "Please create .env file manually"
fi

# Start Qdrant (if Docker is available)
echo "\n[4/5] Starting Qdrant vector database..."
if command -v docker >/dev/null 2>&1; then
    docker pull qdrant/qdrant
    docker run -d -p 6333:6333 --name qdrant-book qdrant/qdrant || echo "Qdrant container may already be running"
else
    echo "Docker not found. Please start Qdrant manually."
fi

echo "\n[5/5] Setup complete!"
echo "\nNext steps:"
echo "1. Edit .env file with your API keys"
echo "2. Start backend: cd backend/src && python main.py"
echo "3. Start frontend: cd frontend/my-book && npm start"
echo "4. Embed book content: python scripts/embed_book.py"
echo "5. Test RAG: python scripts/test_rag.py"
echo "\n===================================="
