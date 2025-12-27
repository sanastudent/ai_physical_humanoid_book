import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.main import app
import uvicorn

if __name__ == "__main__":
    host = os.getenv("BACKEND_HOST", "0.0.0.0")
    port = int(os.getenv("BACKEND_PORT", "8000"))
    uvicorn.run(app, host=host, port=port)