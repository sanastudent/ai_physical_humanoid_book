"""
Vercel Serverless Function Entry Point for FastAPI
"""
from src.main import app

# This allows Vercel to use the FastAPI app as a serverless function
# The app variable is imported from the main module
