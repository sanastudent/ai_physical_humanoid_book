"""
API Integration Agent for handling backend API endpoints

Implements FR-019: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration
"""
import asyncio
import os
from typing import Dict, Any, Optional
from pydantic import BaseModel
import google.generativeai as genai
from dotenv import load_dotenv

from src.config.ai_config import ai_config
from src.skills.rag import RAGSkill
from src.skills.content_processing import ContentProcessingSkill
from src.agents.book_outline_agent import BookOutlineAgent
from src.agents.chapter_writer_agent import ChapterWriterAgent
from src.agents.rag_agent import RAGAgent

load_dotenv()

class APIRequest(BaseModel):
    """Represents an API request with method, endpoint, and payload"""
    method: str
    endpoint: str
    payload: Dict[str, Any]
    headers: Optional[Dict[str, str]] = None


class APIResponse(BaseModel):
    """Represents an API response"""
    status_code: int
    data: Dict[str, Any]
    message: str
    success: bool


class APIIntegrationAgent:
    """
    Agent responsible for handling API integrations and coordinating between different services.

    Implements FR-019: Specialized AI agents and skills MUST be utilized for book generation, RAG, and service integration
    """

    def __init__(self):
        self.gemini_key = ai_config.primary_api_key or os.getenv("GOOGLE_API_KEY")
        genai.configure(api_key=self.gemini_key)
        self.model = genai.GenerativeModel(ai_config.primary_model_name or 'gemini-pro')

        # Initialize specialized agents
        self.book_outline_agent = BookOutlineAgent()
        self.chapter_writer_agent = ChapterWriterAgent()
        self.rag_agent = RAGAgent()

        # Initialize skills
        self.rag_skill = RAGSkill()
        self.content_processing_skill = ContentProcessingSkill()

    async def handle_embed_request(self, payload: Dict[str, Any]) -> APIResponse:
        """
        Handle /embed endpoint requests for content embedding.

        Args:
            payload: Request payload containing content and metadata

        Returns:
            APIResponse with embedding results
        """
        try:
            # Use content processing skill to handle the embedding
            result = await self.content_processing_skill.execute(
                content=payload.get("content", ""),
                chapter=payload.get("chapter", "unknown"),
                book_id=payload.get("book_id", "default"),
                chunk_size=payload.get("chunk_size", 500),
                overlap=payload.get("overlap", 50)
            )

            return APIResponse(
                status_code=200,
                data=result,
                message="Content processed and embedded successfully",
                success=True
            )
        except Exception as e:
            return APIResponse(
                status_code=500,
                data={},
                message=f"Error processing embed request: {str(e)}",
                success=False
            )

    async def handle_query_request(self, payload: Dict[str, Any]) -> APIResponse:
        """
        Handle /query endpoint requests for global QA.

        Args:
            payload: Request payload containing query and parameters

        Returns:
            APIResponse with query results
        """
        try:
            # Determine the mode of the query
            mode = payload.get("mode", "global")

            if mode == "selected" and payload.get("context"):
                # Handle selected-text QA
                result = await self.rag_agent.query_selected_text(
                    query=payload.get("query", ""),
                    selected_text=payload.get("context", ""),
                    book_id=payload.get("book_id", "default")
                )
            else:
                # Handle global QA
                result = await self.rag_agent.query_global(
                    query=payload.get("query", ""),
                    book_id=payload.get("book_id", "default")
                )

            return APIResponse(
                status_code=200,
                data=result.dict() if hasattr(result, 'dict') else result,
                message="Query processed successfully",
                success=True
            )
        except Exception as e:
            return APIResponse(
                status_code=500,
                data={},
                message=f"Error processing query request: {str(e)}",
                success=False
            )

    async def handle_select_request(self, payload: Dict[str, Any]) -> APIResponse:
        """
        Handle /select endpoint requests for selected-text QA.

        Args:
            payload: Request payload containing query and selected text context

        Returns:
            APIResponse with selected-text QA results
        """
        try:
            # Validate required parameters
            if not payload.get("context"):
                return APIResponse(
                    status_code=400,
                    data={},
                    message="Context (selected text) is required for /select endpoint",
                    success=False
                )

            # Handle selected-text QA
            result = await self.rag_agent.query_selected_text(
                query=payload.get("query", ""),
                selected_text=payload.get("context", ""),
                book_id=payload.get("book_id", "default")
            )

            return APIResponse(
                status_code=200,
                data=result.dict() if hasattr(result, 'dict') else result,
                message="Selected-text query processed successfully",
                success=True
            )
        except Exception as e:
            return APIResponse(
                status_code=500,
                data={},
                message=f"Error processing select request: {str(e)}",
                success=False
            )

    async def process_api_request(self, api_request: APIRequest) -> APIResponse:
        """
        Process an API request based on its endpoint and method.

        Args:
            api_request: The API request to process

        Returns:
            APIResponse with the result of processing
        """
        endpoint = api_request.endpoint.lower().strip('/')

        if endpoint == "embed":
            return await self.handle_embed_request(api_request.payload)
        elif endpoint == "query":
            return await self.handle_query_request(api_request.payload)
        elif endpoint == "select":
            return await self.handle_select_request(api_request.payload)
        else:
            return APIResponse(
                status_code=404,
                data={},
                message=f"Endpoint '{endpoint}' not supported by APIIntegrationAgent",
                success=False
            )

    async def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check of the API integration agent and its dependencies.

        Returns:
            Dictionary with health check results
        """
        health_results = {
            "agent_status": "healthy",
            "dependencies": {
                "book_outline_agent": True,
                "chapter_writer_agent": True,
                "rag_agent": True,
                "rag_skill": True,
                "content_processing_skill": True
            },
            "api_connection": False,
            "timestamp": asyncio.get_event_loop().time()
        }

        try:
            # Test API connection by making a simple request
            # For Gemini, we'll test by generating a simple response
            test_response = await self.model.generate_content_async("Hello")
            health_results["api_connection"] = test_response.text is not None
        except Exception:
            health_results["api_connection"] = False
            health_results["agent_status"] = "degraded"

        # Check if any dependency failed
        if not all(health_results["dependencies"].values()) or not health_results["api_connection"]:
            health_results["agent_status"] = "degraded"

        return health_results

    async def validate_request_payload(self, endpoint: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate the request payload for the specified endpoint.

        Args:
            endpoint: The API endpoint
            payload: The request payload to validate

        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "is_valid": True,
            "errors": [],
            "warnings": []
        }

        if endpoint == "embed":
            required_fields = ["content", "chapter"]
            for field in required_fields:
                if field not in payload or not payload[field]:
                    validation_results["is_valid"] = False
                    validation_results["errors"].append(f"Missing required field: {field}")

        elif endpoint == "query":
            if "query" not in payload or not payload["query"]:
                validation_results["is_valid"] = False
                validation_results["errors"].append("Missing required field: query")

        elif endpoint == "select":
            if "query" not in payload or not payload["query"]:
                validation_results["is_valid"] = False
                validation_results["errors"].append("Missing required field: query")

            if "context" not in payload or not payload["context"]:
                validation_results["is_valid"] = False
                validation_results["errors"].append("Missing required field: context")

        return validation_results