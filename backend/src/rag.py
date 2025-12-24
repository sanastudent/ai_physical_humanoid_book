"""
RAG (Retrieval Augmented Generation) logic with support for multiple LLM providers
"""
import os
import time
import random
import requests
from typing import List, Dict, Optional, Any
import google.generativeai as genai
from dotenv import load_dotenv

from .qdrant_manager import QdrantManager
from .embed import EmbeddingGenerator

load_dotenv()


class RAGEngine:
    """Handles RAG queries and response generation with support for multiple LLM providers"""

    def __init__(self):
        self.qdrant = QdrantManager()
        self.embedder = EmbeddingGenerator()

        # Check for available LLM providers in order of preference
        self.gemini_key = os.getenv("GOOGLE_API_KEY")
        self.grok_key = os.getenv("GROK_KEY")  # Added for Grok API
        self.openai_key = os.getenv("OPENAI_KEY")  # Changed from OPENAI_API_KEY to match .env
        self.anthropic_key = os.getenv("ANTHROPIC_API_KEY")

        # Initialize the best available model
        self.model = None
        self.provider = None

        # Try each API in order with fallbacks
        # Order: OpenAI (gpt-4o-mini) -> Grok -> Google (Gemini) -> Anthropic -> Local
        self.providers_order = []

        # Check if API keys are valid format before using them
        valid_google_key = self.gemini_key and (self.gemini_key.startswith("AI") and not self.gemini_key.startswith("gsk_"))
        valid_openai_key = self.openai_key and (self.openai_key.startswith("sk-") and not self.openai_key.startswith("gsk_"))
        valid_grok_key = self.grok_key and self.grok_key.startswith("gsk_")  # Grok keys start with gsk_
        valid_anthropic_key = self.anthropic_key and self.anthropic_key.startswith("sk-ant-")

        # Add valid providers to order with PRIORITY: OpenAI > Grok > Gemini
        if valid_openai_key:
            self.providers_order.append(("openai", self.openai_key))
        if valid_grok_key:
            self.providers_order.append(("grok", self.grok_key))
        if valid_google_key:
            self.providers_order.append(("google", self.gemini_key))
        if valid_anthropic_key:
            self.providers_order.append(("anthropic", self.anthropic_key))

        # Try to initialize the first available provider
        print(f"API Priority Order: {[p[0] for p in self.providers_order]}")
        for provider_name, api_key in self.providers_order:
            if self._initialize_provider(provider_name, api_key):
                self.provider = provider_name
                print(f"[OK] Successfully initialized: {provider_name}")
                break
            else:
                print(f"[FAIL] Failed to initialize: {provider_name}")

        # If no API providers are available, use local models
        if not self.provider:
            # Use local model as fallback
            try:
                from transformers import pipeline
                # Use a lightweight model for local inference
                self.model = pipeline("text-generation", model="gpt2", max_length=100, truncation=True)
                self.provider = "local"
                print("Local LLM model initialized successfully")
            except Exception as e:
                print(f"Warning: Local LLM model failed to initialize: {e}")
                try:
                    # Fallback to a simpler approach with sentence transformers for embeddings only
                    # and basic text processing
                    from sentence_transformers import SentenceTransformer
                    self.local_embedding_model = SentenceTransformer('all-MiniLM-L6-v2', cache_folder='./models')
                    self.model = "basic_local"  # Use a simple identifier
                    self.provider = "basic_local"
                    print("Basic local processing initialized successfully")
                except ImportError:
                    print("Warning: sentence-transformers not installed. Using basic fallback.")
                    self.model = "basic_fallback"  # Use a simple identifier
                    self.provider = "basic_fallback"
                    print("Basic fallback processing initialized successfully")
                except Exception as e2:
                    print(f"Warning: No LLM provider available. Local models not available: {e2}")
                    self.model = None
                    self.provider = "none"

    def _initialize_provider(self, provider_name, api_key):
        """Initialize a specific provider and return True if successful"""
        try:
            if provider_name == "google":
                import google.generativeai as genai
                genai.configure(api_key=api_key)
                # Use gemini-2.5-flash instead of deprecated gemini-1.5-flash
                self.model = genai.GenerativeModel('gemini-2.5-flash')
                return True
            elif provider_name == "openai":
                from openai import OpenAI
                # Use gpt-4o-mini as requested
                client = OpenAI(api_key=api_key)
                self.model = client
                self.openai_client = client  # Store for later use
                return True
            elif provider_name == "grok":
                # For Grok, we'll use the xAI API or another compatible approach
                # Store the key and provider info for later use in generate_answer
                self.grok_key = api_key
                self.model = "grok"
                return True
            elif provider_name == "anthropic":
                from anthropic import Anthropic
                client = Anthropic(api_key=api_key)
                self.model = client
                return True
        except Exception as e:
            print(f"Warning: {provider_name} API initialization failed: {e}")
            return False
        return False

    def _try_providers_in_order(self, prompt, context_str, query):
        """Try each provider in order until one succeeds, always return some response"""
        # First try the primary provider that was initialized
        if self.provider:
            try:
                print(f"Using primary provider: {self.provider}")
                answer = self._generate_with_provider(self.provider, prompt)
                if answer and "error" not in answer.lower():
                    print(f"[OK] Primary provider {self.provider} succeeded")
                    return answer
            except Exception as e:
                print(f"[FAIL] Primary provider {self.provider} failed: {e}")

        # If primary failed, try all available providers in order
        for provider_name, api_key in self.providers_order:
            if provider_name != self.provider:  # Skip if it's the same as primary
                try:
                    # Initialize this provider temporarily
                    original_model = self.model
                    original_provider = self.provider

                    if self._initialize_provider(provider_name, api_key):
                        self.provider = provider_name
                        answer = self._generate_with_provider(provider_name, prompt)
                        if answer and "error" not in answer.lower():
                            return answer
                    # Restore original provider
                    self.model = original_model
                    self.provider = original_provider
                except Exception as e:
                    print(f"Provider {provider_name} failed during query: {e}")
                    continue

        # If all API providers failed, try local providers
        local_providers = ["local", "basic_local", "basic_fallback"]
        for local_provider in local_providers:
            if local_provider != self.provider:
                try:
                    original_model = self.model
                    original_provider = self.provider

                    # Set up local provider
                    if local_provider == "local":
                        from transformers import pipeline
                        self.model = pipeline("text-generation", model="gpt2", max_length=100, truncation=True)
                        self.provider = "local"
                    elif local_provider == "basic_local":
                        from sentence_transformers import SentenceTransformer
                        self.local_embedding_model = SentenceTransformer('all-MiniLM-L6-v2', cache_folder='./models')
                        self.model = "basic_local"
                        self.provider = "basic_local"
                    elif local_provider == "basic_fallback":
                        self.model = "basic_fallback"
                        self.provider = "basic_fallback"

                    answer = self._generate_with_provider(local_provider, prompt)
                    if answer and "error" not in answer.lower():
                        return answer

                    # Restore original provider
                    self.model = original_model
                    self.provider = original_provider
                except Exception as e:
                    print(f"Local provider {local_provider} failed: {e}")
                    continue

        # If everything fails, return a helpful message with context
        if context_str.strip():
            return f"Based on the provided context: {context_str}\n\nAnswer: {query} - [All providers failed, showing context only]"
        else:
            return f"Question: {query}\n\nAnswer: All available providers failed to generate a response. Please check your API keys and connectivity."

    def _generate_with_provider(self, provider_name, prompt):
        """Generate response using a specific provider"""
        if provider_name == "google":
            def _call_google_api():
                return self.model.generate_content(
                    prompt,
                    generation_config=genai.GenerationConfig(
                        max_output_tokens=1024,
                        temperature=0.7
                    )
                )
            response = self._retry_with_exponential_backoff(_call_google_api)
            return response.text
        elif provider_name == "openai":
            def _call_openai_api():
                response = self.openai_client.chat.completions.create(
                    model="gpt-4o-mini",  # Use gpt-4o-mini as requested
                    messages=[
                        {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context and provides citations."},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=1024,
                    temperature=0.7
                )
                return response.choices[0].message.content
            return self._retry_with_exponential_backoff(_call_openai_api)
        elif provider_name == "grok":
            # For Grok, we'll use a basic implementation with requests
            import requests
            headers = {
                "Authorization": f"Bearer {self.grok_key}",
                "Content-Type": "application/json"
            }
            data = {
                "model": "grok-beta",  # Use appropriate Grok model
                "messages": [
                    {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context and provides citations."},
                    {"role": "user", "content": prompt}
                ],
                "max_tokens": 1024,
                "temperature": 0.7
            }
            response = requests.post("https://api.x.ai/v1/chat/completions", headers=headers, json=data)
            if response.status_code == 200:
                return response.json()["choices"][0]["message"]["content"]
            else:
                raise Exception(f"Grok API error: {response.status_code} - {response.text}")
        elif provider_name == "anthropic":
            def _call_anthropic_api():
                response = self.model.messages.create(
                    model="claude-3-haiku-20240307",
                    max_tokens=1024,
                    temperature=0.7,
                    system="You are a helpful assistant that answers questions based on provided context and provides citations.",
                    messages=[
                        {"role": "user", "content": prompt}
                    ]
                )
                return response.content[0].text
            return self._retry_with_exponential_backoff(_call_anthropic_api)
        elif provider_name == "local":
            # Use local model for generation
            local_prompt = f"Context: {prompt}\n\nAnswer:"
            response = self.model(local_prompt, max_length=200, num_return_sequences=1, pad_token_id=50256)
            return response[0]['generated_text'].replace(local_prompt, "").strip()
        elif provider_name == "basic_local":
            # Basic local processing - just return a simple response based on context
            return f"This response was generated using basic local processing since no advanced LLM provider is available. The answer is based on the provided context."
        elif provider_name == "basic_fallback":
            # Even more basic fallback
            return f"This response is generated as a fallback when no other providers are available."
        else:
            raise Exception(f"Unknown provider: {provider_name}")

    def _retry_with_exponential_backoff(self, func, *args, max_retries=3, base_delay=1, max_delay=60):
        """
        Execute a function with exponential backoff retry logic

        Args:
            func: Function to execute
            *args: Arguments to pass to the function
            max_retries: Maximum number of retry attempts
            base_delay: Base delay in seconds
            max_delay: Maximum delay in seconds
        """
        for attempt in range(max_retries + 1):
            try:
                return func(*args)
            except Exception as e:
                if attempt == max_retries:
                    # Last attempt, raise the exception
                    raise e

                # Check if it's a rate limit error
                error_str = str(e).lower()
                if "quota" not in error_str and "rate limit" not in error_str and "429" not in error_str:
                    # If it's not a rate limit error, don't retry
                    raise e

                # Calculate delay with exponential backoff and jitter
                delay = min(base_delay * (2 ** attempt) + random.uniform(0, 1), max_delay)
                print(f"Rate limit hit, retrying in {delay:.2f} seconds... (attempt {attempt + 1}/{max_retries})")
                time.sleep(delay)

        # This should not be reached
        raise Exception("Max retries exceeded")

    def retrieve_context(self, query: str, limit: int = 5) -> List[Dict]:
        """Retrieve relevant context from vector database

        Args:
            query: User query
            limit: Number of chunks to retrieve

        Returns:
            List of relevant chunks with scores
        """
        try:
            # Generate query embedding
            query_vector = self.embedder.generate_embedding(query)

            # Search Qdrant with error handling for database unavailability (Edge Case #106)
            results = self.qdrant.search(query_vector, limit=limit)

            # Format results
            context_chunks = []
            for result in results:
                context_chunks.append({
                    "text": result.payload["text"],
                    "chapter": result.payload["chapter"],
                    "score": result.score
                })

            return context_chunks
        except Exception as e:
            print(f"Qdrant database unavailable or collection not found: {e}")
            # Return empty results but allow graceful degradation
            # The LLM will still generate an answer, just without retrieved context
            return []

    def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict],
        mode: str = "global"
    ) -> Dict:
        """Generate answer using LLM with retrieved context

        Args:
            query: User query
            context_chunks: Retrieved context chunks
            mode: "global" or "selected"

        Returns:
            Dict with answer, citations, and sources
        """
        # Build context string
        context_str = "\n\n".join([
            f"[{chunk['chapter']}]\n{chunk['text']}"
            for chunk in context_chunks
        ])

        # Build prompt
        if mode == "selected":
            prompt = f"""You are an expert assistant answering questions about a book.
Use ONLY the provided context to answer the question. Do not use external knowledge.

Context:
{context_str}

Question: {query}

Provide a detailed answer with inline citations in the format [Chapter X: Paragraph Y] where X is the chapter number and Y is the paragraph number.
If the answer cannot be found in the context, say so."""
        else:
            prompt = f"""You are an expert assistant answering questions about a book on Embodied AI and Robotics.
Use the provided context from the book to answer the question accurately.

Context from the book:
{context_str}

Question: {query}

Provide a comprehensive answer with inline citations in the format [Chapter X: Paragraph Y] where X is the chapter number and Y is the paragraph number.
Use your knowledge to supplement, but prioritize information from the provided context."""

        try:
            # Generate response with robust fallback system - try each provider until one works
            answer = self._try_providers_in_order(prompt, context_str, query)

            # Extract citations
            citations = self._extract_citations(answer)

            # Validate citations for Edge Case #107
            citation_validation = self.validate_citations(citations, context_chunks)

            # Build sources list
            sources = [
                {
                    "chapter": chunk["chapter"],
                    "text": chunk["text"][:200] + "...",  # Truncate for brevity
                    "score": chunk["score"]
                }
                for chunk in context_chunks
            ]

            return {
                "answer": answer,
                "citations": citations,
                "sources": sources,
                "citation_validation": citation_validation
            }
        except Exception as e:
            print(f"Error generating answer: {e}")
            # Check if it's a quota exceeded error
            error_str = str(e).lower()
            if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
                # Return a more specific error message for quota issues
                return {
                    "answer": "Sorry, I've reached the API quota limit. Please check your API billing settings and quota limits for your provider. Using local models is recommended to avoid these issues.",
                    "citations": [],
                    "sources": [],
                    "citation_validation": {
                        "total_citations": 0,
                        "valid_citations": [],
                        "invalid_citations": [],
                        "has_valid_citations": False,
                        "all_citations_valid": True,
                        "validation_details": [{"citation": "None", "valid": False, "reason": "API quota exceeded"}]
                    }
                }
            else:
                # Return error message directly without adding "Context:" prefix to avoid recursive nesting
                return {
                    "answer": "Sorry, I encountered an error processing your question. Please make sure the backend server is running.",
                    "citations": [],
                    "sources": [],
                    "citation_validation": {
                        "total_citations": 0,
                        "valid_citations": [],
                        "invalid_citations": [],
                        "has_valid_citations": False,
                        "all_citations_valid": True,
                        "validation_details": [{"citation": "None", "valid": False, "reason": "Error occurred during answer generation"}]
                    }
                }

    def query_global(self, query: str) -> Dict:
        """Process global QA query

        Args:
            query: User question

        Returns:
            Response dict with answer and citations
        """
        try:
            context_chunks = self.retrieve_context(query, limit=5)

            # Add warning if no context was retrieved due to Qdrant issues
            if not context_chunks:
                print("Warning: No context retrieved. Qdrant database may be unavailable.")

            return self.generate_answer(query, context_chunks, mode="global")
        except Exception as e:
            print(f"Error in global QA: {e}")
            # Even if there's an error in the overall flow, try to return some response
            # using our fallback system
            try:
                # Build a minimal context string
                context_str = "\n\n".join([
                    f"[{chunk['chapter']}]\n{chunk['text']}"
                    for chunk in context_chunks
                ]) if 'context_chunks' in locals() else ""

                # Use the fallback method to generate an answer
                answer = self._try_providers_in_order(f"Question: {query}", context_str, query)

                return {
                    "answer": answer,
                    "citations": [],
                    "sources": [],
                    "citation_validation": {
                        "total_citations": 0,
                        "valid_citations": [],
                        "invalid_citations": [],
                        "has_valid_citations": False,
                        "all_citations_valid": True,
                        "validation_details": [{"citation": "None", "valid": False, "reason": "Error occurred during processing but fallback response generated"}]
                    }
                }
            except Exception as fallback_error:
                print(f"Fallback also failed: {fallback_error}")
                # As a last resort, return a helpful message
                return {
                    "answer": f"Question: {query}\n\nAnswer: All available providers failed to generate a response. Please check your API keys and connectivity. The system is running but cannot generate answers at this time.",
                    "citations": [],
                    "sources": [],
                    "citation_validation": {
                        "total_citations": 0,
                        "valid_citations": [],
                        "invalid_citations": [],
                        "has_valid_citations": False,
                        "all_citations_valid": True,
                        "validation_details": [{"citation": "None", "valid": False, "reason": "All fallback methods failed"}]
                    }
                }

    def query_selected(self, query: str, selected_text: str) -> Dict:
        """Process selected-text QA query

        Args:
            query: User question
            selected_text: Highlighted text from the book

        Returns:
            Response dict with answer and citations
        """
        try:
            # For selected text mode, we use the selected text as context
            # and still retrieve similar chunks to provide additional context
            context_chunks = self.retrieve_context(selected_text, limit=3)

            # Add warning if no additional context was retrieved due to Qdrant issues
            if not context_chunks or len(context_chunks) == 1:  # Only the selected text was added
                print("Warning: Limited context retrieved. Qdrant database may be unavailable.")

            # Add the selected text as the primary context
            context_chunks.insert(0, {
                "text": selected_text,
                "chapter": "Selected Text",
                "score": 1.0
            })

            return self.generate_answer(query, context_chunks, mode="selected")
        except Exception as e:
            print(f"Error in selected-text QA: {e}")
            # Return error message directly without adding "Context:" prefix to avoid recursive nesting
            return {
                "answer": "Sorry, I encountered an error processing your question. Please make sure the backend server is running.",
                "citations": [],
                "sources": [],
                "citation_validation": {
                    "total_citations": 0,
                    "valid_citations": [],
                    "invalid_citations": [],
                    "has_valid_citations": False,
                    "all_citations_valid": True,
                    "validation_details": [{"citation": "None", "valid": False, "reason": "Error occurred during processing"}]
                }
            }

    def _extract_citations(self, text: str) -> List[str]:
        """Extract citation references from generated text

        Args:
            text: Generated answer text

        Returns:
            List of citation strings
        """
        import re
        # Match patterns like [Chapter X: Paragraph Y] as specified in FR-014
        pattern = r'\[Chapter\s+\d+:\s*Paragraph\s+\d+\]'
        citations = re.findall(pattern, text)

        # Also capture any other citation formats that might be present
        all_matches = re.findall(r'\[([^\]]+)\]', text)
        # Filter to only include citations that match the required format
        formatted_citations = []
        for match in all_matches:
            full_match = f'[{match}]'
            if re.match(r'Chapter\s+\d+:\s*Paragraph\s+\d+', match, re.IGNORECASE):
                formatted_citations.append(full_match)

        # If no properly formatted citations found, return all matches as fallback
        if not formatted_citations:
            formatted_citations = [f'[{c}]' for c in all_matches]

        return list(set(formatted_citations))  # Remove duplicates

    def validate_citations(self, citations: List[str], context_chunks: List[Dict]) -> Dict[str, Any]:
        """
        Validate citations to ensure they correspond to actual content in context chunks.
        Implements citation validation when no clear citations exist (Edge Case #107).

        Args:
            citations: List of extracted citations
            context_chunks: List of context chunks that were used

        Returns:
            Dictionary with validation results
        """
        import re

        validation_results = {
            "total_citations": len(citations),
            "valid_citations": [],
            "invalid_citations": [],
            "has_valid_citations": len(citations) > 0,
            "all_citations_valid": True,
            "validation_details": []
        }

        for citation in citations:
            # Extract chapter and paragraph info from citation
            match = re.search(r'Chapter\s+(\d+)', citation)
            if match:
                chapter_num = match.group(1)
                # Check if this chapter exists in our context
                chapter_exists = any(
                    chunk.get('chapter', '').lower().startswith(f'chapter {chapter_num}') or
                    str(chapter_num) in chunk.get('chapter', '')
                    for chunk in context_chunks
                )

                if chapter_exists:
                    validation_results["valid_citations"].append(citation)
                    validation_results["validation_details"].append({
                        "citation": citation,
                        "valid": True,
                        "reason": f"Chapter {chapter_num} found in context"
                    })
                else:
                    validation_results["invalid_citations"].append(citation)
                    validation_results["all_citations_valid"] = False
                    validation_results["validation_details"].append({
                        "citation": citation,
                        "valid": False,
                        "reason": f"Chapter {chapter_num} not found in provided context"
                    })
            else:
                # If citation doesn't match expected format, mark as invalid
                validation_results["invalid_citations"].append(citation)
                validation_results["all_citations_valid"] = False
                validation_results["validation_details"].append({
                    "citation": citation,
                    "valid": False,
                    "reason": "Citation format does not match expected pattern"
                })

        # Special handling for Edge Case #107: when no clear citations exist
        if len(citations) == 0:
            validation_results["has_valid_citations"] = False
            validation_results["validation_details"].append({
                "citation": "None",
                "valid": False,
                "reason": "No citations were generated in the response"
            })
            # This is particularly important for selected-text mode where context is limited
            print("Warning: No citations generated. This may indicate limited source material or LLM not following citation format requirements.")

        return validation_results
