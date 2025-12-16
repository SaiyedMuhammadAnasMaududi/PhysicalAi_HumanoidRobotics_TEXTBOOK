"""
LLM service abstraction supporting Gemini/Litellm
"""
import os
from typing import List, Dict, Any, Optional
from abc import ABC, abstractmethod
import google.generativeai as genai
import litellm
from .config import settings


class LLMServiceInterface(ABC):
    @abstractmethod
    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response based on the prompt and optional context"""
        pass

    @abstractmethod
    async def embed_text(self, text: str) -> List[float]:
        """Generate embeddings for the given text"""
        pass


class GeminiService(LLMServiceInterface):
    def __init__(self):
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-pro')  # Could be configurable

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response using Gemini"""
        try:
            # Combine context and prompt if context is provided
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"

            response = await self.model.generate_content_async(full_prompt)
            return response.text
        except Exception as e:
            raise Exception(f"Error generating response with Gemini: {str(e)}")

    async def embed_text(self, text: str) -> List[float]:
        """Generate embeddings using Gemini"""
        try:
            result = await genai.embed_content_async(
                model="models/embedding-001",
                content=[text],
                task_type="retrieval_document"
            )
            return result['embedding'][0]  # Return the first embedding
        except Exception as e:
            raise Exception(f"Error generating embedding with Gemini: {str(e)}")


class LitellmService(LLMServiceInterface):
    def __init__(self, model_name: str = "gpt-3.5-turbo"):
        self.model_name = model_name
        # Litellm will use environment variables automatically (OPENAI_API_KEY, etc.)

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response using Litellm (which abstracts multiple providers)"""
        try:
            # Combine context and prompt if context is provided
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"

            response = await litellm.acompletion(
                model=self.model_name,
                messages=[{"role": "user", "content": full_prompt}],
                temperature=0.7
            )
            return response.choices[0].message.content
        except Exception as e:
            raise Exception(f"Error generating response with Litellm: {str(e)}")

    async def embed_text(self, text: str) -> List[float]:
        """Generate embeddings using Litellm"""
        try:
            response = await litellm.aembedding(
                model="text-embedding-ada-002",  # Default embedding model
                input=[text]
            )
            return response.data[0]['embedding']  # Return the first embedding
        except Exception as e:
            raise Exception(f"Error generating embedding with Litellm: {str(e)}")


class LLMService:
    def __init__(self):
        model_provider = os.getenv("MODEL_PROVIDER", "gemini").lower()

        if model_provider == "gemini":
            self.service = GeminiService()
        elif model_provider == "litellm":
            model_name = os.getenv("LITELM_MODEL", "gpt-3.5-turbo")
            self.service = LitellmService(model_name)
        else:
            # Default to Gemini
            self.service = GeminiService()

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response using the configured LLM service"""
        return await self.service.generate_response(prompt, context)

    async def embed_text(self, text: str) -> List[float]:
        """Generate embeddings using the configured LLM service"""
        return await self.service.embed_text(text)

    async def embed_query(self, query: str) -> List[float]:
        """Generate embeddings for a query (specialized for search)"""
        return await self.service.embed_text(query)