"""
RAG Agent Implementation - Simplified Working Version

This module implements a working AI agent that processes queries with retrieval.
"""
import os
from typing import Dict, Any, List
from dotenv import load_dotenv
from litellm import completion

# Load environment variables
load_dotenv()


class RAGAgent:
    """
    Simplified RAG Agent that works reliably
    """

    def __init__(self):
        """
        Initialize the RAG agent
        """
        # Get configuration from environment
        self.model_name = os.getenv("LITELLM_MODEL", "gemini/gemini-2.0-flash")
        self.api_key = os.getenv("GEMINI_API_KEY") or os.getenv("OPENAI_API_KEY")

        if not self.api_key:
            raise ValueError("Either GEMINI_API_KEY or OPENAI_API_KEY environment variable is required")

        # Set API key for litellm
        os.environ["GEMINI_API_KEY"] = self.api_key

        # Store for source attribution
        self.last_retrieved_context = []

        print(f"RAGAgent initialized with model: {self.model_name}")

    def query(self, query_text: str, user_context: Dict[str, Any] = None) -> str:
        """
        Process a query using retrieval + LLM
        """
        try:
            # Import retrieval tool
            from .retrieval_tool import retrieve_context

            # Retrieve relevant context from the book
            retrieved_context = retrieve_context(query_text, top_k=3)

            # Store the retrieved context for source attribution
            self.last_retrieved_context = retrieved_context

            # Build context string for the LLM
            if not retrieved_context:
                context_str = "No relevant information found in the book."
            else:
                context_str = "Retrieved information from the book:\n\n"
                for i, chunk in enumerate(retrieved_context, 1):
                    context_str += f"Source {i}: {chunk['section_title']} ({chunk['source_file']})\n"
                    context_str += f"Content: {chunk['content'][:400]}...\n\n"

            # Create prompt for the LLM
            messages = [
                {
                    "role": "system",
                    "content": "You are a helpful technical assistant that answers questions about Physical AI and Humanoid Robotics. Base your answers on the provided context from the book. Be concise and cite sources."
                },
                {
                    "role": "user",
                    "content": f"Context from the book:\n{context_str}\n\nQuestion: {query_text}\n\nPlease answer based on the context provided."
                }
            ]

            # Call LiteLLM
            response = completion(
                model=self.model_name,
                messages=messages,
                temperature=0.1
            )

            # Extract the response
            answer = response.choices[0].message.content

            return answer

        except Exception as e:
            # If retrieval or LLM fails, return error
            self.last_retrieved_context = []
            return f"Error processing query: {str(e)}"
