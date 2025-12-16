"""
Input sanitization middleware
"""
import re
from typing import Dict, Any
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from ..logging import get_logger

logger = get_logger(__name__)


class InputSanitizationMiddleware:
    def __init__(self):
        # Define patterns for potentially dangerous content
        self.dangerous_patterns = [
            r'<script[^>]*>.*?</script>',  # Script tags
            r'javascript:',               # JavaScript protocol
            r'vbscript:',                # VBScript protocol
            r'on\w+\s*=',                # Event handlers
            r'<iframe[^>]*>.*?</iframe>', # Iframe tags
            r'<object[^>]*>.*?</object>', # Object tags
            r'<embed[^>]*>.*?</embed>',   # Embed tags
            r'<form[^>]*>.*?</form>',     # Form tags
            r'<link[^>]*/>',              # Link tags
            r'<meta[^>]*/>',              # Meta tags
        ]

        # Compile regex patterns for better performance
        self.compiled_patterns = [re.compile(pattern, re.IGNORECASE) for pattern in self.dangerous_patterns]

    def sanitize_text(self, text: str) -> str:
        """Sanitize text input by removing potentially dangerous content"""
        if not text:
            return text

        # Remove potentially dangerous patterns
        sanitized = text
        for pattern in self.compiled_patterns:
            sanitized = pattern.sub('', sanitized)

        # Remove control characters except common whitespace
        sanitized = ''.join(char for char in sanitized if ord(char) >= 32 or char in '\n\r\t')

        return sanitized.strip()

    def validate_text_length(self, text: str, min_length: int = 1, max_length: int = 2000) -> bool:
        """Validate text length is within acceptable bounds"""
        if len(text) < min_length or len(text) > max_length:
            return False
        return True

    def sanitize_query_request(self, query_data: Dict[str, Any]) -> Dict[str, Any]:
        """Sanitize a query request"""
        sanitized_data = query_data.copy()

        # Sanitize the main query
        if 'query' in sanitized_data and sanitized_data['query']:
            sanitized_data['query'] = self.sanitize_text(sanitized_data['query'])
            if not self.validate_text_length(sanitized_data['query'], 1, 2000):
                raise ValueError("Query length must be between 1 and 2000 characters")

        # Sanitize selected text if present
        if 'selected_text' in sanitized_data and sanitized_data['selected_text']:
            sanitized_data['selected_text'] = self.sanitize_text(sanitized_data['selected_text'])
            if not self.validate_text_length(sanitized_data['selected_text'], 1, 5000):
                raise ValueError("Selected text length must be between 1 and 5000 characters")

        # Validate and sanitize session_id if present
        if 'session_id' in sanitized_data and sanitized_data['session_id']:
            # Basic UUID format validation
            session_id = sanitized_data['session_id']
            uuid_pattern = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$', re.IGNORECASE)
            if not uuid_pattern.match(session_id):
                raise ValueError("Invalid session ID format")

        return sanitized_data

    async def __call__(self, request: Request, call_next):
        """Middleware to sanitize incoming requests"""
        # Only process POST requests that might contain query data
        if request.method in ["POST"] and request.url.path.endswith('/query'):
            # Read the request body
            body = await request.body()
            if body:
                try:
                    # Parse the JSON body
                    import json
                    body_data = json.loads(body.decode('utf-8'))

                    # Sanitize the data
                    sanitized_data = self.sanitize_query_request(body_data)

                    # Replace the request body with sanitized data
                    new_body = json.dumps(sanitized_data).encode('utf-8')
                    request._body = new_body

                    # Update content length header
                    request.headers.__dict__['_list'] = [
                        (b'content-length', str(len(new_body)).encode())
                        if h[0].lower() == b'content-length'
                        else h
                        for h in request.headers.__dict__['_list']
                    ]

                except json.JSONDecodeError:
                    logger.warning("Invalid JSON in request body")
                    return JSONResponse(
                        status_code=400,
                        content={"error": "Invalid JSON in request body"}
                    )
                except ValueError as e:
                    logger.warning(f"Input validation error: {str(e)}")
                    return JSONResponse(
                        status_code=400,
                        content={"error": f"Invalid input: {str(e)}"}
                    )
                except Exception as e:
                    logger.error(f"Unexpected error during sanitization: {str(e)}")
                    return JSONResponse(
                        status_code=500,
                        content={"error": "Internal server error during sanitization"}
                    )

        response = await call_next(request)
        return response

    async def get_body(self, request: Request):
        """Helper to get request body"""
        body = await request.body()
        return body


# Helper function to sanitize text input
def sanitize_input(text: str) -> str:
    """Convenience function to sanitize text input"""
    middleware = InputSanitizationMiddleware()
    return middleware.sanitize_text(text)