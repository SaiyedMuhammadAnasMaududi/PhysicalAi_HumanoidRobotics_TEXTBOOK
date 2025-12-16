"""
API rate limiting and security middleware
"""
import time
from typing import Dict, Optional
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from collections import defaultdict, deque
from starlette.middleware.base import BaseHTTPMiddleware
import hashlib
from ..utils import hash_ip_address


class RateLimitMiddleware:
    def __init__(
        self,
        requests_per_minute: int = 60,
        requests_per_hour: int = 1000,
        block_duration: int = 3600  # 1 hour
    ):
        self.requests_per_minute = requests_per_minute
        self.requests_per_hour = requests_per_hour
        self.block_duration = block_duration

        # Store request times for each IP
        self.requests: Dict[str, deque] = defaultdict(deque)
        # Store blocked IPs and when they were blocked
        self.blocked_ips: Dict[str, float] = {}

    async def __call__(self, request: Request, call_next):
        # Get client IP
        client_ip = request.client.host
        if request.headers.get("X-Forwarded-For"):
            client_ip = request.headers.get("X-Forwarded-For").split(",")[0].strip()
        elif request.headers.get("X-Real-IP"):
            client_ip = request.headers.get("X-Real-IP")

        # Check if IP is blocked
        if client_ip in self.blocked_ips:
            if time.time() - self.blocked_ips[client_ip] < self.block_duration:
                return JSONResponse(
                    status_code=429,
                    content={"error": "Too many requests", "message": "IP temporarily blocked due to rate limiting"}
                )
            else:
                # Remove from blocked list if block duration has passed
                del self.blocked_ips[client_ip]

        # Get current time
        now = time.time()

        # Clean old requests (older than 1 hour)
        while (self.requests[client_ip] and
               self.requests[client_ip][0] < now - 3600):
            self.requests[client_ip].popleft()

        # Check hourly limit
        if len(self.requests[client_ip]) >= self.requests_per_hour:
            self.blocked_ips[client_ip] = now
            return JSONResponse(
                status_code=429,
                content={"error": "Too many requests", "message": "Hourly limit exceeded. IP blocked for 1 hour."}
            )

        # Clean minute-old requests
        minute_requests = [req_time for req_time in self.requests[client_ip] if req_time >= now - 60]

        # Check minute limit
        if len(minute_requests) >= self.requests_per_minute:
            return JSONResponse(
                status_code=429,
                content={"error": "Too many requests", "message": f"Rate limit exceeded. Maximum {self.requests_per_minute} requests per minute."}
            )

        # Add current request to tracking
        self.requests[client_ip].append(now)

        response = await call_next(request)
        return response


class SecurityHeadersMiddleware:
    """Middleware to add security headers to responses"""
    async def __call__(self, request: Request, call_next):
        response = await call_next(request)

        # Add security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Content-Security-Policy"] = "default-src 'self'"
        response.headers["Referrer-Policy"] = "no-referrer"

        return response


class SecurityMiddleware:
    """Main security middleware that combines rate limiting and other security measures"""
    def __init__(
        self,
        requests_per_minute: int = 60,
        requests_per_hour: int = 1000,
        block_duration: int = 3600
    ):
        self.rate_limit_middleware = RateLimitMiddleware(
            requests_per_minute,
            requests_per_hour,
            block_duration
        )
        self.security_headers_middleware = SecurityHeadersMiddleware()

    async def __call__(self, request: Request, call_next):
        # Apply rate limiting
        client_ip = request.client.host
        if request.headers.get("X-Forwarded-For"):
            client_ip = request.headers.get("X-Forwarded-For").split(",")[0].strip()

        # Check if IP is blocked
        if hasattr(self.rate_limit_middleware, 'blocked_ips'):
            if client_ip in self.rate_limit_middleware.blocked_ips:
                if (time.time() - self.rate_limit_middleware.blocked_ips[client_ip] <
                        self.rate_limit_middleware.block_duration):
                    return JSONResponse(
                        status_code=429,
                        content={"error": "Too many requests", "message": "IP temporarily blocked due to rate limiting"}
                    )
                else:
                    # Remove from blocked list if block duration has passed
                    del self.rate_limit_middleware.blocked_ips[client_ip]

        # Add current request for rate limiting
        now = time.time()
        self.rate_limit_middleware.requests[client_ip].append(now)

        # Continue with request
        response = await call_next(request)

        # Apply security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["Content-Security-Policy"] = "default-src 'self'"
        response.headers["Referrer-Policy"] = "no-referrer"

        return response


# Create a default instance
security_middleware = SecurityMiddleware()