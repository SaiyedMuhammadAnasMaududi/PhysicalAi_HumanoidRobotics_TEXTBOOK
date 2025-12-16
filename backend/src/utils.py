"""
Utility functions for UUID generation and timestamp handling
"""
import uuid
from datetime import datetime
from typing import Union
import time


def generate_uuid() -> str:
    """
    Generate a new UUID string
    """
    return str(uuid.uuid4())


def generate_uuid_bytes() -> bytes:
    """
    Generate a new UUID as bytes
    """
    return uuid.uuid4().bytes


def get_current_timestamp() -> datetime:
    """
    Get the current timestamp as a datetime object
    """
    return datetime.utcnow()


def get_current_timestamp_iso() -> str:
    """
    Get the current timestamp as an ISO format string
    """
    return datetime.utcnow().isoformat()


def timestamp_to_iso(timestamp: datetime) -> str:
    """
    Convert a datetime object to ISO format string
    """
    return timestamp.isoformat()


def iso_to_timestamp(iso_string: str) -> datetime:
    """
    Convert an ISO format string to a datetime object
    """
    return datetime.fromisoformat(iso_string.replace('Z', '+00:00'))


def get_elapsed_time_ms(start_time: datetime) -> float:
    """
    Calculate elapsed time in milliseconds between start_time and now
    """
    elapsed = datetime.utcnow() - start_time
    return elapsed.total_seconds() * 1000


def format_duration_ms(milliseconds: Union[int, float]) -> str:
    """
    Format a duration in milliseconds to a human-readable string
    """
    if milliseconds < 1000:
        return f"{milliseconds:.0f}ms"
    else:
        seconds = milliseconds / 1000
        if seconds < 60:
            return f"{seconds:.2f}s"
        else:
            minutes = seconds / 60
            return f"{minutes:.2f}m"


def sanitize_for_logging(text: str) -> str:
    """
    Sanitize text for safe logging (remove potentially sensitive information)
    """
    if not text:
        return text

    # Remove potential API keys or tokens from logs (basic pattern)
    import re
    sanitized = re.sub(r'(?i)(api[_-]?key|token|secret)\s*[:=]\s*["\']?[\w-]+["\']?',
                      r'\1: [REDACTED]', text)
    return sanitized


def hash_ip_address(ip_address: str) -> str:
    """
    Hash an IP address for privacy compliance
    """
    import hashlib
    return hashlib.sha256(ip_address.encode()).hexdigest()


def truncate_text(text: str, max_length: int = 100, suffix: str = "...") -> str:
    """
    Truncate text to a maximum length with an optional suffix
    """
    if len(text) <= max_length:
        return text
    else:
        return text[:max_length - len(suffix)] + suffix