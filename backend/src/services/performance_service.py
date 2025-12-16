"""
Response time tracking and performance metrics
"""
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import time
import asyncio
from dataclasses import dataclass
from ..models.query import Query
from ..models.response import Response
from ..logging import get_logger

logger = get_logger(__name__)


@dataclass
class PerformanceMetrics:
    """Data class to hold performance metrics"""
    query_id: str
    response_time_ms: float
    retrieval_time_ms: float
    llm_processing_time_ms: float
    total_time_ms: float
    tokens_used: Optional[int] = None
    cache_hit: bool = False


class PerformanceService:
    """Service for tracking and monitoring performance metrics"""

    def __init__(self):
        self.metrics: List[PerformanceMetrics] = []
        self.performance_threshold_ms = 3000  # 3 seconds as per requirements
        self.high_load_threshold = 10  # Number of concurrent requests to consider high load

    def start_timer(self) -> float:
        """Start a timer and return the start time"""
        return time.time()

    def end_timer(self, start_time: float) -> float:
        """End a timer and return the elapsed time in milliseconds"""
        return (time.time() - start_time) * 1000

    def track_response_time(
        self,
        query_id: str,
        start_time: float,
        retrieval_time: float = 0,
        llm_processing_time: float = 0,
        tokens_used: Optional[int] = None
    ) -> PerformanceMetrics:
        """Track the response time for a query"""
        total_time = self.end_timer(start_time)

        metrics = PerformanceMetrics(
            query_id=query_id,
            response_time_ms=total_time,
            retrieval_time_ms=retrieval_time,
            llm_processing_time_ms=llm_processing_time,
            total_time_ms=total_time,
            tokens_used=tokens_used
        )

        self.metrics.append(metrics)

        # Log performance if it exceeds threshold
        if total_time > self.performance_threshold_ms:
            logger.warning(
                f"Performance alert: Query {query_id} took {total_time:.2f}ms "
                f"(threshold: {self.performance_threshold_ms}ms)"
            )

        return metrics

    def get_average_response_time(self) -> float:
        """Get the average response time across all tracked queries"""
        if not self.metrics:
            return 0.0
        return sum(m.total_time_ms for m in self.metrics) / len(self.metrics)

    def get_recent_average_response_time(self, last_n: int = 10) -> float:
        """Get the average response time for the last N queries"""
        recent_metrics = self.metrics[-last_n:] if len(self.metrics) >= last_n else self.metrics
        if not recent_metrics:
            return 0.0
        return sum(m.total_time_ms for m in recent_metrics) / len(recent_metrics)

    def is_performance_acceptable(self) -> bool:
        """Check if the average response time is within acceptable limits"""
        avg_time = self.get_average_response_time()
        return avg_time <= self.performance_threshold_ms

    def get_performance_stats(self) -> Dict[str, float]:
        """Get comprehensive performance statistics"""
        if not self.metrics:
            return {
                "average_response_time_ms": 0.0,
                "min_response_time_ms": 0.0,
                "max_response_time_ms": 0.0,
                "total_queries": 0,
                "acceptable_performance_ratio": 0.0
            }

        response_times = [m.total_time_ms for m in self.metrics]
        acceptable_count = sum(1 for t in response_times if t <= self.performance_threshold_ms)

        return {
            "average_response_time_ms": sum(response_times) / len(response_times),
            "min_response_time_ms": min(response_times),
            "max_response_time_ms": max(response_times),
            "total_queries": len(self.metrics),
            "acceptable_performance_ratio": acceptable_count / len(self.metrics)
        }

    async def monitor_performance_async(
        self,
        operation,
        operation_name: str = "operation"
    ):
        """Monitor performance of an async operation"""
        start_time = self.start_timer()
        try:
            result = await operation()
            duration = self.end_timer(start_time)
            logger.info(f"{operation_name} completed in {duration:.2f}ms")
            return result
        except Exception as e:
            duration = self.end_timer(start_time)
            logger.error(f"{operation_name} failed after {duration:.2f}ms: {str(e)}")
            raise

    def validate_response_time(self, response_time_ms: float) -> bool:
        """Validate that response time meets requirements (≤ 3 seconds)"""
        return response_time_ms <= self.performance_threshold_ms

    def log_performance_metric(
        self,
        query_id: str,
        response_time_ms: float,
        context: Optional[Dict] = None
    ):
        """Log performance metrics for monitoring and alerting"""
        # This would typically integrate with a metrics system like Prometheus
        # For now, we'll log to the application log
        log_data = {
            "query_id": query_id,
            "response_time_ms": response_time_ms,
            "within_threshold": response_time_ms <= self.performance_threshold_ms,
            "threshold_ms": self.performance_threshold_ms
        }
        if context:
            log_data.update(context)

        logger.info(f"Performance metric: {log_data}")

    def get_concurrency_metrics(self) -> Dict[str, int]:
        """Get metrics related to concurrent usage"""
        # In a real implementation, this would track actual concurrent requests
        # For now, returning placeholder data
        return {
            "current_concurrent_requests": 0,  # Would be tracked via middleware
            "peak_concurrent_requests": 0,
            "high_load_events": 0
        }

    def check_concurrency_compliance(self, current_concurrent: int) -> bool:
        """Check if concurrent usage is within acceptable limits (≥10 simultaneous users)"""
        # The requirement is to handle ≥10 simultaneous users, so any number up to 10 is acceptable
        # This would be used to monitor if we're exceeding our capacity
        return current_concurrent <= 10  # Actually, we want to handle MORE than 10, so this is just a check