"""
Performance monitoring for response time ≤ 3 seconds
"""
from typing import Dict, List, Optional, Callable, Awaitable
from datetime import datetime, timedelta
import time
import asyncio
import logging
from dataclasses import dataclass
from .services.performance_service import PerformanceService, PerformanceMetrics


@dataclass
class MonitoringConfig:
    """Configuration for monitoring system"""
    response_time_threshold_ms: float = 3000.0  # 3 seconds
    alert_on_threshold_exceeded: bool = True
    log_performance_metrics: bool = True
    performance_check_interval: int = 60  # seconds
    max_metrics_to_store: int = 1000


class MonitoringService:
    """Service for monitoring performance metrics and ensuring response time ≤ 3 seconds"""

    def __init__(self, config: Optional[MonitoringConfig] = None):
        self.config = config or MonitoringConfig()
        self.performance_service = PerformanceService()
        self.logger = logging.getLogger(__name__)
        self.metrics_history: List[PerformanceMetrics] = []
        self.alert_callbacks: List[Callable] = []

    def add_alert_callback(self, callback: Callable):
        """Add a callback function to be called when performance issues are detected"""
        self.alert_callbacks.append(callback)

    def track_request(
        self,
        query_id: str,
        start_time: float,
        retrieval_time: float = 0,
        llm_processing_time: float = 0,
        tokens_used: Optional[int] = None
    ) -> PerformanceMetrics:
        """Track a request and check if it meets performance requirements"""
        metrics = self.performance_service.track_response_time(
            query_id,
            start_time,
            retrieval_time,
            llm_processing_time,
            tokens_used
        )

        # Add to history
        self.metrics_history.append(metrics)
        if len(self.metrics_history) > self.config.max_metrics_to_store:
            self.metrics_history.pop(0)  # Remove oldest metrics

        # Check if response time exceeds threshold
        if metrics.total_time_ms > self.config.response_time_threshold_ms:
            self._handle_threshold_exceeded(metrics)

        return metrics

    def _handle_threshold_exceeded(self, metrics: PerformanceMetrics):
        """Handle when a request exceeds the response time threshold"""
        if self.config.alert_on_threshold_exceeded:
            alert_msg = (
                f"Performance alert: Query {metrics.query_id} took "
                f"{metrics.total_time_ms:.2f}ms (threshold: {self.config.response_time_threshold_ms}ms)"
            )
            self.logger.warning(alert_msg)

            # Call alert callbacks
            for callback in self.alert_callbacks:
                try:
                    callback(metrics)
                except Exception as e:
                    self.logger.error(f"Error in alert callback: {e}")

    async def monitor_async_operation(
        self,
        operation: Callable[[], Awaitable],
        operation_name: str = "operation",
        query_id: Optional[str] = None
    ) -> PerformanceMetrics:
        """Monitor an async operation for performance"""
        start_time = time.time()
        try:
            result = await operation()
            duration_ms = (time.time() - start_time) * 1000

            # Create a temporary metrics object for this operation
            temp_metrics = PerformanceMetrics(
                query_id=query_id or "unknown",
                response_time_ms=duration_ms,
                retrieval_time_ms=0,
                llm_processing_time_ms=duration_ms,
                total_time_ms=duration_ms
            )

            # Log the performance if needed
            if self.config.log_performance_metrics:
                self.logger.info(f"{operation_name} completed in {duration_ms:.2f}ms")

            return temp_metrics
        except Exception as e:
            duration_ms = (time.time() - start_time) * 1000
            self.logger.error(f"{operation_name} failed after {duration_ms:.2f}ms: {str(e)}")
            raise

    def get_performance_summary(self) -> Dict[str, float]:
        """Get a summary of performance metrics"""
        if not self.metrics_history:
            return {
                "average_response_time_ms": 0.0,
                "max_response_time_ms": 0.0,
                "min_response_time_ms": 0.0,
                "threshold_exceeded_count": 0,
                "compliance_percentage": 100.0,
                "total_requests": 0
            }

        response_times = [m.total_time_ms for m in self.metrics_history]
        threshold_exceeded_count = sum(1 for t in response_times if t > self.config.response_time_threshold_ms)

        return {
            "average_response_time_ms": sum(response_times) / len(response_times),
            "max_response_time_ms": max(response_times),
            "min_response_time_ms": min(response_times),
            "threshold_exceeded_count": threshold_exceeded_count,
            "compliance_percentage": (
                (len(response_times) - threshold_exceeded_count) / len(response_times) * 100
                if response_times else 100.0
            ),
            "total_requests": len(response_times)
        }

    def is_currently_performing_well(self, window_minutes: int = 5) -> bool:
        """Check if performance is acceptable in the recent time window"""
        if not self.metrics_history:
            return True

        # Get metrics from the last N minutes
        time_threshold = datetime.now().timestamp() - (window_minutes * 60)
        recent_metrics = [
            m for m in self.metrics_history
            if (datetime.now().timestamp() - (m.total_time_ms / 1000)) > time_threshold
        ]

        if not recent_metrics:
            return True

        # Calculate compliance in recent window
        threshold_exceeded_count = sum(
            1 for m in recent_metrics
            if m.total_time_ms > self.config.response_time_threshold_ms
        )

        compliance_rate = (
            (len(recent_metrics) - threshold_exceeded_count) / len(recent_metrics)
            if recent_metrics else 1.0
        )

        # Consider performing well if at least 95% of requests are under threshold
        return compliance_rate >= 0.95

    def get_sla_compliance(self) -> Dict[str, float]:
        """Get SLA compliance metrics"""
        summary = self.get_performance_summary()

        # Calculate 95th percentile response time
        if self.metrics_history:
            sorted_times = sorted(m.total_time_ms for m in self.metrics_history)
            percentile_95_idx = int(0.95 * len(sorted_times))
            p95_response_time = sorted_times[min(percentile_95_idx, len(sorted_times) - 1)]
        else:
            p95_response_time = 0.0

        return {
            "compliance_percentage": summary["compliance_percentage"],
            "p95_response_time_ms": p95_response_time,
            "average_response_time_ms": summary["average_response_time_ms"],
            "threshold_ms": self.config.response_time_threshold_ms,
            "is_sla_met": summary["compliance_percentage"] >= 95.0  # SLA: 95% of requests under threshold
        }

    async def start_continuous_monitoring(self):
        """Start continuous monitoring in the background"""
        async def monitoring_loop():
            while True:
                try:
                    await asyncio.sleep(self.config.performance_check_interval)
                    summary = self.get_performance_summary()
                    self.logger.info(f"Performance summary: {summary}")
                except Exception as e:
                    self.logger.error(f"Error in monitoring loop: {e}")

        # Run the monitoring loop in the background
        asyncio.create_task(monitoring_loop())

    def validate_response_time_requirement(self, response_time_ms: float) -> bool:
        """Validate that response time meets the ≤ 3 seconds requirement"""
        return response_time_ms <= self.config.response_time_threshold_ms

    def get_performance_recommendations(self) -> List[str]:
        """Get recommendations for improving performance if needed"""
        recommendations = []
        summary = self.get_performance_summary()

        if summary["average_response_time_ms"] > self.config.response_time_threshold_ms * 0.8:
            recommendations.append(
                "Average response time is approaching the 3-second threshold. "
                "Consider optimizing database queries or caching frequently accessed content."
            )

        if summary["max_response_time_ms"] > self.config.response_time_threshold_ms:
            recommendations.append(
                "Some requests are exceeding the 3-second threshold. "
                "Investigate slow queries or consider implementing request queuing."
            )

        if summary["compliance_percentage"] < 95:
            recommendations.append(
                f"Only {summary['compliance_percentage']:.1f}% of requests are meeting "
                "the 3-second requirement. This is below the 95% SLA target."
            )

        return recommendations


# Global monitoring service instance
monitoring_service = MonitoringService()