"""
Performance monitoring middleware for tracking response times
"""
import time
import logging
from typing import Callable, Awaitable
from fastapi import Request, Response
from fastapi.responses import StreamingResponse
import asyncio

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global variables for performance tracking
request_times = []
slow_requests = []  # Track requests that exceed the 2-second threshold
MAX_RESPONSE_TIME = 2.0  # 2 seconds as per requirement


class PerformanceMiddleware:
    """
    Middleware to monitor response times and track performance metrics
    Implements SC-002: Verify backend endpoints respond within 2 seconds (p95 latency)
    """

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        request = Request(scope)
        start_time = time.time()

        # Capture the original send function to intercept the response
        async def send_wrapper(message):
            if message["type"] == "http.response.start":
                # Calculate response time when headers are sent
                response_time = time.time() - start_time

                # Log response time
                logger.info(f"Request: {request.method} {request.url.path} | "
                           f"Response Time: {response_time:.3f}s | "
                           f"Status: {message.get('status', 'unknown')}")

                # Track performance metrics
                request_times.append(response_time)

                # Check if response time exceeds threshold
                if response_time > MAX_RESPONSE_TIME:
                    slow_requests.append({
                        "method": request.method,
                        "path": request.url.path,
                        "response_time": response_time,
                        "timestamp": time.time()
                    })
                    logger.warning(f"SLOW REQUEST: {request.method} {request.url.path} "
                                  f"took {response_time:.3f}s (threshold: {MAX_RESPONSE_TIME}s)")

                # Maintain performance statistics
                if len(request_times) > 1000:  # Keep only last 1000 measurements
                    request_times.pop(0)
                if len(slow_requests) > 100:  # Keep only last 100 slow requests
                    slow_requests.pop(0)

            await send(message)

        await self.app(scope, receive, send_wrapper)


def get_performance_stats():
    """
    Get current performance statistics
    """
    if not request_times:
        return {
            "total_requests": 0,
            "avg_response_time": 0,
            "p95_response_time": 0,
            "p99_response_time": 0,
            "max_response_time": 0,
            "slow_requests_count": len(slow_requests),
            "slow_request_percentage": 0
        }

    sorted_times = sorted(request_times)
    n = len(sorted_times)

    # Calculate percentiles
    p95_index = int(0.95 * n) - 1 if n > 0 else 0
    p99_index = int(0.99 * n) - 1 if n > 0 else 0

    p95_time = sorted_times[min(p95_index, n-1)] if n > 0 else 0
    p99_time = sorted_times[min(p99_index, n-1)] if n > 0 else 0

    avg_response_time = sum(request_times) / n if n > 0 else 0
    max_response_time = max(request_times) if request_times else 0
    slow_request_percentage = (len(slow_requests) / n * 100) if n > 0 else 0

    return {
        "total_requests": n,
        "avg_response_time": round(avg_response_time, 3),
        "p95_response_time": round(p95_time, 3),
        "p99_response_time": round(p99_time, 3),
        "max_response_time": round(max_response_time, 3),
        "slow_requests_count": len(slow_requests),
        "slow_request_percentage": round(slow_request_percentage, 2),
        "threshold_exceeded": p95_time > MAX_RESPONSE_TIME
    }


def is_within_performance_threshold():
    """
    Check if the system is meeting the 2-second response time requirement
    """
    stats = get_performance_stats()
    return not stats.get("threshold_exceeded", False)


def get_slow_requests():
    """
    Get list of slow requests (exceeding 2-second threshold)
    """
    return slow_requests


# Optional: Add decorator for specific endpoint performance monitoring
def track_performance(func: Callable) -> Callable:
    """
    Decorator to track performance of specific functions
    """
    async def wrapper(*args, **kwargs):
        start_time = time.time()
        try:
            result = await func(*args, **kwargs)
            response_time = time.time() - start_time
            logger.info(f"Function {func.__name__} took {response_time:.3f}s")
            return result
        except Exception as e:
            response_time = time.time() - start_time
            logger.error(f"Function {func.__name__} failed after {response_time:.3f}s: {str(e)}")
            raise
    return wrapper