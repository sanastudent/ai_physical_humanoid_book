"""
Status reporting utilities for health checks

Provides functions for creating standardized status reports and error messages
"""
from typing import Optional, Dict, Any
from ..schema import ComponentStatus, HealthStatus, HealthCheckResult


def create_component_status(
    name: str,
    status: HealthStatus,
    response_time_ms: Optional[int] = None,
    message: Optional[str] = None,
    error: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None
) -> ComponentStatus:
    """
    Create a standardized component status report

    Args:
        name: Component name (backend, qdrant, embeddings, config)
        status: Health status (healthy, degraded, unhealthy)
        response_time_ms: Response time in milliseconds
        message: Human-readable status message
        error: Error message if unhealthy
        metadata: Component-specific metadata

    Returns:
        ComponentStatus object
    """
    return ComponentStatus(
        name=name,
        status=status,
        response_time_ms=response_time_ms,
        message=message,
        error=error,
        metadata=metadata or {}
    )


def create_health_check_result(
    components: list[ComponentStatus],
    version: str = "1.0.0"
) -> HealthCheckResult:
    """
    Create overall health check result from component statuses

    Automatically determines overall status based on component statuses:
    - If any component is UNHEALTHY: overall is UNHEALTHY
    - If any component is DEGRADED: overall is DEGRADED
    - If all components are HEALTHY: overall is HEALTHY

    Args:
        components: List of component statuses
        version: API version

    Returns:
        HealthCheckResult object
    """
    # Determine overall status
    if any(c.status == HealthStatus.UNHEALTHY for c in components):
        overall_status = HealthStatus.UNHEALTHY
    elif any(c.status == HealthStatus.DEGRADED for c in components):
        overall_status = HealthStatus.DEGRADED
    else:
        overall_status = HealthStatus.HEALTHY

    # Calculate total response time
    total_response_time = sum(
        c.response_time_ms for c in components if c.response_time_ms is not None
    )

    # Collect all errors
    errors = [c.error for c in components if c.error is not None]

    return HealthCheckResult(
        status=overall_status,
        version=version,
        components=components,
        total_response_time_ms=total_response_time if total_response_time > 0 else None,
        errors=errors
    )


def format_error_message(error: Exception, component: str) -> str:
    """
    Format error message with component context

    Args:
        error: Exception that occurred
        component: Component name where error occurred

    Returns:
        Formatted error message
    """
    error_type = type(error).__name__
    error_msg = str(error)
    return f"{component} error: {error_type}: {error_msg}"


def get_remediation_hint(error: Exception, component: str) -> Optional[str]:
    """
    Get remediation hint based on error type and component

    Args:
        error: Exception that occurred
        component: Component where error occurred

    Returns:
        Remediation hint or None
    """
    error_msg = str(error).lower()

    # Qdrant-specific hints
    if component == "qdrant":
        if "connection refused" in error_msg:
            return "Verify Qdrant is running and accessible. Check QDRANT_HOST and QDRANT_PORT environment variables."
        elif "authentication" in error_msg or "unauthorized" in error_msg:
            return "Check QDRANT_API_KEY is set correctly for cloud instances."
        elif "collection not found" in error_msg:
            return "Ensure the book_embeddings collection exists in Qdrant."

    # Embeddings-specific hints
    elif component == "embeddings":
        if "api key" in error_msg or "authentication" in error_msg:
            return "Verify ANTHROPIC_API_KEY, OPENAI_API_KEY, or GOOGLE_API_KEY is set correctly."
        elif "rate limit" in error_msg:
            return "API rate limit exceeded. Wait and retry, or reduce test frequency."

    # Configuration-specific hints
    elif component == "config":
        if "missing" in error_msg:
            return "Set missing environment variables in .env file or environment."
        elif "invalid" in error_msg:
            return "Check environment variable values are within valid ranges."

    return None
