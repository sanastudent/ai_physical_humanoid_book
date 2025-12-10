"""
Validation and error handling utilities for health checks

Provides timeout management, error handling, and configuration validation
"""
import asyncio
from typing import Callable, TypeVar, Any, Optional
from functools import wraps
import logging

from .config import settings

logger = logging.getLogger(__name__)

T = TypeVar('T')


async def with_timeout(
    coro: Callable[..., T],
    timeout_seconds: Optional[int] = None,
    component_name: str = "component"
) -> T:
    """
    Execute async function with timeout

    Args:
        coro: Async function to execute
        timeout_seconds: Timeout in seconds (uses component_timeout from settings if None)
        component_name: Name of component for error messages

    Returns:
        Result of the async function

    Raises:
        asyncio.TimeoutError: If function exceeds timeout
    """
    timeout = timeout_seconds or settings.component_timeout

    try:
        return await asyncio.wait_for(coro, timeout=timeout)
    except asyncio.TimeoutError:
        logger.warning(f"{component_name} health check timed out after {timeout}s")
        raise asyncio.TimeoutError(f"{component_name} health check exceeded {timeout}s timeout")


def handle_health_check_error(component: str):
    """
    Decorator for handling health check errors gracefully

    Args:
        component: Component name for error reporting

    Returns:
        Decorated function that catches and logs errors
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                return await func(*args, **kwargs)
            except asyncio.TimeoutError as e:
                logger.error(f"{component} health check timed out: {e}")
                raise
            except Exception as e:
                logger.error(f"{component} health check failed: {type(e).__name__}: {e}")
                raise
        return wrapper
    return decorator


def validate_environment_variable(
    var_name: str,
    required: bool = True,
    validator: Optional[Callable[[str], bool]] = None
) -> tuple[bool, Optional[str]]:
    """
    Validate environment variable

    Args:
        var_name: Environment variable name
        required: Whether variable is required
        validator: Optional validation function

    Returns:
        Tuple of (is_valid, error_message)
    """
    import os

    value = os.getenv(var_name)

    # Check if required variable is present
    if required and not value:
        return False, f"Required environment variable {var_name} is not set"

    # If optional and not set, that's okay
    if not required and not value:
        return True, None

    # Run custom validator if provided
    if validator and value:
        try:
            if not validator(value):
                return False, f"Environment variable {var_name} has invalid value"
        except Exception as e:
            return False, f"Validation error for {var_name}: {str(e)}"

    return True, None


async def retry_with_backoff(
    func: Callable[..., T],
    max_retries: int = 3,
    initial_delay: float = 1.0,
    backoff_factor: float = 2.0,
    component_name: str = "component"
) -> T:
    """
    Retry async function with exponential backoff

    Args:
        func: Async function to retry
        max_retries: Maximum number of retry attempts
        initial_delay: Initial delay in seconds
        backoff_factor: Multiply delay by this factor after each retry
        component_name: Component name for logging

    Returns:
        Result of the function

    Raises:
        Last exception if all retries fail
    """
    delay = initial_delay
    last_exception = None

    for attempt in range(max_retries):
        try:
            return await func()
        except Exception as e:
            last_exception = e
            if attempt < max_retries - 1:
                logger.warning(
                    f"{component_name} check failed (attempt {attempt + 1}/{max_retries}), "
                    f"retrying in {delay}s: {e}"
                )
                await asyncio.sleep(delay)
                delay *= backoff_factor
            else:
                logger.error(f"{component_name} check failed after {max_retries} attempts: {e}")

    raise last_exception


def check_required_settings(settings_obj: Any, required_fields: list[str]) -> tuple[list[str], list[str]]:
    """
    Check if required settings are present

    Args:
        settings_obj: Settings object to check
        required_fields: List of required field names

    Returns:
        Tuple of (missing_fields, warnings)
    """
    missing = []
    warnings = []

    for field in required_fields:
        value = getattr(settings_obj, field, None)
        if value is None or (isinstance(value, str) and not value):
            missing.append(field)

    # Check for at least one embeddings provider
    has_anthropic = getattr(settings_obj, 'anthropic_api_key', None)
    has_openai = getattr(settings_obj, 'openai_api_key', None)
    has_google = getattr(settings_obj, 'google_api_key', None)

    if not any([has_anthropic, has_openai, has_google]):
        missing.append("embeddings_api_key (at least one of: ANTHROPIC_API_KEY, OPENAI_API_KEY, GOOGLE_API_KEY)")

    # Add warnings for optional but recommended settings
    if not has_anthropic:
        warnings.append("ANTHROPIC_API_KEY not set - Anthropic embeddings unavailable")
    if not has_openai:
        warnings.append("OPENAI_API_KEY not set - OpenAI embeddings unavailable")
    if not has_google:
        warnings.append("GOOGLE_API_KEY not set - Google embeddings unavailable")

    return missing, warnings
