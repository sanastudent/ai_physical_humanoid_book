"""
Health check module for backend, Qdrant, and embeddings readiness verification

This module provides comprehensive health checking capabilities for:
- Backend service health
- Qdrant vector database connection
- Embeddings service availability
- Configuration validation
- End-to-end workflow testing
"""

from .config import HealthCheckSettings, settings
from .reporters import (
    create_component_status,
    create_health_check_result,
    format_error_message,
    get_remediation_hint
)
from .validators import (
    with_timeout,
    handle_health_check_error,
    validate_environment_variable,
    retry_with_backoff,
    check_required_settings
)

__all__ = [
    # Configuration
    'HealthCheckSettings',
    'settings',

    # Reporters
    'create_component_status',
    'create_health_check_result',
    'format_error_message',
    'get_remediation_hint',

    # Validators
    'with_timeout',
    'handle_health_check_error',
    'validate_environment_variable',
    'retry_with_backoff',
    'check_required_settings',
]
