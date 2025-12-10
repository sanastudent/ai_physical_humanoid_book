"""
Health check functions for backend components

Implements individual health checks for backend, Qdrant, and embeddings services
"""
import time
import asyncio
from typing import Optional
import logging

from ..schema import ComponentStatus, HealthStatus, HealthCheckResult
from ..qdrant_manager import QdrantManager
from ..embed import EmbeddingGenerator
from .reporters import create_component_status, create_health_check_result
from .validators import with_timeout, handle_health_check_error

logger = logging.getLogger(__name__)


@handle_health_check_error("backend")
async def check_backend_health() -> ComponentStatus:
    """
    Check backend service health

    Returns:
        ComponentStatus with backend health information
    """
    start_time = time.time()

    # Backend is healthy if we can execute this function
    response_time_ms = int((time.time() - start_time) * 1000)

    return create_component_status(
        name="backend",
        status=HealthStatus.HEALTHY,
        response_time_ms=response_time_ms,
        message="Backend service operational"
    )


@handle_health_check_error("qdrant")
async def check_qdrant_health(qdrant_manager: QdrantManager) -> ComponentStatus:
    """
    Check Qdrant vector database health

    Args:
        qdrant_manager: QdrantManager instance

    Returns:
        ComponentStatus with Qdrant health information
    """
    start_time = time.time()

    try:
        # Check basic connection
        metadata = await with_timeout(
            qdrant_manager.health_check(),
            component_name="qdrant"
        )

        # Verify collection schema
        schema_valid = await with_timeout(
            qdrant_manager.verify_collection_schema(),
            component_name="qdrant"
        )

        response_time_ms = int((time.time() - start_time) * 1000)

        if not schema_valid:
            return create_component_status(
                name="qdrant",
                status=HealthStatus.DEGRADED,
                response_time_ms=response_time_ms,
                message="Qdrant connected but collection schema invalid",
                metadata=metadata
            )

        return create_component_status(
            name="qdrant",
            status=HealthStatus.HEALTHY,
            response_time_ms=response_time_ms,
            message="Qdrant connection successful and schema valid",
            metadata=metadata
        )

    except asyncio.TimeoutError as e:
        response_time_ms = int((time.time() - start_time) * 1000)
        return create_component_status(
            name="qdrant",
            status=HealthStatus.UNHEALTHY,
            response_time_ms=response_time_ms,
            error=str(e),
            message="Qdrant health check timed out"
        )
    except Exception as e:
        response_time_ms = int((time.time() - start_time) * 1000)
        return create_component_status(
            name="qdrant",
            status=HealthStatus.UNHEALTHY,
            response_time_ms=response_time_ms,
            error=str(e),
            message="Qdrant connection failed"
        )


@handle_health_check_error("embeddings")
async def check_embeddings_health(embedding_generator: Optional[EmbeddingGenerator] = None) -> ComponentStatus:
    """
    Check embeddings service health

    Args:
        embedding_generator: Optional EmbeddingGenerator instance

    Returns:
        ComponentStatus with embeddings service health information
    """
    start_time = time.time()

    if embedding_generator is None:
        # Try to initialize a new one for health check
        try:
            embedding_generator = EmbeddingGenerator()
        except Exception as e:
            response_time_ms = int((time.time() - start_time) * 1000)
            return create_component_status(
                name="embeddings",
                status=HealthStatus.UNHEALTHY,
                response_time_ms=response_time_ms,
                error=str(e),
                message="Failed to initialize embeddings service"
            )

    try:
        # Validate embeddings service
        validation_result = await with_timeout(
            embedding_generator.validate_embeddings_service(),
            component_name="embeddings"
        )

        response_time_ms = int((time.time() - start_time) * 1000)

        return create_component_status(
            name="embeddings",
            status=HealthStatus.HEALTHY,
            response_time_ms=response_time_ms,
            message="Embeddings service operational",
            metadata=validation_result
        )

    except asyncio.TimeoutError as e:
        response_time_ms = int((time.time() - start_time) * 1000)
        return create_component_status(
            name="embeddings",
            status=HealthStatus.UNHEALTHY,
            response_time_ms=response_time_ms,
            error=str(e),
            message="Embeddings health check timed out"
        )
    except Exception as e:
        response_time_ms = int((time.time() - start_time) * 1000)
        return create_component_status(
            name="embeddings",
            status=HealthStatus.UNHEALTHY,
            response_time_ms=response_time_ms,
            error=str(e),
            message="Embeddings service check failed"
        )


async def aggregate_health_status(
    qdrant_manager: Optional[QdrantManager] = None,
    embedding_generator: Optional[EmbeddingGenerator] = None
) -> HealthCheckResult:
    """
    Aggregate all component health checks

    Args:
        qdrant_manager: Optional QdrantManager instance
        embedding_generator: Optional EmbeddingGenerator instance

    Returns:
        HealthCheckResult with overall system health
    """
    # Run all health checks in parallel for efficiency
    backend_check = check_backend_health()
    qdrant_check = check_qdrant_health(qdrant_manager) if qdrant_manager else None
    embeddings_check = check_embeddings_health(embedding_generator)

    # Gather results
    checks = [backend_check, embeddings_check]
    if qdrant_check:
        checks.append(qdrant_check)

    component_statuses = await asyncio.gather(*checks, return_exceptions=True)

    # Filter out exceptions and create error statuses
    components = []
    for i, status in enumerate(component_statuses):
        if isinstance(status, Exception):
            component_name = ["backend", "embeddings", "qdrant"][i]
            components.append(
                create_component_status(
                    name=component_name,
                    status=HealthStatus.UNHEALTHY,
                    error=str(status),
                    message=f"{component_name} health check failed with exception"
                )
            )
        else:
            components.append(status)

    # Create and return aggregate result
    return create_health_check_result(components)
