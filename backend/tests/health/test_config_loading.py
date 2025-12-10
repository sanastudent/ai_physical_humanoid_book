"""
Test to verify environment variables load correctly for health check configuration
"""
import os
import sys
import pytest

# Add src directory to Python path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from health.config import HealthCheckSettings, settings


def test_qdrant_url_loads_from_env():
    """Verify QDRANT_URL is loaded from .env file"""
    # This test verifies the fix for the environment variable loading issue
    # where load_dotenv() was called after HealthCheckSettings was instantiated

    # Check that QDRANT_URL is loaded
    assert settings.qdrant_url is not None, "QDRANT_URL should be loaded from .env"
    assert settings.qdrant_url.startswith("http"), "QDRANT_URL should be a valid URL"

    print(f"✓ QDRANT_URL loaded successfully: {settings.qdrant_url}")


def test_health_check_settings_instantiation():
    """Verify HealthCheckSettings can be instantiated without errors"""
    try:
        test_settings = HealthCheckSettings()
        assert test_settings is not None
        print(f"✓ HealthCheckSettings instantiated successfully")
    except ValueError as e:
        pytest.fail(f"HealthCheckSettings instantiation failed: {e}")


def test_qdrant_connection_validator():
    """Test that either QDRANT_URL or QDRANT_HOST must be set"""
    # If QDRANT_URL is set, validation should pass
    if settings.qdrant_url:
        assert settings.qdrant_url is not None
        print(f"✓ QDRANT_URL validation passed")
    # If QDRANT_URL is not set, QDRANT_HOST should be set
    elif settings.qdrant_host:
        assert settings.qdrant_host is not None
        print(f"✓ QDRANT_HOST validation passed")
    else:
        pytest.fail("Neither QDRANT_URL nor QDRANT_HOST is set")


def test_settings_singleton_is_valid():
    """Test that the global settings instance is valid"""
    assert settings is not None, "Global settings instance should exist"

    # Verify critical configuration is present
    assert settings.qdrant_url or settings.qdrant_host, \
        "Either QDRANT_URL or QDRANT_HOST must be configured"

    # Verify timeout settings have sensible defaults
    assert 5 <= settings.health_check_timeout <= 300, \
        "health_check_timeout should be between 5 and 300 seconds"
    assert 1 <= settings.component_timeout <= 30, \
        "component_timeout should be between 1 and 30 seconds"

    print(f"✓ Global settings singleton is valid")
    print(f"  - QDRANT_URL: {settings.qdrant_url}")
    print(f"  - health_check_timeout: {settings.health_check_timeout}s")
    print(f"  - component_timeout: {settings.component_timeout}s")


if __name__ == "__main__":
    # Run tests directly
    test_qdrant_url_loads_from_env()
    test_health_check_settings_instantiation()
    test_qdrant_connection_validator()
    test_settings_singleton_is_valid()
    print("\n✅ All configuration loading tests passed!")
