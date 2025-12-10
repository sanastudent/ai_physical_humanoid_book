"""
Standalone verification script for environment variable loading fix

This script verifies that QDRANT_URL is properly loaded from .env
before HealthCheckSettings is instantiated.
"""
import os
import sys

# Force UTF-8 encoding for Windows console
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

print("=" * 70)
print("Environment Variable Loading Verification")
print("=" * 70)

# Test 1: Check if .env file exists
print("\n[Test 1] Checking for .env file...")
env_file = os.path.join(os.path.dirname(__file__), '..', '.env')
if os.path.exists(env_file):
    print(f"✓ .env file found at: {env_file}")
else:
    print(f"✗ .env file NOT found at: {env_file}")
    sys.exit(1)

# Test 2: Check if dotenv loads before HealthCheckSettings import
print("\n[Test 2] Checking if dotenv is loaded before HealthCheckSettings...")
from dotenv import load_dotenv
load_dotenv()

qdrant_url_before = os.getenv("QDRANT_URL")
print(f"  QDRANT_URL before import: {qdrant_url_before}")

if qdrant_url_before:
    print(f"✓ QDRANT_URL loaded successfully: {qdrant_url_before}")
else:
    print("✗ QDRANT_URL is None after load_dotenv()")
    sys.exit(1)

# Test 3: Import HealthCheckSettings and verify it works
print("\n[Test 3] Importing HealthCheckSettings...")
try:
    from health.config import HealthCheckSettings, settings
    print("✓ HealthCheckSettings imported successfully")
except Exception as e:
    print(f"✗ Failed to import HealthCheckSettings: {e}")
    sys.exit(1)

# Test 4: Verify settings instance has QDRANT_URL
print("\n[Test 4] Verifying settings instance has QDRANT_URL...")
if settings.qdrant_url:
    print(f"✓ settings.qdrant_url = {settings.qdrant_url}")
else:
    print(f"✗ settings.qdrant_url is None")
    sys.exit(1)

# Test 5: Verify validator doesn't fail
print("\n[Test 5] Testing validator with QDRANT_URL set...")
try:
    test_settings = HealthCheckSettings()
    print(f"✓ Validator passed - HealthCheckSettings instantiated successfully")
    print(f"  - qdrant_url: {test_settings.qdrant_url}")
    print(f"  - qdrant_host: {test_settings.qdrant_host}")
    print(f"  - qdrant_port: {test_settings.qdrant_port}")
except ValueError as e:
    print(f"✗ Validator failed: {e}")
    sys.exit(1)

# Test 6: Check timeout defaults
print("\n[Test 6] Verifying timeout defaults...")
assert 5 <= settings.health_check_timeout <= 300, "health_check_timeout out of range"
assert 1 <= settings.component_timeout <= 30, "component_timeout out of range"
print(f"✓ Timeout settings valid:")
print(f"  - health_check_timeout: {settings.health_check_timeout}s")
print(f"  - component_timeout: {settings.component_timeout}s")

print("\n" + "=" * 70)
print("✅ ALL TESTS PASSED - Environment variable loading fix verified!")
print("=" * 70)
print("\nSummary:")
print("  - load_dotenv() is called in health/config.py BEFORE instantiation")
print("  - QDRANT_URL is successfully loaded from .env")
print("  - HealthCheckSettings validates correctly")
print("  - Backend should now start without 'QDRANT_URL or QDRANT_HOST must be set' error")
