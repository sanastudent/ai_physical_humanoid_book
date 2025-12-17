#!/usr/bin/env python3
"""
Final test to verify all RAG functionality is working properly
"""
import os
import sys
import json

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.schema import QueryRequest
from src.main import app
from fastapi.testclient import TestClient

def test_api_endpoints():
    """Test the main API endpoints to ensure they're working"""
    print("Testing API endpoints...")

    # Create a test client
    client = TestClient(app)

    # Test health endpoint
    print("\n1. Testing health endpoint...")
    try:
        response = client.get("/health")
        print(f"Health endpoint: {response.status_code}")
        if response.status_code == 200:
            print("✓ Health endpoint working")
        else:
            print(f"✗ Health endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Health endpoint error: {e}")

    # Test health ready endpoint
    print("\n2. Testing readiness endpoint...")
    try:
        response = client.get("/health/ready")
        print(f"Readiness endpoint: {response.status_code}")
        if response.status_code in [200, 503]:  # 503 is expected if dependencies are down
            print("✓ Readiness endpoint working")
        else:
            print(f"✗ Readiness endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Readiness endpoint error: {e}")

    # Test translation endpoint exists
    print("\n3. Testing translation endpoint availability...")
    try:
        # This will fail due to missing body, but should return 422 (validation error) not 404 (not found)
        response = client.post("/translate", json={})
        print(f"Translation endpoint: {response.status_code}")
        if response.status_code == 422:  # Validation error is expected
            print("✓ Translation endpoint available")
        elif response.status_code == 404:
            print("✗ Translation endpoint not found")
        else:
            print(f"? Translation endpoint response: {response.status_code}")
    except Exception as e:
        print(f"✗ Translation endpoint error: {e}")

    # Test query endpoint
    print("\n4. Testing query endpoint...")
    try:
        query_data = {"query": "What is artificial intelligence?"}
        response = client.post("/query", json=query_data)
        print(f"Query endpoint: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Query endpoint working, answer: {result.get('answer', 'No answer')[:50]}...")
        else:
            print(f"✗ Query endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Query endpoint error: {e}")

    # Test select endpoint
    print("\n5. Testing select endpoint...")
    try:
        query_data = {
            "query": "What does this text mean?",
            "context": "Sample text for testing"
        }
        response = client.post("/select", json=query_data)
        print(f"Select endpoint: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"✓ Select endpoint working, answer: {result.get('answer', 'No answer')[:50]}...")
        elif response.status_code == 400:
            print("✓ Select endpoint available (expected 400 for missing context)")
        else:
            print(f"✗ Select endpoint failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Select endpoint error: {e}")

if __name__ == "__main__":
    print("Running final API endpoint tests...")
    test_api_endpoints()
    print("\nAPI endpoint tests completed!")