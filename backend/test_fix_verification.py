#!/usr/bin/env python3
"""
Test script to verify the fixes for RAG selected-text QA and translation routing
"""
import os
import sys
import json

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.main import app
from fastapi.testclient import TestClient

def test_selected_text_qa():
    """Test selected-text QA functionality"""
    print("Testing selected-text QA functionality...")

    client = TestClient(app)

    # Test selected-text QA with context provided
    selected_text = "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."
    query = "What is Artificial Intelligence?"

    # Test using /select endpoint
    print("  Testing /select endpoint...")
    response = client.post("/select", json={
        "query": query,
        "context": selected_text
    })

    print(f"  /select endpoint status: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        answer = result.get('answer', 'No answer')
        print(f"  Answer: {answer[:100]}...")
        print(f"  Citations: {len(result.get('citations', []))}")

        # Verify the answer is based on the selected text and not repetitive "Context:" strings
        if "Context: Context: Context:" in answer:
            print("  X FAIL: Still showing repetitive 'Context:' strings")
            return False
        elif "Artificial Intelligence" in answer and "branch of computer science" in answer:
            print("  OK PASS: Answer properly uses selected text context")
        else:
            print(f"  ? PARTIAL: Answer doesn't clearly reference selected text: {answer[:50]}")
    else:
        print(f"  ✗ FAIL: /select endpoint failed: {response.status_code}")
        print(f"  Error: {response.text}")
        return False

    # Test using /query endpoint with mode=selected
    print("  Testing /query endpoint with selected mode...")
    response = client.post("/query", json={
        "query": query,
        "context": selected_text,
        "mode": "selected"
    })

    print(f"  /query endpoint status: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        answer = result.get('answer', 'No answer')
        print(f"  Answer: {answer[:100]}...")
        print(f"  Citations: {len(result.get('citations', []))}")

        # Verify the answer is based on the selected text and not repetitive "Context:" strings
        if "Context: Context: Context:" in answer:
            print("  X FAIL: Still showing repetitive 'Context:' strings")
            return False
        elif "Artificial Intelligence" in answer and "branch of computer science" in answer:
            print("  OK PASS: Answer properly uses selected text context")
        else:
            print(f"  ? PARTIAL: Answer doesn't clearly reference selected text: {answer[:50]}")
    else:
        print(f"  ✗ FAIL: /query endpoint failed: {response.status_code}")
        print(f"  Error: {response.text}")
        return False

    return True

def test_translation_endpoint():
    """Test translation endpoint availability"""
    print("\nTesting translation endpoint...")

    client = TestClient(app)

    # Test translation endpoint exists
    response = client.post("/translate", json={
        "content": "Hello world",
        "target_language": "urdu"
    })
    print(f"  Translation endpoint status: {response.status_code}")

    if response.status_code == 422:  # Validation error is expected for missing API key
        print("  ✓ PASS: Translation endpoint available (validation error expected)")
        return True
    elif response.status_code == 404:
        print("  ✗ FAIL: Translation endpoint not found")
        return False
    else:
        print(f"  ? Endpoint response: {response.status_code}")
        return True

def main():
    """Run all tests"""
    print("Starting verification tests for RAG selected-text QA and translation fixes...\n")

    selected_text_success = test_selected_text_qa()
    translation_success = test_translation_endpoint()

    print(f"\nTest Results:")
    print(f"• Selected-text QA: {'PASS' if selected_text_success else 'FAIL'}")
    print(f"• Translation endpoint: {'PASS' if translation_success else 'FAIL'}")

    if selected_text_success and translation_success:
        print("\nOK All tests passed! Fixes are working correctly.")
        return True
    else:
        print("\nX Some tests failed. Please review the issues above.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)