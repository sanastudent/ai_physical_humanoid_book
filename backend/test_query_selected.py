"""
Test script for /query_selected endpoint

This script tests the Selected Text QA functionality with various scenarios:
1. Valid request with selected text and question
2. Empty selected text (should return friendly message)
3. Empty question
4. Long selected text
"""
import requests
import json
import os

BASE_URL = os.getenv("BACKEND_URL", "http://localhost:8000")

def test_valid_request():
    """Test with valid selected text and question"""
    print("\n" + "="*80)
    print("TEST 1: Valid Request")
    print("="*80)

    payload = {
        "selected_text": """
        ROS 2 (Robot Operating System 2) is the next generation of the most widely
        used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is
        built from the ground up for production systems, offering real-time capabilities,
        improved security, and multi-robot support.
        """,
        "question": "What are the key improvements of ROS 2 over ROS 1?"
    }

    try:
        response = requests.post(f"{BASE_URL}/query_selected", json=payload)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            data = response.json()
            if data.get("answer") and "real-time" in data["answer"].lower():
                print("✓ TEST PASSED: Valid answer returned")
            else:
                print("✗ TEST FAILED: Answer doesn't contain expected content")
        else:
            print("✗ TEST FAILED: Non-200 status code")

    except Exception as e:
        print(f"✗ TEST FAILED: {e}")


def test_empty_selected_text():
    """Test with empty selected text"""
    print("\n" + "="*80)
    print("TEST 2: Empty Selected Text")
    print("="*80)

    payload = {
        "selected_text": "",
        "question": "What is ROS 2?"
    }

    try:
        response = requests.post(f"{BASE_URL}/query_selected", json=payload)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            data = response.json()
            if "Please select some text first" in data.get("answer", ""):
                print("✓ TEST PASSED: Friendly message for empty text")
            else:
                print("✗ TEST FAILED: Unexpected response for empty text")
        else:
            print("✗ TEST FAILED: Non-200 status code")

    except Exception as e:
        print(f"✗ TEST FAILED: {e}")


def test_empty_question():
    """Test with empty question"""
    print("\n" + "="*80)
    print("TEST 3: Empty Question")
    print("="*80)

    payload = {
        "selected_text": "ROS 2 is a robotics middleware framework.",
        "question": ""
    }

    try:
        response = requests.post(f"{BASE_URL}/query_selected", json=payload)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            data = response.json()
            if "Please provide a question" in data.get("answer", ""):
                print("✓ TEST PASSED: Friendly message for empty question")
            else:
                print("✗ TEST FAILED: Unexpected response for empty question")
        else:
            print("✗ TEST FAILED: Non-200 status code")

    except Exception as e:
        print(f"✗ TEST FAILED: {e}")


def test_long_selected_text():
    """Test with long selected text"""
    print("\n" + "="*80)
    print("TEST 4: Long Selected Text")
    print("="*80)

    payload = {
        "selected_text": """
        ROS 2 (Robot Operating System 2) is the next generation of the most widely
        used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is
        built from the ground up for production systems, offering real-time capabilities,
        improved security, and multi-robot support.

        Key improvements include:
        1. Real-Time Performance: RTOS support and deterministic communication
        2. Security: DDS-Security for encrypted, authenticated communication
        3. Multi-Platform: Windows, macOS, Linux, embedded systems
        4. Production-Ready: Quality of Service (QoS) policies
        5. Multi-Robot Systems: Native support for robot teams

        The DDS middleware provides automatic discovery, publish-subscribe messaging,
        request-reply patterns, and configurable Quality of Service policies.
        """ * 3,  # Repeat 3 times to make it longer
        "question": "How many key improvements are listed for ROS 2?"
    }

    try:
        response = requests.post(f"{BASE_URL}/query_selected", json=payload)
        print(f"Status Code: {response.status_code}")
        print(f"Response (truncated): {json.dumps(response.json(), indent=2)[:500]}...")

        if response.status_code == 200:
            data = response.json()
            if data.get("answer"):
                print("✓ TEST PASSED: Answer generated for long text")
            else:
                print("✗ TEST FAILED: No answer for long text")
        else:
            print("✗ TEST FAILED: Non-200 status code")

    except Exception as e:
        print(f"✗ TEST FAILED: {e}")


if __name__ == "__main__":
    print("\n" + "="*80)
    print("TESTING /query_selected ENDPOINT")
    print("="*80)
    print(f"Target: {BASE_URL}/query_selected")
    print("\nMake sure the backend server is running on port 8000")
    print("="*80)

    # Run all tests
    test_valid_request()
    test_empty_selected_text()
    test_empty_question()
    test_long_selected_text()

    print("\n" + "="*80)
    print("ALL TESTS COMPLETED")
    print("="*80)
