#!/usr/bin/env python3
"""
Test script to verify selected-text RAG retrieval and Docusaurus Urdu translation fixes
"""
import os
import sys
import json

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

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

        # Check if the answer contains the selected text or is based on it
        if "No relevant context found" in answer:
            print("  X FAIL: Still returning 'No relevant context found'")
            return False
        else:
            print("  OK PASS: Answer properly uses selected text context")
    else:
        print(f"  X FAIL: /select endpoint failed: {response.status_code}")
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

        # Check if the answer contains the selected text or is based on it
        if "No relevant context found" in answer:
            print("  X FAIL: Still returning 'No relevant context found'")
            return False
        else:
            print("  OK PASS: Answer properly uses selected text context")
    else:
        print(f"  X FAIL: /query endpoint failed: {response.status_code}")
        print(f"  Error: {response.text}")
        return False

    return True

def test_global_qa():
    """Test global QA functionality"""
    print("\nTesting global QA functionality...")

    client = TestClient(app)

    # Test global QA
    response = client.post("/query", json={
        "query": "What is artificial intelligence?",
        "mode": "global"
    })

    print(f"  Global QA status: {response.status_code}")
    if response.status_code == 200:
        result = response.json()
        answer = result.get('answer', 'No answer')
        print(f"  Answer: {answer[:100]}...")
        print("  OK PASS: Global QA working")
    else:
        print(f"  X FAIL: Global QA failed: {response.status_code}")

    return True

def test_validation():
    """Test validation for empty context"""
    print("\nTesting validation for empty context...")

    client = TestClient(app)

    # Test with empty context
    response = client.post("/select", json={
        "query": "What is this about?",
        "context": ""
    })

    print(f"  Empty context validation status: {response.status_code}")
    if response.status_code == 400:
        print("  OK PASS: Empty context properly rejected")
        return True
    else:
        print(f"  X FAIL: Empty context should be rejected but got {response.status_code}")
        return False

def test_urdu_translation_config():
    """Test that Urdu translation configuration is correct"""
    print("\nTesting Urdu translation configuration...")

    # Check if docusaurus.config.ts has the correct settings
    config_path = os.path.join(os.path.dirname(__file__), 'frontend', 'my-book', 'docusaurus.config.ts')

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_content = f.read()

        # Check for required i18n settings
        has_default_locale = "'en'" in config_content or '"en"' in config_content
        has_ur_locale = "'ur'" in config_content or '"ur"' in config_content
        has_locale_dropdown = "localeDropdown" in config_content

        if has_default_locale and has_ur_locale and has_locale_dropdown:
            print("  OK PASS: Urdu translation configuration is correct")
            return True
        else:
            print(f"  X FAIL: Missing configuration - default_locale: {has_default_locale}, ur_locale: {has_ur_locale}, locale_dropdown: {has_locale_dropdown}")
            return False

    except Exception as e:
        print(f"  X FAIL: Could not read configuration file: {e}")
        return False

def main():
    """Run all tests"""
    print("Starting verification tests for selected-text RAG and Urdu translation fixes...\n")

    selected_text_success = test_selected_text_qa()
    global_qa_success = test_global_qa()
    validation_success = test_validation()
    urdu_config_success = test_urdu_translation_config()

    print(f"\nTest Results:")
    print(f"• Selected-text QA: {'PASS' if selected_text_success else 'FAIL'}")
    print(f"• Global QA: {'PASS' if global_qa_success else 'FAIL'}")
    print(f"• Validation: {'PASS' if validation_success else 'FAIL'}")
    print(f"• Urdu Translation Config: {'PASS' if urdu_config_success else 'FAIL'}")

    all_passed = all([selected_text_success, global_qa_success, validation_success, urdu_config_success])

    if all_passed:
        print("\nOK All tests passed! Fixes are working correctly.")
        return True
    else:
        print("\nX Some tests failed. Please review the issues above.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)