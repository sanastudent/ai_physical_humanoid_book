"""
Script to test RAG queries
"""
import os
import sys
import requests
import json

def query_global(backend_url, question):
    """Test global QA mode"""
    endpoint = f'{backend_url}/query'

    payload = {
        "query": question,
        "mode": "global"
    }

    print(f"\n{'='*60}")
    print(f"GLOBAL QA MODE")
    print(f"{'='*60}")
    print(f"Question: {question}\n")

    try:
        response = requests.post(endpoint, json=payload, timeout=30)
        response.raise_for_status()

        result = response.json()

        print("Answer:")
        print(result['answer'])
        print(f"\nCitations: {result['citations']}")
        print(f"Sources: {len(result['sources'])} chunks retrieved")

        return True
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return False

def query_selected(backend_url, question, context):
    """Test selected-text QA mode"""
    endpoint = f'{backend_url}/select'

    payload = {
        "query": question,
        "mode": "selected",
        "context": context
    }

    print(f"\n{'='*60}")
    print(f"SELECTED TEXT QA MODE")
    print(f"{'='*60}")
    print(f"Context: {context[:100]}...")
    print(f"Question: {question}\n")

    try:
        response = requests.post(endpoint, json=payload, timeout=30)
        response.raise_for_status()

        result = response.json()

        print("Answer:")
        print(result['answer'])
        print(f"\nCitations: {result['citations']}")

        return True
    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return False

def main():
    backend_url = os.getenv('BACKEND_URL', 'http://localhost:8000')

    print(f"Testing RAG system at {backend_url}")

    # Test global QA
    query_global(backend_url, "What is ROS 2?")
    query_global(backend_url, "Explain digital twin technology")
    query_global(backend_url, "What are VLA models?")

    # Test selected-text QA
    context = """
    ROS 2 (Robot Operating System 2) is the next generation of the most widely
    used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is
    built from the ground up for production systems, offering real-time
    capabilities, improved security, and multi-robot support.
    """

    query_selected(
        backend_url,
        "What are the main improvements?",
        context
    )

    print(f"\n{'='*60}")
    print("Testing complete!")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
