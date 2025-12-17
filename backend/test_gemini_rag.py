"""Test RAG functionality with Gemini embeddings"""
import requests
import json
import os

def test_health():
    """Test health endpoint"""
    print("\n" + "="*70)
    print("Testing Health Endpoint")
    print("="*70)

    try:
        BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
        response = requests.get(f"{BACKEND_URL}/health/qdrant", timeout=60)
        data = response.json()

        print(f"Status Code: {response.status_code}")
        print(f"\nResponse:")
        print(json.dumps(data, indent=2))

        if data.get("status") == "healthy":
            print("\n✓ HEALTH CHECK: PASSED")
            return True
        else:
            print(f"\n✗ HEALTH CHECK: FAILED - Status: {data.get('status')}")
            return False

    except Exception as e:
        print(f"\n✗ HEALTH CHECK: ERROR - {e}")
        return False


def test_rag_query():
    """Test RAG query endpoint"""
    print("\n" + "="*70)
    print("Testing RAG Query")
    print("="*70)

    try:
        payload = {"query": "What is ROS 2?"}
        BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
        response = requests.post(
            f"{BACKEND_URL}/query",
            json=payload,
            timeout=60
        )

        data = response.json()

        print(f"Status Code: {response.status_code}")
        print(f"\nQuery: {payload['query']}")
        print(f"\nAnswer:")
        print(data.get("answer", "No answer"))

        if data.get("citations"):
            print(f"\nCitations: {data['citations']}")

        if data.get("sources"):
            print(f"\nSources ({len(data['sources'])}):")
            for i, source in enumerate(data['sources'][:3], 1):
                print(f"  {i}. Chapter: {source.get('chapter')}")
                print(f"     Score: {source.get('score')}")

        # Check if we got relevant context
        answer = data.get("answer", "")
        if "No relevant context found" in answer:
            print("\n✗ RAG QUERY: FAILED - No relevant context")
            return False
        else:
            print("\n✓ RAG QUERY: PASSED - Got relevant context")
            return True

    except Exception as e:
        print(f"\n✗ RAG QUERY: ERROR - {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests"""
    print("\n" + "#"*70)
    print("# Gemini RAG System Test")
    print("#"*70)

    health_passed = test_health()
    rag_passed = test_rag_query()

    print("\n" + "="*70)
    print("Test Summary")
    print("="*70)
    print(f"Health Endpoint: {'✓ PASSED' if health_passed else '✗ FAILED'}")
    print(f"RAG Query: {'✓ PASSED' if rag_passed else '✗ FAILED'}")

    if health_passed and rag_passed:
        print("\n✓✓✓ ALL TESTS PASSED ✓✓✓")
        return 0
    else:
        print("\n✗✗✗ SOME TESTS FAILED ✗✗✗")
        return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
