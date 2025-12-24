import requests
import json
import time

# Test the API endpoints
base_url = "http://localhost:8001"

def test_health():
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code}")
        print(f"Response: {response.json()}")
        return response.status_code == 200
    except Exception as e:
        print(f"Health check failed: {e}")
        return False

def test_query():
    try:
        # Start timer
        start_time = time.time()

        response = requests.post(
            f"{base_url}/query",
            headers={"Content-Type": "application/json"},
            json={"query": "What is a simple test?"},
            timeout=120  # 120 second timeout to allow for fallback mechanisms
        )

        end_time = time.time()
        print(f"Query test: {response.status_code}")
        print(f"Time taken: {end_time - start_time:.2f} seconds")

        if response.status_code == 200:
            data = response.json()
            print(f"Answer: {data.get('answer', 'No answer field')[:200]}...")
            print(f"Has citations: {len(data.get('citations', []))}")
            return True
        else:
            print(f"Query failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except requests.exceptions.Timeout:
        print("Query timed out after 120 seconds")
        return False
    except Exception as e:
        print(f"Query failed: {e}")
        return False

if __name__ == "__main__":
    print("Testing API endpoints...")

    print("\n1. Testing health endpoint:")
    health_ok = test_health()

    print("\n2. Testing query endpoint:")
    query_ok = test_query()

    print(f"\nResults:")
    print(f"Health check: {'PASS' if health_ok else 'FAIL'}")
    print(f"Query test: {'PASS' if query_ok else 'FAIL'}")

    if health_ok and query_ok:
        print("\nSUCCESS: API is working correctly with fallback mechanisms!")
    else:
        print("\nFAILURE: API has issues")