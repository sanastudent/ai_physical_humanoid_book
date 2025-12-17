"""Quick health check verification script"""
import requests
import json
import os

def check_health():
    print("\n" + "="*70)
    print("Verifying Qdrant Health Status")
    print("="*70 + "\n")

    try:
        BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
        response = requests.get(f"{BACKEND_URL}/health/qdrant", timeout=60)

        print(f"Status Code: {response.status_code}")
        print(f"\nResponse:")
        print(json.dumps(response.json(), indent=2))

        data = response.json()

        if data.get("status") == "healthy":
            print("\n✓ QDRANT IS HEALTHY!")
            return True
        elif data.get("status") == "degraded":
            print("\n⚠ QDRANT IS DEGRADED")
            print(f"Message: {data.get('message')}")
            return False
        else:
            print(f"\n? UNKNOWN STATUS: {data.get('status')}")
            return False

    except requests.exceptions.Timeout:
        print("✗ Request timed out after 60 seconds")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == "__main__":
    success = check_health()
    exit(0 if success else 1)
