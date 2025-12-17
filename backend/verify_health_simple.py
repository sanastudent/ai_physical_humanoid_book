"""Simple health check verification"""
import requests
import json
import os

try:
    BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
    response = requests.get(f"{BACKEND_URL}/health/qdrant", timeout=60)
    print(f"Status Code: {response.status_code}\n")
    data = response.json()
    print(json.dumps(data, indent=2))

    print("\n" + "="*70)
    if data.get("status") == "healthy":
        print("SUCCESS: QDRANT IS HEALTHY!")
    elif data.get("status") == "degraded":
        print(f"DEGRADED: {data.get('message')}")
    else:
        print(f"UNKNOWN STATUS: {data.get('status')}")
    print("="*70)

except Exception as e:
    print(f"Error: {e}")
