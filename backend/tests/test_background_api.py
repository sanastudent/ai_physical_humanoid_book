"""
Integration tests for Background API endpoints

Tests the complete background data flow:
1. Create background data after signup
2. Get background data
3. Update background data
4. Delete background data
"""

import pytest
from fastapi.testclient import TestClient
from uuid import uuid4

from src.main import app
from src.database.connection import get_db_pool
from src.services.user_service import UserService
from src.services.session_service import SessionService
from src.services.background_service import BackgroundService

client = TestClient(app)


@pytest.fixture(scope="function")
def authenticated_user():
    """Create a test user and return email, password, and session cookie"""
    test_email = f"test_{uuid4()}@example.com"
    password = "TestPassword123"

    # Signup
    response = client.post(
        "/auth/signup",
        json={"email": test_email, "password": password}
    )
    session_cookie = response.cookies.get("session_token")

    yield {
        "email": test_email,
        "password": password,
        "session_cookie": session_cookie
    }

    # Cleanup
    try:
        user = UserService.get_user_by_email(test_email)
        if user:
            SessionService.invalidate_user_sessions(user.id)
            BackgroundService.delete_background(user.id)
            db_pool = get_db_pool()
            with db_pool.get_cursor() as cursor:
                cursor.execute("DELETE FROM users WHERE email = %s", (test_email,))
    except Exception as e:
        print(f"Cleanup warning: {e}")


def test_create_background_success(authenticated_user):
    """Test successful background creation"""
    session_cookie = authenticated_user["session_cookie"]

    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python", "JavaScript", "Go"],
            "preferred_frameworks": ["FastAPI", "React"]
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web", "desktop"],
            "device_types": ["laptop", "smartphone"]
        }
    }

    response = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )

    assert response.status_code == 201
    data = response.json()
    assert data["software"]["experience_level"] == "intermediate"
    assert "Python" in data["software"]["preferred_languages"]
    assert data["hardware"]["experience_level"] == "beginner"
    assert "laptop" in data["hardware"]["device_types"]


def test_create_background_unauthenticated():
    """Test background creation without authentication"""
    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    response = client.post("/background", json=background_data)
    assert response.status_code == 401


def test_create_background_duplicate(authenticated_user):
    """Test creating background when it already exists"""
    session_cookie = authenticated_user["session_cookie"]

    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    # First creation
    response1 = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert response1.status_code == 201

    # Duplicate creation
    response2 = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert response2.status_code == 400
    assert "already exists" in response2.json()["detail"].lower()


def test_create_background_invalid_experience_level(authenticated_user):
    """Test background creation with invalid experience level"""
    session_cookie = authenticated_user["session_cookie"]

    background_data = {
        "software": {
            "experience_level": "invalid_level",  # Invalid
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    response = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert response.status_code == 422  # Pydantic validation error


def test_create_background_empty_languages(authenticated_user):
    """Test background creation with empty required arrays"""
    session_cookie = authenticated_user["session_cookie"]

    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": [],  # Empty - should fail
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    response = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert response.status_code == 422  # Pydantic validation error (min_length=1)


def test_get_background_success(authenticated_user):
    """Test successful background retrieval"""
    session_cookie = authenticated_user["session_cookie"]

    # Create background first
    background_data = {
        "software": {
            "experience_level": "advanced",
            "preferred_languages": ["Rust", "Go"],
            "preferred_frameworks": ["Fiber"]
        },
        "hardware": {
            "experience_level": "intermediate",
            "preferred_platforms": ["embedded", "iot"],
            "device_types": ["raspberry-pi", "arduino"]
        }
    }

    create_response = client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert create_response.status_code == 201

    # Get background
    get_response = client.get(
        "/background",
        cookies={"session_token": session_cookie}
    )

    assert get_response.status_code == 200
    data = get_response.json()
    assert data["software"]["experience_level"] == "advanced"
    assert "Rust" in data["software"]["preferred_languages"]
    assert "raspberry-pi" in data["hardware"]["device_types"]


def test_get_background_not_found(authenticated_user):
    """Test getting background when it doesn't exist"""
    session_cookie = authenticated_user["session_cookie"]

    response = client.get(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert response.status_code == 404


def test_get_background_unauthenticated():
    """Test getting background without authentication"""
    response = client.get("/background")
    assert response.status_code == 401


def test_update_background_success(authenticated_user):
    """Test successful background update"""
    session_cookie = authenticated_user["session_cookie"]

    # Create initial background
    initial_data = {
        "software": {
            "experience_level": "beginner",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    client.post(
        "/background",
        json=initial_data,
        cookies={"session_token": session_cookie}
    )

    # Update background
    updated_data = {
        "software": {
            "experience_level": "intermediate",  # Changed
            "preferred_languages": ["Python", "JavaScript"],  # Added language
            "preferred_frameworks": ["FastAPI"]  # Added framework
        },
        "hardware": {
            "experience_level": "intermediate",  # Changed
            "preferred_platforms": ["web", "mobile"],  # Added platform
            "device_types": ["laptop", "smartphone"]  # Added devices
        }
    }

    update_response = client.put(
        "/background",
        json=updated_data,
        cookies={"session_token": session_cookie}
    )

    assert update_response.status_code == 200
    data = update_response.json()
    assert data["software"]["experience_level"] == "intermediate"
    assert "JavaScript" in data["software"]["preferred_languages"]
    assert "FastAPI" in data["software"]["preferred_frameworks"]
    assert data["hardware"]["experience_level"] == "intermediate"
    assert "smartphone" in data["hardware"]["device_types"]


def test_update_background_not_found(authenticated_user):
    """Test updating background when it doesn't exist"""
    session_cookie = authenticated_user["session_cookie"]

    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    response = client.put(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )
    assert response.status_code == 400
    assert "does not exist" in response.json()["detail"].lower()


def test_delete_background_success(authenticated_user):
    """Test successful background deletion"""
    session_cookie = authenticated_user["session_cookie"]

    # Create background
    background_data = {
        "software": {
            "experience_level": "intermediate",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    client.post(
        "/background",
        json=background_data,
        cookies={"session_token": session_cookie}
    )

    # Delete background
    delete_response = client.delete(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert delete_response.status_code == 204

    # Verify deletion
    get_response = client.get(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert get_response.status_code == 404


def test_complete_background_flow(authenticated_user):
    """Test complete background flow: create → get → update → delete"""
    session_cookie = authenticated_user["session_cookie"]

    # 1. Create
    create_data = {
        "software": {
            "experience_level": "beginner",
            "preferred_languages": ["Python"],
            "preferred_frameworks": []
        },
        "hardware": {
            "experience_level": "beginner",
            "preferred_platforms": ["web"],
            "device_types": []
        }
    }

    create_response = client.post(
        "/background",
        json=create_data,
        cookies={"session_token": session_cookie}
    )
    assert create_response.status_code == 201

    # 2. Get
    get_response = client.get(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert get_response.status_code == 200

    # 3. Update
    update_data = {
        "software": {
            "experience_level": "advanced",
            "preferred_languages": ["Python", "Rust"],
            "preferred_frameworks": ["FastAPI"]
        },
        "hardware": {
            "experience_level": "intermediate",
            "preferred_platforms": ["web", "embedded"],
            "device_types": ["laptop", "raspberry-pi"]
        }
    }

    update_response = client.put(
        "/background",
        json=update_data,
        cookies={"session_token": session_cookie}
    )
    assert update_response.status_code == 200
    assert update_response.json()["software"]["experience_level"] == "advanced"

    # 4. Delete
    delete_response = client.delete(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert delete_response.status_code == 204

    # 5. Verify deletion
    final_get = client.get(
        "/background",
        cookies={"session_token": session_cookie}
    )
    assert final_get.status_code == 404


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
