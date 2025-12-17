"""
Integration tests for Authentication API endpoints

Tests the complete authentication flow:
1. User signup with email/password
2. User signin with credentials
3. Session validation
4. User signout
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
def cleanup_test_user():
    """Cleanup test user after each test"""
    test_email = f"test_{uuid4()}@example.com"

    yield test_email

    # Cleanup: Delete test user and associated data
    try:
        user = UserService.get_user_by_email(test_email)
        if user:
            # Delete sessions
            SessionService.invalidate_user_sessions(user.id)
            # Delete background data
            BackgroundService.delete_background(user.id)
            # Delete user
            db_pool = get_db_pool()
            with db_pool.get_cursor() as cursor:
                cursor.execute("DELETE FROM users WHERE email = %s", (test_email,))
    except Exception as e:
        print(f"Cleanup warning: {e}")


def test_signup_success(cleanup_test_user):
    """Test successful user signup"""
    test_email = cleanup_test_user

    response = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "TestPassword123"
        }
    )

    assert response.status_code == 201
    data = response.json()
    assert data["email"] == test_email
    assert "id" in data
    assert "created_at" in data
    assert "password" not in data  # Password should not be in response

    # Verify session cookie is set
    assert "session_token" in response.cookies


def test_signup_duplicate_email(cleanup_test_user):
    """Test signup with duplicate email"""
    test_email = cleanup_test_user

    # First signup
    response1 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "TestPassword123"
        }
    )
    assert response1.status_code == 201

    # Duplicate signup
    response2 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "DifferentPassword456"
        }
    )
    assert response2.status_code == 400
    assert "already registered" in response2.json()["detail"].lower()


def test_signup_weak_password(cleanup_test_user):
    """Test signup with weak password"""
    test_email = cleanup_test_user

    # No uppercase
    response1 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "testpassword123"
        }
    )
    assert response1.status_code == 400

    # No lowercase
    response2 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "TESTPASSWORD123"
        }
    )
    assert response2.status_code == 400

    # No digit
    response3 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "TestPassword"
        }
    )
    assert response3.status_code == 400

    # Too short
    response4 = client.post(
        "/auth/signup",
        json={
            "email": test_email,
            "password": "Test12"
        }
    )
    assert response4.status_code in [400, 422]  # 422 is Pydantic validation error


def test_signup_invalid_email(cleanup_test_user):
    """Test signup with invalid email"""
    response = client.post(
        "/auth/signup",
        json={
            "email": "invalid-email",
            "password": "TestPassword123"
        }
    )
    assert response.status_code in [400, 422]  # 422 is Pydantic validation error


def test_signin_success(cleanup_test_user):
    """Test successful signin"""
    test_email = cleanup_test_user
    password = "TestPassword123"

    # Signup first
    signup_response = client.post(
        "/auth/signup",
        json={"email": test_email, "password": password}
    )
    assert signup_response.status_code == 201

    # Signin
    signin_response = client.post(
        "/auth/signin",
        json={"email": test_email, "password": password, "remember_me": False}
    )

    assert signin_response.status_code == 200
    data = signin_response.json()
    assert data["email"] == test_email
    assert "session_token" in signin_response.cookies


def test_signin_wrong_password(cleanup_test_user):
    """Test signin with wrong password"""
    test_email = cleanup_test_user

    # Signup
    client.post(
        "/auth/signup",
        json={"email": test_email, "password": "TestPassword123"}
    )

    # Signin with wrong password
    response = client.post(
        "/auth/signin",
        json={"email": test_email, "password": "WrongPassword456"}
    )
    assert response.status_code == 401


def test_signin_nonexistent_user():
    """Test signin with nonexistent email"""
    response = client.post(
        "/auth/signin",
        json={"email": "nonexistent@example.com", "password": "TestPassword123"}
    )
    assert response.status_code == 401


def test_get_session_authenticated(cleanup_test_user):
    """Test getting session with valid authentication"""
    test_email = cleanup_test_user

    # Signup and get session cookie
    signup_response = client.post(
        "/auth/signup",
        json={"email": test_email, "password": "TestPassword123"}
    )
    session_cookie = signup_response.cookies.get("session_token")

    # Get session
    response = client.get(
        "/auth/session",
        cookies={"session_token": session_cookie}
    )

    assert response.status_code == 200
    data = response.json()
    assert data["email"] == test_email


def test_get_session_unauthenticated():
    """Test getting session without authentication"""
    response = client.get("/auth/session")
    assert response.status_code == 401


def test_get_session_invalid_token():
    """Test getting session with invalid token"""
    response = client.get(
        "/auth/session",
        cookies={"session_token": "invalid_token_12345"}
    )
    assert response.status_code == 401


def test_signout_success(cleanup_test_user):
    """Test successful signout"""
    test_email = cleanup_test_user

    # Signup
    signup_response = client.post(
        "/auth/signup",
        json={"email": test_email, "password": "TestPassword123"}
    )
    session_cookie = signup_response.cookies.get("session_token")

    # Signout
    signout_response = client.post(
        "/auth/signout",
        cookies={"session_token": session_cookie}
    )
    assert signout_response.status_code == 204

    # Verify session is invalidated
    session_check = client.get(
        "/auth/session",
        cookies={"session_token": session_cookie}
    )
    assert session_check.status_code == 401


def test_complete_auth_flow(cleanup_test_user):
    """Test complete authentication flow: signup → signin → get session → signout"""
    test_email = cleanup_test_user
    password = "TestPassword123"

    # 1. Signup
    signup_response = client.post(
        "/auth/signup",
        json={"email": test_email, "password": password}
    )
    assert signup_response.status_code == 201
    signup_cookie = signup_response.cookies.get("session_token")

    # 2. Verify session after signup
    session_check1 = client.get(
        "/auth/session",
        cookies={"session_token": signup_cookie}
    )
    assert session_check1.status_code == 200

    # 3. Signout
    signout_response = client.post(
        "/auth/signout",
        cookies={"session_token": signup_cookie}
    )
    assert signout_response.status_code == 204

    # 4. Signin again
    signin_response = client.post(
        "/auth/signin",
        json={"email": test_email, "password": password}
    )
    assert signin_response.status_code == 200
    signin_cookie = signin_response.cookies.get("session_token")

    # 5. Verify new session
    session_check2 = client.get(
        "/auth/session",
        cookies={"session_token": signin_cookie}
    )
    assert session_check2.status_code == 200


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
