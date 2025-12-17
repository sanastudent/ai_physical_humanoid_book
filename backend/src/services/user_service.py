"""
User Service - Business logic for user management and authentication
"""

from datetime import datetime
from typing import Optional, Dict
from uuid import UUID
import uuid

from ..config.auth_config import hash_password, verify_password
from ..database.connection import get_db_pool
from ..models.user import User, UserCreate, UserResponse
from ..utils.validators import validate_email, validate_password_strength, ValidationError


class UserService:
    """Service for user authentication and management"""

    @staticmethod
    def create_user(user_data: UserCreate) -> UserResponse:
        """
        Create a new user account

        Args:
            user_data: User creation data (email, password)

        Returns:
            UserResponse: Created user data (without password)

        Raises:
            ValidationError: If email is invalid or already exists
            Exception: If database operation fails
        """
        # Validate email format
        normalized_email = validate_email(user_data.email)

        # Validate password strength
        validate_password_strength(user_data.password)

        # Check if email already exists
        if UserService.check_email_exists(normalized_email):
            raise ValidationError("Email already registered")

        # Hash password
        password_hash = hash_password(user_data.password)

        # Insert user into database
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                INSERT INTO users (email, password_hash, created_at, updated_at)
                VALUES (%s, %s, NOW(), NOW())
                RETURNING id, email, created_at, updated_at
                """,
                (normalized_email, password_hash)
            )
            user_record = cursor.fetchone()

        if not user_record:
            raise Exception("Failed to create user")

        return UserResponse(
            id=user_record['id'],
            email=user_record['email'],
            created_at=user_record['created_at'],
            updated_at=user_record['updated_at']
        )

    @staticmethod
    def check_email_exists(email: str) -> bool:
        """
        Check if email already exists in database

        Args:
            email: Email address to check

        Returns:
            bool: True if email exists, False otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "SELECT EXISTS(SELECT 1 FROM users WHERE email = %s)",
                (email,)
            )
            result = cursor.fetchone()
            return result['exists'] if result else False

    @staticmethod
    def authenticate_user(email: str, password: str) -> Optional[UserResponse]:
        """
        Authenticate user with email and password

        Args:
            email: User email
            password: Plain text password

        Returns:
            UserResponse: User data if authentication successful, None otherwise
        """
        # Normalize email
        try:
            normalized_email = validate_email(email)
        except ValidationError:
            return None

        # Get user from database
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, email, password_hash, created_at, updated_at
                FROM users
                WHERE email = %s
                """,
                (normalized_email,)
            )
            user_record = cursor.fetchone()

        if not user_record:
            return None

        # Verify password
        if not verify_password(password, user_record['password_hash']):
            return None

        return UserResponse(
            id=user_record['id'],
            email=user_record['email'],
            created_at=user_record['created_at'],
            updated_at=user_record['updated_at']
        )

    @staticmethod
    def get_user_by_id(user_id: UUID) -> Optional[UserResponse]:
        """
        Get user by ID

        Args:
            user_id: User UUID

        Returns:
            UserResponse: User data if found, None otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, email, created_at, updated_at
                FROM users
                WHERE id = %s
                """,
                (str(user_id),)
            )
            user_record = cursor.fetchone()

        if not user_record:
            return None

        return UserResponse(
            id=user_record['id'],
            email=user_record['email'],
            created_at=user_record['created_at'],
            updated_at=user_record['updated_at']
        )

    @staticmethod
    def get_user_by_email(email: str) -> Optional[UserResponse]:
        """
        Get user by email

        Args:
            email: User email

        Returns:
            UserResponse: User data if found, None otherwise
        """
        try:
            normalized_email = validate_email(email)
        except ValidationError:
            return None

        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, email, created_at, updated_at
                FROM users
                WHERE email = %s
                """,
                (normalized_email,)
            )
            user_record = cursor.fetchone()

        if not user_record:
            return None

        return UserResponse(
            id=user_record['id'],
            email=user_record['email'],
            created_at=user_record['created_at'],
            updated_at=user_record['updated_at']
        )
