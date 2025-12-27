"""
BetterAuth Backend Adapter
Implements BetterAuth-compatible adapter for Neon Postgres database
"""
from typing import Dict, Any, Optional, List
from datetime import datetime
from uuid import UUID
import secrets

from ..database.connection import get_db_pool
from ..config.auth_config import hash_password, verify_password


class BetterAuthAdapter:
    """
    BetterAuth-compatible database adapter for Neon Postgres
    Implements the required methods for user and session management
    """

    def __init__(self):
        """Initialize the adapter with database connection"""
        self.db_pool = get_db_pool()

    async def create_user(self, user_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a new user in the database

        Args:
            user_data: Dictionary containing user information
                     Expected keys: email, password, name (optional)

        Returns:
            Dictionary with created user data
        """
        # Validate required fields
        if 'email' not in user_data or 'password' not in user_data:
            raise ValueError("Email and password are required")

        email = user_data['email'].strip().lower()
        password = user_data['password']
        name = user_data.get('name')

        # Hash the password
        password_hash = hash_password(password)

        # Check if user already exists
        existing_user = await self.find_user_by_email(email)
        if existing_user:
            raise ValueError("Email already registered")

        # Generate user ID and create user
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                INSERT INTO users (
                    email,
                    password_hash,
                    created_at,
                    updated_at,
                    email_verified,
                    email_verified_at
                ) VALUES (
                    %(email)s,
                    %(password_hash)s,
                    NOW(),
                    NOW(),
                    %(email_verified)s,
                    CASE WHEN %(email_verified)s THEN NOW() ELSE NULL END
                )
                RETURNING id, email, created_at, updated_at, email_verified, email_verified_at
            """, {
                'email': email,
                'password_hash': password_hash,
                'email_verified': False  # New users need to verify email
            })

            user_record = cursor.fetchone()

        if not user_record:
            raise Exception("Failed to create user")

        return {
            'id': user_record['id'],
            'email': user_record['email'],
            'email_verified': user_record['email_verified'],
            'email_verified_at': user_record['email_verified_at'],
            'created_at': user_record['created_at'],
            'updated_at': user_record['updated_at'],
            'name': name
        }

    async def find_user_by_email(self, email: str) -> Optional[Dict[str, Any]]:
        """
        Find a user by email address

        Args:
            email: Email address to search for

        Returns:
            User dictionary if found, None otherwise
        """
        email = email.strip().lower()

        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                SELECT
                    id,
                    email,
                    password_hash,
                    created_at,
                    updated_at,
                    email_verified,
                    email_verified_at
                FROM users
                WHERE email = %s
            """, (email,))

            user_record = cursor.fetchone()

        if not user_record:
            return None

        return {
            'id': user_record['id'],
            'email': user_record['email'],
            'password_hash': user_record['password_hash'],
            'email_verified': user_record['email_verified'],
            'email_verified_at': user_record['email_verified_at'],
            'created_at': user_record['created_at'],
            'updated_at': user_record['updated_at']
        }

    async def find_user_by_id(self, user_id: str) -> Optional[Dict[str, Any]]:
        """
        Find a user by ID

        Args:
            user_id: User UUID to search for

        Returns:
            User dictionary if found, None otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                SELECT
                    id,
                    email,
                    created_at,
                    updated_at,
                    email_verified,
                    email_verified_at
                FROM users
                WHERE id = %s
            """, (user_id,))

            user_record = cursor.fetchone()

        if not user_record:
            return None

        return {
            'id': user_record['id'],
            'email': user_record['email'],
            'email_verified': user_record['email_verified'],
            'email_verified_at': user_record['email_verified_at'],
            'created_at': user_record['created_at'],
            'updated_at': user_record['updated_at']
        }

    async def create_session(self, session_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create a new session in the database

        Args:
            session_data: Dictionary containing session information
                         Expected keys: user_id, expires_at, session_type (optional), provider_id (optional)

        Returns:
            Dictionary with created session data
        """
        user_id = session_data['user_id']
        expires_at = session_data.get('expires_at')
        session_type = session_data.get('session_type', 'betterauth')
        provider_id = session_data.get('provider_id', 'credentials')

        # Generate secure session token
        session_token = secrets.token_urlsafe(32)  # 256-bit token

        # Set default expiration if not provided (24 hours)
        if not expires_at:
            from datetime import timedelta
            expires_at = datetime.utcnow() + timedelta(hours=24)

        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                INSERT INTO sessions (
                    user_id,
                    token,
                    expires_at,
                    session_type,
                    provider_id,
                    created_at,
                    updated_at
                ) VALUES (
                    %(user_id)s,
                    %(token)s,
                    %(expires_at)s,
                    %(session_type)s,
                    %(provider_id)s,
                    NOW(),
                    NOW()
                )
                RETURNING id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
            """, {
                'user_id': user_id,
                'token': session_token,
                'expires_at': expires_at,
                'session_type': session_type,
                'provider_id': provider_id
            })

            session_record = cursor.fetchone()

        if not session_record:
            raise Exception("Failed to create session")

        return {
            'id': session_record['id'],
            'user_id': session_record['user_id'],
            'token': session_record['token'],
            'expires_at': session_record['expires_at'],
            'session_type': session_record['session_type'],
            'provider_id': session_record['provider_id'],
            'created_at': session_record['created_at'],
            'updated_at': session_record['updated_at']
        }

    async def find_session_by_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Find a session by token

        Args:
            token: Session token to search for

        Returns:
            Session dictionary if found and not expired, None otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                SELECT
                    id,
                    user_id,
                    token,
                    expires_at,
                    session_type,
                    provider_id,
                    created_at,
                    updated_at
                FROM sessions
                WHERE token = %s AND expires_at > NOW()
            """, (token,))

            session_record = cursor.fetchone()

        if not session_record:
            return None

        return {
            'id': session_record['id'],
            'user_id': session_record['user_id'],
            'token': session_record['token'],
            'expires_at': session_record['expires_at'],
            'session_type': session_record['session_type'],
            'provider_id': session_record['provider_id'],
            'created_at': session_record['created_at'],
            'updated_at': session_record['updated_at']
        }

    async def invalidate_session(self, token: str) -> bool:
        """
        Invalidate a session by deleting it

        Args:
            token: Session token to invalidate

        Returns:
            True if session was deleted, False if not found
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                DELETE FROM sessions
                WHERE token = %s
                RETURNING id
            """, (token,))

            result = cursor.fetchone()

        return result is not None

    async def invalidate_user_sessions(self, user_id: str) -> int:
        """
        Invalidate all sessions for a user

        Args:
            user_id: User ID to invalidate sessions for

        Returns:
            Number of sessions invalidated
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute("""
                DELETE FROM sessions
                WHERE user_id = %s
                RETURNING id
            """, (user_id,))

            results = cursor.fetchall()

        return len(results) if results else 0

    async def update_user(self, user_id: str, update_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Update user information

        Args:
            user_id: User ID to update
            update_data: Dictionary with fields to update

        Returns:
            Updated user dictionary or None if user not found
        """
        # Build dynamic update query
        allowed_fields = {'email', 'email_verified', 'email_verified_at'}
        update_fields = {k: v for k, v in update_data.items() if k in allowed_fields}

        if not update_fields:
            raise ValueError("No valid fields to update")

        # Build SET clause dynamically
        set_parts = []
        params = {'user_id': user_id}

        for field, value in update_fields.items():
            set_parts.append(f"{field} = %({field})s")
            params[field] = value

        set_clause = ", ".join(set_parts)

        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(f"""
                UPDATE users
                SET {set_clause}, updated_at = NOW()
                WHERE id = %(user_id)s
                RETURNING id, email, email_verified, email_verified_at, created_at, updated_at
            """, params)

            user_record = cursor.fetchone()

        if not user_record:
            return None

        return {
            'id': user_record['id'],
            'email': user_record['email'],
            'email_verified': user_record['email_verified'],
            'email_verified_at': user_record['email_verified_at'],
            'created_at': user_record['created_at'],
            'updated_at': user_record['updated_at']
        }


# Global adapter instance
adapter_instance = BetterAuthAdapter()


# Convenience functions for use in routes
async def get_user_by_email(email: str) -> Optional[Dict[str, Any]]:
    """Get user by email using the adapter"""
    return await adapter_instance.find_user_by_email(email)


async def get_user_by_id(user_id: str) -> Optional[Dict[str, Any]]:
    """Get user by ID using the adapter"""
    return await adapter_instance.find_user_by_id(user_id)


async def create_new_user(user_data: Dict[str, Any]) -> Dict[str, Any]:
    """Create a new user using the adapter"""
    return await adapter_instance.create_user(user_data)


async def create_new_session(session_data: Dict[str, Any]) -> Dict[str, Any]:
    """Create a new session using the adapter"""
    return await adapter_instance.create_session(session_data)


async def get_session_by_token(token: str) -> Optional[Dict[str, Any]]:
    """Get session by token using the adapter"""
    return await adapter_instance.find_session_by_token(token)


async def remove_session(token: str) -> bool:
    """Remove session by token using the adapter"""
    return await adapter_instance.invalidate_session(token)


async def remove_user_sessions(user_id: str) -> int:
    """Remove all sessions for a user using the adapter"""
    return await adapter_instance.invalidate_user_sessions(user_id)


async def update_user_info(user_id: str, update_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Update user information using the adapter"""
    return await adapter_instance.update_user(user_id, update_data)