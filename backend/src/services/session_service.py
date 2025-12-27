"""
Session Service - Business logic for session management
"""

import secrets
from datetime import datetime
from typing import Optional, List
from uuid import UUID

from ..config.auth_config import AuthConfig
from ..database.connection import get_db_pool
from ..models.session import Session, SessionCreate, SessionResponse


class SessionService:
    """Service for session creation, validation, and management"""

    @staticmethod
    def generate_session_token() -> str:
        """
        Generate cryptographically secure random session token

        Returns:
            str: 64-character hexadecimal token (256 bits of randomness)
        """
        return secrets.token_hex(32)  # 32 bytes = 256 bits = 64 hex characters

    @staticmethod
    def create_session(user_id: UUID, remember_me: bool = False, session_type: str = "legacy", provider_id: str = "credentials") -> Session:
        """
        Create a new session for a user

        Args:
            user_id: User UUID
            remember_me: If True, session expires in 30 days; otherwise 24 hours
            session_type: Type of session (legacy or betterauth)
            provider_id: Authentication provider ID (credentials, google, etc.)

        Returns:
            Session: Created session data

        Raises:
            Exception: If database operation fails
        """
        # Generate secure random token
        token = SessionService.generate_session_token()

        # Calculate expiration based on remember_me flag
        expires_at = AuthConfig.get_session_expiration(remember_me)

        # Insert session into database
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                INSERT INTO sessions (user_id, token, expires_at, session_type, provider_id, created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s, NOW(), NOW())
                RETURNING id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
                """,
                (str(user_id), token, expires_at, session_type, provider_id)
            )
            session_record = cursor.fetchone()

        if not session_record:
            raise Exception("Failed to create session")

        return Session(
            id=session_record['id'],
            user_id=session_record['user_id'],
            token=session_record['token'],
            expires_at=session_record['expires_at'],
            session_type=session_record['session_type'],
            provider_id=session_record['provider_id'],
            created_at=session_record['created_at'],
            updated_at=session_record['updated_at']
        )

    @staticmethod
    def get_current_session(token: str) -> Optional[Session]:
        """
        Get session by token and validate expiration

        Args:
            token: Session token

        Returns:
            Session: Session data if valid and not expired, None otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
                FROM sessions
                WHERE token = %s
                """,
                (token,)
            )
            session_record = cursor.fetchone()

        if not session_record:
            return None

        # Check if session is expired
        if session_record['expires_at'] < datetime.utcnow():
            # Automatically delete expired session
            SessionService.invalidate_session(token)
            return None

        # Update session activity timestamp
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                UPDATE sessions
                SET updated_at = NOW()
                WHERE token = %s
                """,
                (token,)
            )

        return Session(
            id=session_record['id'],
            user_id=session_record['user_id'],
            token=session_record['token'],
            expires_at=session_record['expires_at'],
            session_type=session_record['session_type'],
            provider_id=session_record['provider_id'],
            created_at=session_record['created_at'],
            updated_at=datetime.utcnow()  # Use updated timestamp
        )

    @staticmethod
    def invalidate_session(token: str) -> None:
        """
        Invalidate (delete) a session by token

        Args:
            token: Session token to invalidate
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "DELETE FROM sessions WHERE token = %s",
                (token,)
            )

    @staticmethod
    def invalidate_user_sessions(user_id: UUID) -> int:
        """
        Invalidate all sessions for a user (useful for logout from all devices)

        Args:
            user_id: User UUID

        Returns:
            int: Number of sessions invalidated
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "DELETE FROM sessions WHERE user_id = %s RETURNING id",
                (str(user_id),)
            )
            deleted_sessions = cursor.fetchall()
            return len(deleted_sessions)

    @staticmethod
    def get_user_sessions(user_id: UUID) -> List[SessionResponse]:
        """
        Get all active sessions for a user

        Args:
            user_id: User UUID

        Returns:
            List[SessionResponse]: List of active sessions (excluding tokens)
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, user_id, expires_at, session_type, provider_id, created_at
                FROM sessions
                WHERE user_id = %s AND expires_at > NOW()
                ORDER BY created_at DESC
                """,
                (str(user_id),)
            )
            session_records = cursor.fetchall()

        return [
            SessionResponse(
                id=record['id'],
                user_id=record['user_id'],
                expires_at=record['expires_at'],
                session_type=record['session_type'],
                provider_id=record['provider_id'],
                created_at=record['created_at']
            )
            for record in session_records
        ]

    @staticmethod
    def find_session_by_token(token: str) -> Optional[Session]:
        """
        Find session by token - specifically for BetterAuth compatibility

        Args:
            token: Session token

        Returns:
            Session: Session data if found, None otherwise
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                """
                SELECT id, user_id, token, expires_at, session_type, provider_id, created_at, updated_at
                FROM sessions
                WHERE token = %s
                """,
                (token,)
            )
            session_record = cursor.fetchone()

        if not session_record:
            return None

        # Check if session is expired
        if session_record['expires_at'] < datetime.utcnow():
            # Automatically delete expired session
            SessionService.invalidate_session(token)
            return None

        return Session(
            id=session_record['id'],
            user_id=session_record['user_id'],
            token=session_record['token'],
            expires_at=session_record['expires_at'],
            session_type=session_record['session_type'],
            provider_id=session_record['provider_id'],
            created_at=session_record['created_at'],
            updated_at=session_record['updated_at']
        )

    @staticmethod
    def cleanup_expired_sessions() -> int:
        """
        Delete all expired sessions from database
        (Should be run periodically, e.g., via cron job or background task)

        Returns:
            int: Number of sessions deleted
        """
        db_pool = get_db_pool()
        with db_pool.get_cursor() as cursor:
            cursor.execute(
                "DELETE FROM sessions WHERE expires_at < NOW() RETURNING id"
            )
            deleted_sessions = cursor.fetchall()
            return len(deleted_sessions)
