"""
Database Connection Pool Manager for Neon DB
Handles PostgreSQL connection pooling for BetterAuth authentication system
"""

import psycopg2
from psycopg2 import pool
from psycopg2.extras import RealDictCursor
from typing import Optional
from contextlib import contextmanager

from ..config.auth_config import AuthConfig


class DatabaseConnectionPool:
    """
    PostgreSQL connection pool manager for Neon DB

    Provides connection pooling with min/max connections and context manager support
    """

    def __init__(
        self,
        minconn: int = 1,
        maxconn: int = 20,
        database_url: Optional[str] = None
    ):
        """
        Initialize database connection pool

        Args:
            minconn: Minimum number of idle connections
            maxconn: Maximum number of connections
            database_url: PostgreSQL connection string (uses AuthConfig.NEON_DB_URL if None)
        """
        self.database_url = database_url or AuthConfig.NEON_DB_URL

        if not self.database_url:
            raise ValueError("Database URL is required (set NEON_DB_URL in .env)")

        try:
            self.pool = psycopg2.pool.SimpleConnectionPool(
                minconn,
                maxconn,
                self.database_url
            )
            print(f"Database connection pool initialized (min={minconn}, max={maxconn})")
        except psycopg2.Error as e:
            raise ConnectionError(f"Failed to create database connection pool: {e}")

    @contextmanager
    def get_connection(self):
        """
        Get database connection from pool (context manager)

        Usage:
            with db_pool.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT * FROM users")
                result = cursor.fetchall()

        Yields:
            connection: PostgreSQL connection from pool
        """
        conn = None
        try:
            conn = self.pool.getconn()
            if conn is None:
                raise ConnectionError("Failed to get connection from pool")
            yield conn
        except Exception as e:
            if conn:
                conn.rollback()
            raise e
        finally:
            if conn:
                self.pool.putconn(conn)

    @contextmanager
    def get_cursor(self, cursor_factory=None):
        """
        Get database cursor from pool (context manager)

        Usage:
            with db_pool.get_cursor() as cursor:
                cursor.execute("SELECT * FROM users WHERE email = %s", (email,))
                user = cursor.fetchone()

        Args:
            cursor_factory: Optional cursor factory (default: RealDictCursor for dict results)

        Yields:
            cursor: PostgreSQL cursor
        """
        if cursor_factory is None:
            cursor_factory = RealDictCursor

        with self.get_connection() as conn:
            cursor = conn.cursor(cursor_factory=cursor_factory)
            try:
                yield cursor
                conn.commit()
            except Exception as e:
                conn.rollback()
                raise e
            finally:
                cursor.close()

    def close_all_connections(self):
        """Close all connections in the pool"""
        if self.pool:
            self.pool.closeall()
            print("All database connections closed")

    def __del__(self):
        """Cleanup on object destruction"""
        self.close_all_connections()


# Global connection pool instance
_db_pool: Optional[DatabaseConnectionPool] = None


def get_db_pool() -> DatabaseConnectionPool:
    """
    Get global database connection pool instance (singleton)

    Returns:
        DatabaseConnectionPool: Global connection pool
    """
    global _db_pool
    if _db_pool is None:
        _db_pool = DatabaseConnectionPool(minconn=5, maxconn=20)
    return _db_pool


def close_db_pool():
    """Close global database connection pool"""
    global _db_pool
    if _db_pool:
        _db_pool.close_all_connections()
        _db_pool = None
