"""
Database Connection Pool Manager for Neon DB with Retry Logic
Handles PostgreSQL connection pooling with automatic reconnection for Neon Serverless
"""

import psycopg2
from psycopg2 import pool, OperationalError
from psycopg2.extras import RealDictCursor
from typing import Optional
from contextlib import contextmanager
import time

from ..config.auth_config import AuthConfig


class DatabaseConnectionPool:
    """
    PostgreSQL connection pool manager for Neon DB with retry logic

    Handles Neon Serverless connection timeouts and reconnection
    """

    def __init__(
        self,
        minconn: int = 1,
        maxconn: int = 20,
        database_url: Optional[str] = None,
        max_retries: int = 3,
        retry_delay: float = 0.5
    ):
        """
        Initialize database connection pool

        Args:
            minconn: Minimum number of idle connections
            maxconn: Maximum number of connections
            database_url: PostgreSQL connection string (uses AuthConfig.NEON_DB_URL if None)
            max_retries: Maximum number of retry attempts for failed operations
            retry_delay: Delay between retries in seconds (exponential backoff)
        """
        self.database_url = database_url or AuthConfig.NEON_DB_URL
        self.minconn = minconn
        self.maxconn = maxconn
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.pool = None

        if not self.database_url:
            print("WARNING: Database URL is not set (set NEON_DB_URL in .env)")
            print("WARNING: Database connection pool will not be initialized until database_url is provided")

    def _create_pool(self):
        """Create connection pool with retry logic"""
        retries = 0
        last_error = None

        while retries < self.max_retries:
            try:
                self.pool = psycopg2.pool.SimpleConnectionPool(
                    self.minconn,
                    self.maxconn,
                    self.database_url
                )
                print(f"[OK] Database connection pool initialized (min={self.minconn}, max={self.maxconn})")
                return
            except (OperationalError, psycopg2.Error) as e:
                last_error = e
                retries += 1
                if retries < self.max_retries:
                    delay = self.retry_delay * (2 ** (retries - 1))  # Exponential backoff
                    print(f"WARNING: Database connection failed (attempt {retries}/{self.max_retries}), retrying in {delay}s...")
                    time.sleep(delay)

        raise ConnectionError(f"Failed to create database connection pool after {self.max_retries} attempts: {last_error}")

    def _ensure_pool(self):
        """Ensure connection pool is initialized (lazy initialization with retry)"""
        if self.pool is None:
            if not self.database_url:
                raise ValueError("Database URL is required (set NEON_DB_URL in .env)")
            self._create_pool()

    def _validate_connection(self, conn) -> bool:
        """
        Validate that connection is alive and usable

        Args:
            conn: psycopg2 connection object

        Returns:
            bool: True if connection is valid, False otherwise
        """
        if conn is None or conn.closed:
            return False

        try:
            # Test connection with a simple query
            cursor = conn.cursor()
            cursor.execute("SELECT 1")
            cursor.close()
            return True
        except (OperationalError, psycopg2.Error):
            return False

    def _get_valid_connection(self):
        """
        Get a valid connection from pool with retry logic

        Returns:
            Valid psycopg2 connection

        Raises:
            ConnectionError: If unable to get valid connection after retries
        """
        self._ensure_pool()

        retries = 0
        while retries < self.max_retries:
            try:
                conn = self.pool.getconn()

                if conn is None:
                    raise ConnectionError("Pool returned None connection")

                # Validate connection
                if self._validate_connection(conn):
                    return conn

                # Connection is invalid, put it back and mark for closure
                print(f"WARNING: Connection is invalid (closed={conn.closed}), getting new connection...")
                self.pool.putconn(conn, close=True)

                # Get a new connection
                conn = self.pool.getconn()

                if conn and self._validate_connection(conn):
                    return conn

                # If still invalid, recreate the entire pool
                print("WARNING: Connection pool may be stale, recreating pool...")
                self.close_all_connections()
                self.pool = None
                self._ensure_pool()

                conn = self.pool.getconn()
                if conn and self._validate_connection(conn):
                    return conn

            except (OperationalError, psycopg2.Error) as e:
                retries += 1
                if retries < self.max_retries:
                    delay = self.retry_delay * (2 ** (retries - 1))
                    print(f"WARNING: Failed to get connection (attempt {retries}/{self.max_retries}): {e}")
                    print(f"   Retrying in {delay}s...")
                    time.sleep(delay)

                    # Try recreating pool on connection errors
                    if self.pool:
                        try:
                            self.close_all_connections()
                        except:
                            pass
                        self.pool = None

        raise ConnectionError(f"Failed to get valid connection after {self.max_retries} attempts")

    @contextmanager
    def get_connection(self):
        """
        Get database connection from pool (context manager) with retry logic

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
            conn = self._get_valid_connection()
            yield conn
        except Exception as e:
            if conn and not conn.closed:
                try:
                    conn.rollback()
                except:
                    pass
            raise e
        finally:
            if conn:
                try:
                    # Return connection to pool
                    if not conn.closed:
                        self.pool.putconn(conn)
                    else:
                        # If connection was closed, mark it for cleanup
                        self.pool.putconn(conn, close=True)
                except:
                    # If putconn fails, try to close the connection directly
                    try:
                        if not conn.closed:
                            conn.close()
                    except:
                        pass

    @contextmanager
    def get_cursor(self, cursor_factory=None):
        """
        Get database cursor from pool (context manager) with retry logic

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
                try:
                    cursor.close()
                except:
                    pass

    def close_all_connections(self):
        """Close all connections in the pool"""
        if self.pool:
            try:
                self.pool.closeall()
                print("[OK] All database connections closed")
            except Exception as e:
                print(f"WARNING: Error closing connections: {e}")

    def __del__(self):
        """Cleanup on object destruction"""
        try:
            self.close_all_connections()
        except:
            pass


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
        _db_pool = DatabaseConnectionPool(
            minconn=1,  # Reduced from 5 for Neon Serverless (conserve connections)
            maxconn=10,  # Reduced from 20 for Neon Serverless
            max_retries=3,
            retry_delay=0.5
        )
    return _db_pool


def close_db_pool():
    """Close global database connection pool"""
    global _db_pool
    if _db_pool:
        _db_pool.close_all_connections()
        _db_pool = None
