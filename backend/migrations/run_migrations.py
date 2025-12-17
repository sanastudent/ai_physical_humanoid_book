"""
Migration Runner for BetterAuth Database Schema
Purpose: Execute all SQL migration files in order
Usage: python backend/migrations/run_migrations.py
"""

import os
import sys
from pathlib import Path
import psycopg2
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def get_db_connection():
    """Create database connection from NEON_DB_URL environment variable"""
    db_url = os.getenv('NEON_DB_URL')
    if not db_url:
        raise ValueError(
            "NEON_DB_URL not found in environment variables. "
            "Please add your Neon DB connection string to .env file"
        )
    return psycopg2.connect(db_url)

def run_migrations():
    """Execute all migration files in numerical order"""
    migrations_dir = Path(__file__).parent
    migration_files = sorted(migrations_dir.glob('*.sql'))

    if not migration_files:
        print("No migration files found!")
        return

    print(f"Found {len(migration_files)} migration files")

    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        for migration_file in migration_files:
            print(f"\nExecuting {migration_file.name}...")

            with open(migration_file, 'r', encoding='utf-8') as f:
                sql = f.read()

            try:
                cursor.execute(sql)
                conn.commit()
                print(f"[OK] {migration_file.name} completed successfully")
            except psycopg2.Error as e:
                print(f"[ERROR] Error in {migration_file.name}:")
                print(f"  {e}")
                conn.rollback()
                raise

        cursor.close()
        conn.close()

        print("\n" + "="*50)
        print("All migrations completed successfully!")
        print("="*50)

    except psycopg2.Error as e:
        print(f"\nDatabase connection error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    print("="*50)
    print("BetterAuth Database Migration Runner")
    print("="*50)
    run_migrations()
