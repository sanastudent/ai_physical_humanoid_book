# Database Migrations for BetterAuth Authentication System

## Overview

This directory contains SQL migration files for setting up the BetterAuth authentication database schema in Neon DB (PostgreSQL).

## Migration Files

1. **001_create_users.sql** - Core user authentication table
2. **002_create_sessions.sql** - User session management
3. **003_create_software_background.sql** - Software development background data
4. **004_create_hardware_background.sql** - Hardware development background data
5. **005_create_indexes.sql** - Additional performance indexes
6. **006_create_triggers.sql** - Auto-update triggers for updated_at columns

## Running Migrations

### Prerequisites

1. Set up your Neon DB instance at https://console.neon.tech
2. Add your connection string to `.env` file:
   ```
   NEON_DB_URL=postgresql://username:password@your-neon-host.neon.tech/dbname?sslmode=require
   ```

### Execute All Migrations

```bash
# From the repository root
python backend/migrations/run_migrations.py
```

The script will:
- Load your database connection from `.env`
- Execute all `.sql` files in numerical order
- Report success/failure for each migration
- Automatically commit or rollback based on results

### Manual Execution (Alternative)

You can also run migrations manually using `psql`:

```bash
psql $NEON_DB_URL -f backend/migrations/001_create_users.sql
psql $NEON_DB_URL -f backend/migrations/002_create_sessions.sql
psql $NEON_DB_URL -f backend/migrations/003_create_software_background.sql
psql $NEON_DB_URL -f backend/migrations/004_create_hardware_background.sql
psql $NEON_DB_URL -f backend/migrations/005_create_indexes.sql
psql $NEON_DB_URL -f backend/migrations/006_create_triggers.sql
```

## Schema Diagram

```
┌──────────────────┐
│     users        │
│──────────────────│
│ id (PK)          │◄─────┐
│ email (UNIQUE)   │      │
│ password_hash    │      │
│ created_at       │      │
│ updated_at       │      │
└──────────────────┘      │
                          │ FK (user_id)
                          │
┌──────────────────────────┼─────────────────────────┐
│                          │                         │
│                          │                         │
▼                          ▼                         ▼
┌────────────────────┐ ┌─────────────────────────┐ ┌──────────────────┐
│ software_background│ │  hardware_background    │ │    sessions      │
│────────────────────│ │─────────────────────────│ │──────────────────│
│ id (PK)            │ │ id (PK)                 │ │ id (PK)          │
│ user_id (FK, UQ)   │ │ user_id (FK, UQ)        │ │ user_id (FK)     │
│ experience_level   │ │ experience_level        │ │ token (UNIQUE)   │
│ preferred_languages│ │ preferred_platforms     │ │ expires_at       │
│ preferred_frameworks│ │ device_types           │ │ created_at       │
│ created_at         │ │ created_at              │ │ updated_at       │
│ updated_at         │ │ updated_at              │ └──────────────────┘
└────────────────────┘ └─────────────────────────┘
```

## Verification

After running migrations, verify the schema:

```sql
-- List all tables
\dt

-- Describe tables
\d users
\d sessions
\d software_background
\d hardware_background

-- Check indexes
\di

-- Verify triggers
SELECT tgname, tgrelid::regclass FROM pg_trigger;
```

## Rollback

To rollback all migrations (⚠️ **WARNING: This will delete all data!**):

```sql
DROP TABLE IF EXISTS hardware_background CASCADE;
DROP TABLE IF EXISTS software_background CASCADE;
DROP TABLE IF EXISTS sessions CASCADE;
DROP TABLE IF EXISTS users CASCADE;
DROP FUNCTION IF EXISTS update_updated_at_column CASCADE;
```

## Notes

- All foreign keys use `ON DELETE CASCADE` to maintain referential integrity
- The `updated_at` column is automatically updated via triggers
- Array fields (languages, frameworks, platforms, devices) use PostgreSQL TEXT[] type with GIN indexes for efficient searching
- Sessions table includes indexes for fast token lookups and expiration cleanup
