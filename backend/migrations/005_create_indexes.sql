-- Migration 005: Create additional indexes for performance optimization
-- Purpose: Ensure all critical queries are optimized with proper indexes
-- Created: 2025-12-11

-- Note: Primary indexes are already created in previous migrations
-- This file serves as a placeholder for any additional indexes needed in the future

-- Verify existing indexes are in place (these are created in previous migrations):
-- users: idx_users_email (UNIQUE)
-- sessions: idx_sessions_token (UNIQUE), idx_sessions_user_id, idx_sessions_expires_at
-- software_background: idx_software_background_user_id (UNIQUE), idx_software_background_languages (GIN), idx_software_background_frameworks (GIN)
-- hardware_background: idx_hardware_background_user_id (UNIQUE), idx_hardware_background_platforms (GIN), idx_hardware_background_devices (GIN)

-- Additional indexes can be added here as needed based on query patterns
-- Example:
-- CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);

COMMENT ON SCHEMA public IS 'All critical indexes for authentication and background data are created in migrations 001-004';
