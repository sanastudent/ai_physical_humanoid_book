-- Migration 007: Add BetterAuth compatibility fields to users table
-- Purpose: Add email_verified and email_verified_at columns for BetterAuth compliance
-- Created: 2025-12-22

-- Add email_verified column (default false for existing users)
ALTER TABLE users
ADD COLUMN IF NOT EXISTS email_verified BOOLEAN DEFAULT FALSE;

-- Add email_verified_at column (null until verified)
ALTER TABLE users
ADD COLUMN IF NOT EXISTS email_verified_at TIMESTAMP;

-- Add index on email_verified for verification queries
CREATE INDEX IF NOT EXISTS idx_users_email_verified ON users(email_verified);

-- Add comments for documentation
COMMENT ON COLUMN users.email_verified IS 'Whether email has been verified (BetterAuth compatibility)';
COMMENT ON COLUMN users.email_verified_at IS 'Timestamp when email was verified (BetterAuth compatibility)';
