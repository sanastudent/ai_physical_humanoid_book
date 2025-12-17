-- Migration 002: Create sessions table
-- Purpose: Active user sessions for BetterAuth-compatible backend
-- Created: 2025-12-11

-- Create sessions table
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL,
    token VARCHAR(512) NOT NULL UNIQUE,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),

    -- Foreign key constraint with CASCADE delete
    CONSTRAINT fk_sessions_user_id FOREIGN KEY (user_id)
        REFERENCES users(id) ON DELETE CASCADE
);

-- Create indexes for performance
CREATE UNIQUE INDEX IF NOT EXISTS idx_sessions_token ON sessions(token);
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON sessions(expires_at);

-- Add comments for documentation
COMMENT ON TABLE sessions IS 'Active user sessions managed by BetterAuth-compatible backend';
COMMENT ON COLUMN sessions.id IS 'Unique session identifier';
COMMENT ON COLUMN sessions.user_id IS 'Associated user (CASCADE delete on user deletion)';
COMMENT ON COLUMN sessions.token IS 'Session token stored in HTTP-only cookie';
COMMENT ON COLUMN sessions.expires_at IS 'Session expiration time';
COMMENT ON COLUMN sessions.created_at IS 'Session creation timestamp';
COMMENT ON COLUMN sessions.updated_at IS 'Last session activity timestamp';

-- Add constraint to ensure expires_at is in the future at creation
-- Note: This is a soft constraint, actual validation happens in application code
