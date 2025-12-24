-- Migration 008: Add BetterAuth compatibility fields to sessions table
-- Purpose: Add session_type and provider_id columns for BetterAuth compliance
-- Created: 2025-12-22

-- Add session_type column (default 'legacy' for existing sessions)
ALTER TABLE sessions
ADD COLUMN IF NOT EXISTS session_type VARCHAR(20) NOT NULL DEFAULT 'legacy';

-- Add provider_id column (null for legacy sessions)
ALTER TABLE sessions
ADD COLUMN IF NOT EXISTS provider_id VARCHAR(50);

-- Add index on session_type for migration queries
CREATE INDEX IF NOT EXISTS idx_sessions_session_type ON sessions(session_type);

-- Add comments for documentation
COMMENT ON COLUMN sessions.session_type IS 'Session type: betterauth or legacy (for migration tracking)';
COMMENT ON COLUMN sessions.provider_id IS 'Authentication provider ID (for BetterAuth compatibility)';
