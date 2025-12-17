-- Migration 004: Create hardware_background table
-- Purpose: User's hardware development background and platform preferences
-- Created: 2025-12-11

-- Create hardware_background table
CREATE TABLE IF NOT EXISTS hardware_background (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL UNIQUE,
    experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
    preferred_platforms TEXT[] NOT NULL DEFAULT '{}',
    device_types TEXT[] NOT NULL DEFAULT '{}',
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),

    -- Foreign key constraint with CASCADE delete
    CONSTRAINT fk_hardware_background_user_id FOREIGN KEY (user_id)
        REFERENCES users(id) ON DELETE CASCADE
);

-- Create indexes for performance
CREATE UNIQUE INDEX IF NOT EXISTS idx_hardware_background_user_id ON hardware_background(user_id);
-- GIN indexes for array searches
CREATE INDEX IF NOT EXISTS idx_hardware_background_platforms ON hardware_background USING GIN(preferred_platforms);
CREATE INDEX IF NOT EXISTS idx_hardware_background_devices ON hardware_background USING GIN(device_types);

-- Add comments for documentation
COMMENT ON TABLE hardware_background IS 'User hardware development background and platform preferences';
COMMENT ON COLUMN hardware_background.id IS 'Unique background record ID';
COMMENT ON COLUMN hardware_background.user_id IS 'Associated user (one-to-one, CASCADE delete)';
COMMENT ON COLUMN hardware_background.experience_level IS 'Hardware development experience (beginner/intermediate/advanced)';
COMMENT ON COLUMN hardware_background.preferred_platforms IS 'Development platforms (e.g., desktop, mobile, embedded)';
COMMENT ON COLUMN hardware_background.device_types IS 'Familiar device types (e.g., laptop, smartphone, raspberry-pi)';
COMMENT ON COLUMN hardware_background.created_at IS 'Record creation timestamp';
COMMENT ON COLUMN hardware_background.updated_at IS 'Last update timestamp';
