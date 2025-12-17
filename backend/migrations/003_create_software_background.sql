-- Migration 003: Create software_background table
-- Purpose: User's software development background and preferences
-- Created: 2025-12-11

-- Create software_background table
CREATE TABLE IF NOT EXISTS software_background (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL UNIQUE,
    experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
    preferred_languages TEXT[] NOT NULL DEFAULT '{}',
    preferred_frameworks TEXT[] NOT NULL DEFAULT '{}',
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW(),

    -- Foreign key constraint with CASCADE delete
    CONSTRAINT fk_software_background_user_id FOREIGN KEY (user_id)
        REFERENCES users(id) ON DELETE CASCADE
);

-- Create indexes for performance
CREATE UNIQUE INDEX IF NOT EXISTS idx_software_background_user_id ON software_background(user_id);
-- GIN indexes for array searches (enable fast queries like "all Python developers")
CREATE INDEX IF NOT EXISTS idx_software_background_languages ON software_background USING GIN(preferred_languages);
CREATE INDEX IF NOT EXISTS idx_software_background_frameworks ON software_background USING GIN(preferred_frameworks);

-- Add comments for documentation
COMMENT ON TABLE software_background IS 'User software development background and preferences';
COMMENT ON COLUMN software_background.id IS 'Unique background record ID';
COMMENT ON COLUMN software_background.user_id IS 'Associated user (one-to-one, CASCADE delete)';
COMMENT ON COLUMN software_background.experience_level IS 'Software development experience (beginner/intermediate/advanced)';
COMMENT ON COLUMN software_background.preferred_languages IS 'Programming languages (e.g., Python, JavaScript, Go)';
COMMENT ON COLUMN software_background.preferred_frameworks IS 'Frameworks (e.g., React, FastAPI, Django)';
COMMENT ON COLUMN software_background.created_at IS 'Record creation timestamp';
COMMENT ON COLUMN software_background.updated_at IS 'Last update timestamp';
