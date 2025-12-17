-- Migration 006: Create triggers for automatic updated_at timestamp updates
-- Purpose: Automatically update updated_at column on row modifications
-- Created: 2025-12-11

-- Create function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Create triggers for each table

-- Trigger for users table
DROP TRIGGER IF EXISTS trigger_users_updated_at ON users;
CREATE TRIGGER trigger_users_updated_at
    BEFORE UPDATE ON users
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for sessions table
DROP TRIGGER IF EXISTS trigger_sessions_updated_at ON sessions;
CREATE TRIGGER trigger_sessions_updated_at
    BEFORE UPDATE ON sessions
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for software_background table
DROP TRIGGER IF EXISTS trigger_software_background_updated_at ON software_background;
CREATE TRIGGER trigger_software_background_updated_at
    BEFORE UPDATE ON software_background
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Trigger for hardware_background table
DROP TRIGGER IF EXISTS trigger_hardware_background_updated_at ON hardware_background;
CREATE TRIGGER trigger_hardware_background_updated_at
    BEFORE UPDATE ON hardware_background
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Add comments for documentation
COMMENT ON FUNCTION update_updated_at_column IS 'Automatically updates updated_at column to current timestamp on row update';
