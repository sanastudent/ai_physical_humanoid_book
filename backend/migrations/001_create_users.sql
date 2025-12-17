-- Migration 001: Create users table
-- Purpose: Core user authentication table for BetterAuth-compatible backend
-- Created: 2025-12-11

-- Enable UUID extension if not already enabled
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Create users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) NOT NULL UNIQUE,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Create unique index on email for fast lookups
CREATE UNIQUE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Add comment for documentation
COMMENT ON TABLE users IS 'Core user authentication table managed by BetterAuth-compatible backend';
COMMENT ON COLUMN users.id IS 'Unique user identifier';
COMMENT ON COLUMN users.email IS 'User email address (login credential, must be unique)';
COMMENT ON COLUMN users.password_hash IS 'bcrypt hashed password (never store plaintext)';
COMMENT ON COLUMN users.created_at IS 'Account creation timestamp';
COMMENT ON COLUMN users.updated_at IS 'Last account update timestamp';
