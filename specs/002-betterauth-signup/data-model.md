# Data Model: BetterAuth User Authentication & Background Collection

**Feature**: 002-betterauth-signup
**Date**: 2025-12-11
**Database**: Neon DB (PostgreSQL)

## Entity Relationship Diagram

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
                          │
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
│ user_id (FK)       │ │ user_id (FK)            │ │ user_id (FK)     │
│ experience_level   │ │ experience_level        │ │ token (UNIQUE)   │
│ preferred_languages│ │ preferred_platforms     │ │ expires_at       │
│ preferred_frameworks│ │ device_types           │ │ created_at       │
│ created_at         │ │ created_at              │ │ updated_at       │
│ updated_at         │ │ updated_at              │ └──────────────────┘
└────────────────────┘ └─────────────────────────┘
```

## Table Definitions

### users

**Purpose**: Core user authentication table managed by BetterAuth

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique user identifier |
| email | VARCHAR(255) | NOT NULL, UNIQUE | User email address (login credential) |
| password_hash | VARCHAR(255) | NOT NULL | bcrypt hashed password |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last account update timestamp |

**Indexes**:
- PRIMARY KEY on `id`
- UNIQUE INDEX on `email` (for login lookups)

**Constraints**:
- `email` must be valid email format (enforced by application layer)
- `password_hash` never null (enforced by BetterAuth)

**Validation Rules** (Application Layer):
- Email: RFC 5322 compliant format
- Password (before hashing): Min 8 chars, 1 uppercase, 1 lowercase, 1 number

---

### sessions

**Purpose**: Active user sessions managed by BetterAuth

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique session identifier |
| user_id | UUID | NOT NULL, FOREIGN KEY → users(id) ON DELETE CASCADE | Associated user |
| token | VARCHAR(512) | NOT NULL, UNIQUE | Session token stored in HTTP-only cookie |
| expires_at | TIMESTAMP | NOT NULL | Session expiration time |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last session activity |

**Indexes**:
- PRIMARY KEY on `id`
- UNIQUE INDEX on `token` (for session lookups)
- INDEX on `user_id` (for user session queries)
- INDEX on `expires_at` (for cleanup queries)

**Constraints**:
- `user_id` must reference existing user (CASCADE delete when user is deleted)
- `expires_at` must be future timestamp at creation

**Lifecycle**:
- Created on successful signin/signup
- Updated on each authenticated request (activity tracking)
- Deleted on signout or expiration
- Cleanup job removes expired sessions (cronjob or trigger)

---

### software_background

**Purpose**: User's software development background and preferences

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique background record ID |
| user_id | UUID | NOT NULL, UNIQUE, FOREIGN KEY → users(id) ON DELETE CASCADE | Associated user (one-to-one) |
| experience_level | VARCHAR(20) | NOT NULL, CHECK IN ('beginner', 'intermediate', 'advanced') | Software development experience |
| preferred_languages | TEXT[] | NOT NULL, DEFAULT '{}' | Programming languages (e.g., ['Python', 'JavaScript']) |
| preferred_frameworks | TEXT[] | NOT NULL, DEFAULT '{}' | Frameworks (e.g., ['React', 'FastAPI']) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Record creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Indexes**:
- PRIMARY KEY on `id`
- UNIQUE INDEX on `user_id` (enforce one-to-one relationship)
- GIN INDEX on `preferred_languages` (for array searches)
- GIN INDEX on `preferred_frameworks` (for array searches)

**Constraints**:
- `user_id` must reference existing user (CASCADE delete)
- `experience_level` must be one of: beginner, intermediate, advanced
- Arrays can be empty but not null

**Validation Rules** (Application Layer):
- At least one language must be selected during signup
- Language/framework values must be from predefined list

**Example Row**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "experience_level": "intermediate",
  "preferred_languages": ["Python", "JavaScript", "Go"],
  "preferred_frameworks": ["FastAPI", "React", "Fiber"],
  "created_at": "2025-12-11T10:30:00Z",
  "updated_at": "2025-12-11T10:30:00Z"
}
```

---

### hardware_background

**Purpose**: User's hardware development background and platform preferences

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique background record ID |
| user_id | UUID | NOT NULL, UNIQUE, FOREIGN KEY → users(id) ON DELETE CASCADE | Associated user (one-to-one) |
| experience_level | VARCHAR(20) | NOT NULL, CHECK IN ('beginner', 'intermediate', 'advanced') | Hardware development experience |
| preferred_platforms | TEXT[] | NOT NULL, DEFAULT '{}' | Platforms (e.g., ['desktop', 'mobile', 'embedded']) |
| device_types | TEXT[] | NOT NULL, DEFAULT '{}' | Devices (e.g., ['Raspberry Pi', 'Arduino', 'iPhone']) |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Record creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Indexes**:
- PRIMARY KEY on `id`
- UNIQUE INDEX on `user_id` (enforce one-to-one relationship)
- GIN INDEX on `preferred_platforms` (for array searches)
- GIN INDEX on `device_types` (for array searches)

**Constraints**:
- `user_id` must reference existing user (CASCADE delete)
- `experience_level` must be one of: beginner, intermediate, advanced
- Arrays can be empty but not null

**Validation Rules** (Application Layer):
- At least one platform must be selected during signup
- Platform/device values must be from predefined list

**Example Row**:
```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "experience_level": "beginner",
  "preferred_platforms": ["desktop", "mobile"],
  "device_types": ["laptop", "smartphone"],
  "created_at": "2025-12-11T10:30:00Z",
  "updated_at": "2025-12-11T10:30:00Z"
}
```

---

## Database Migrations

### Migration 001: Create Users and Sessions (BetterAuth)
```sql
-- Generated by BetterAuth setup
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) NOT NULL UNIQUE,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_users_email ON users(email);

CREATE TABLE sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  token VARCHAR(512) NOT NULL UNIQUE,
  expires_at TIMESTAMP NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

### Migration 002: Create Background Tables
```sql
-- Software background table
CREATE TABLE software_background (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
  preferred_languages TEXT[] NOT NULL DEFAULT '{}',
  preferred_frameworks TEXT[] NOT NULL DEFAULT '{}',
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_software_background_user_id ON software_background(user_id);
CREATE INDEX idx_software_background_languages ON software_background USING GIN(preferred_languages);
CREATE INDEX idx_software_background_frameworks ON software_background USING GIN(preferred_frameworks);

-- Hardware background table
CREATE TABLE hardware_background (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
  preferred_platforms TEXT[] NOT NULL DEFAULT '{}',
  device_types TEXT[] NOT NULL DEFAULT '{}',
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_hardware_background_user_id ON hardware_background(user_id);
CREATE INDEX idx_hardware_background_platforms ON hardware_background USING GIN(preferred_platforms);
CREATE INDEX idx_hardware_background_devices ON hardware_background USING GIN(device_types);
```

### Migration 003: Triggers for Updated_At
```sql
-- Auto-update updated_at on row modification
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = NOW();
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_sessions_updated_at BEFORE UPDATE ON sessions
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_software_background_updated_at BEFORE UPDATE ON software_background
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_hardware_background_updated_at BEFORE UPDATE ON hardware_background
  FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

---

## Data Access Patterns

### Pattern 1: User Signup (Transaction)
```sql
BEGIN;
  -- Insert user (BetterAuth handles this)
  INSERT INTO users (email, password_hash)
  VALUES ('user@example.com', '$2b$10$...');

  -- Insert software background
  INSERT INTO software_background (user_id, experience_level, preferred_languages, preferred_frameworks)
  VALUES (
    (SELECT id FROM users WHERE email = 'user@example.com'),
    'intermediate',
    ARRAY['Python', 'JavaScript'],
    ARRAY['FastAPI', 'React']
  );

  -- Insert hardware background
  INSERT INTO hardware_background (user_id, experience_level, preferred_platforms, device_types)
  VALUES (
    (SELECT id FROM users WHERE email = 'user@example.com'),
    'beginner',
    ARRAY['desktop', 'mobile'],
    ARRAY['laptop', 'smartphone']
  );
COMMIT;
```

### Pattern 2: Fetch User with Background (JOIN)
```sql
SELECT
  u.id,
  u.email,
  u.created_at,
  sb.experience_level AS software_experience,
  sb.preferred_languages,
  sb.preferred_frameworks,
  hb.experience_level AS hardware_experience,
  hb.preferred_platforms,
  hb.device_types
FROM users u
LEFT JOIN software_background sb ON u.id = sb.user_id
LEFT JOIN hardware_background hb ON u.id = hb.user_id
WHERE u.id = $1;
```

### Pattern 3: Update Background Information
```sql
-- Update software background
UPDATE software_background
SET
  experience_level = 'advanced',
  preferred_languages = ARRAY['Python', 'Rust', 'Go'],
  preferred_frameworks = ARRAY['FastAPI', 'Axum', 'Gin']
WHERE user_id = $1;

-- Update hardware background
UPDATE hardware_background
SET
  experience_level = 'intermediate',
  preferred_platforms = ARRAY['desktop', 'embedded'],
  device_types = ARRAY['laptop', 'Raspberry Pi']
WHERE user_id = $1;
```

### Pattern 4: Session Cleanup (Periodic Job)
```sql
-- Delete expired sessions (run every hour)
DELETE FROM sessions
WHERE expires_at < NOW();
```

---

## Data Validation & Business Rules

### Signup Validation
1. **Email Uniqueness**: Check `users.email` doesn't exist before insert
2. **Password Strength**: Min 8 chars, 1 uppercase, 1 lowercase, 1 number (application layer)
3. **Background Completeness**: At least one language and one platform selected
4. **Transaction Safety**: All inserts in single transaction (rollback on any failure)

### Update Validation
1. **User Ownership**: Verify session user_id matches background user_id before update
2. **Experience Level**: Must be one of: beginner, intermediate, advanced
3. **Array Values**: Languages/frameworks/platforms must be from allowed list

### Session Validation
1. **Token Expiry**: Check `expires_at > NOW()` on every authenticated request
2. **Token Uniqueness**: Enforce via UNIQUE constraint on `sessions.token`
3. **Cascade Deletion**: Sessions auto-deleted when user is deleted

---

## Performance Considerations

**Expected Query Load**:
- Signup: ~10 TPS (low volume)
- Signin: ~50 TPS (moderate volume)
- Session validation: ~500 TPS (high volume, every authenticated request)
- Background fetch: ~100 TPS (personalization clicks)

**Index Strategy**:
- `users.email`: Frequent login lookups
- `sessions.token`: Every authenticated request
- `sessions.expires_at`: Cleanup queries
- GIN indexes on arrays: Personalization queries by language/platform

**Connection Pooling**:
- Neon DB supports up to 100 concurrent connections (serverless scaling)
- Backend API should use connection pool (max 20 connections)

---

## Security Considerations

**Sensitive Data**:
- `password_hash`: Never exposed via API, only stored
- `sessions.token`: Only sent via HTTP-only cookies, never in responses
- User background: Accessible only to authenticated user (ownership check)

**SQL Injection Prevention**:
- All queries use parameterized statements
- No string concatenation for user input
- ORM/query builder enforces this by default

**Data Privacy**:
- User can update own background anytime
- User deletion cascades to all related records (GDPR compliance)
- No cross-user data access (each user sees only own data)

---

## Rollback Strategy

If migration fails or data corruption occurs:

1. **Rollback Migration**: Use migration tool to revert to previous schema version
2. **Backup Restoration**: Neon DB provides point-in-time recovery
3. **Data Validation**: Run validation queries to check referential integrity

```sql
-- Validation query: Check orphaned background records
SELECT 'Orphaned software_background' AS issue, COUNT(*)
FROM software_background sb
LEFT JOIN users u ON sb.user_id = u.id
WHERE u.id IS NULL

UNION ALL

SELECT 'Orphaned hardware_background', COUNT(*)
FROM hardware_background hb
LEFT JOIN users u ON hb.user_id = u.id
WHERE u.id IS NULL;

-- Should return 0 for both (no orphans due to CASCADE)
```

---

## Conclusion

This data model provides a normalized, performant schema for user authentication and background collection. The design supports:
- Secure password storage via BetterAuth
- Efficient session validation
- Flexible background data storage with array types
- Clear relationships with CASCADE deletion for data integrity
- Indexing strategy optimized for common query patterns

**Ready for API Contract Design**: Entities and relationships are defined, enabling RESTful API endpoint specification.
