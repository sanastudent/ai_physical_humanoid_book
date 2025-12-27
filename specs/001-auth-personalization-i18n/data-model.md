# Data Model: Authentication, Personalization, and Localization

**Feature**: 001-auth-personalization-i18n
**Date**: 2025-12-18
**Status**: Implemented (documenting existing schema)

## Overview

This document describes the data model for user authentication, profile management, and personalization/translation features. The schema is implemented in Neon Serverless PostgreSQL with UUID primary keys and proper relational integrity.

---

## Entity Relationship Diagram

```
┌──────────────────────┐
│       users          │
│──────────────────────│
│ id (PK, UUID)        │◄─────┐
│ email (UNIQUE)       │      │
│ password_hash        │      │
│ created_at           │      │
│ updated_at           │      │
└──────────────────────┘      │
         ▲                     │
         │                     │
         │ 1:N                 │ 1:1
         │                     │
┌────────┴──────────────┐     │
│      sessions         │     │
│───────────────────────│     │
│ id (PK, UUID)         │     │
│ user_id (FK)          │─────┘
│ token (UNIQUE)        │
│ expires_at            │
│ created_at            │
│ updated_at            │
└───────────────────────┘

┌──────────────────────────────┐
│   software_background        │
│──────────────────────────────│
│ id (PK, UUID)                │
│ user_id (FK, UNIQUE)         │───┐
│ experience_level (CHECK)     │   │ 1:1
│ preferred_languages (TEXT[]) │   │
│ preferred_frameworks (TEXT[])│   │
│ created_at                   │   │
│ updated_at                   │   │
└──────────────────────────────┘   │
                                   │
                                   ▼
                            ┌──────────────┐
                            │    users     │
                            └──────────────┘
                                   ▲
                                   │ 1:1
┌──────────────────────────────┐  │
│   hardware_background        │  │
│──────────────────────────────│  │
│ id (PK, UUID)                │  │
│ user_id (FK, UNIQUE)         │──┘
│ experience_level (CHECK)     │
│ preferred_platforms (TEXT[]) │
│ device_types (TEXT[])        │
│ created_at                   │
│ updated_at                   │
└──────────────────────────────┘
```

---

## Entity Definitions

### 1. User

**Table Name**: `users`

**Description**: Core user account information for authentication.

**Columns**:

| Column         | Type         | Constraints                  | Description                              |
|----------------|--------------|------------------------------|------------------------------------------|
| id             | UUID         | PRIMARY KEY, NOT NULL        | Unique user identifier (UUID v4)         |
| email          | VARCHAR(255) | NOT NULL, UNIQUE             | User email address (login credential)    |
| password_hash  | VARCHAR(255) | NOT NULL                     | bcrypt hash of user password             |
| created_at     | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Account creation timestamp               |
| updated_at     | TIMESTAMP    | NOT NULL, DEFAULT NOW()      | Last account update timestamp            |

**Indexes**:
- Primary key index on `id`
- Unique index on `email` (for login lookup)

**Validation Rules** (Application Layer):
- Email must be valid format (RFC 5322)
- Password must be >= 8 characters
- Password must contain 1 uppercase, 1 lowercase, 1 digit
- Email uniqueness enforced at database level

**State Transitions**:
- Created → Active (on successful signup)
- Active → Deleted (on account deletion, future enhancement)

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "password_hash": "$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy",
  "created_at": "2025-12-18T10:00:00Z",
  "updated_at": "2025-12-18T10:00:00Z"
}
```

---

### 2. Session

**Table Name**: `sessions`

**Description**: User authentication sessions with expiration tracking.

**Columns**:

| Column     | Type      | Constraints                     | Description                                    |
|------------|-----------|---------------------------------|------------------------------------------------|
| id         | UUID      | PRIMARY KEY, NOT NULL           | Unique session identifier (UUID v4)            |
| user_id    | UUID      | NOT NULL, FOREIGN KEY (users)   | Reference to owning user                       |
| token      | VARCHAR   | NOT NULL, UNIQUE                | 64-char hex session token (256-bit random)     |
| expires_at | TIMESTAMP | NOT NULL                        | Session expiration timestamp                   |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW()         | Session creation timestamp                     |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW()         | Last session update timestamp                  |

**Indexes**:
- Primary key index on `id`
- Unique index on `token` (for session lookup)
- Index on `user_id` (for user session queries)
- Index on `expires_at` (for cleanup queries)

**Validation Rules**:
- Token must be 64-character hexadecimal string
- expires_at must be in the future at creation time
- Foreign key constraint ensures user existence

**State Transitions**:
- Created → Active (on signin)
- Active → Expired (when current time > expires_at)
- Active → Invalidated (on explicit logout)

**Session Lifetimes**:
- Default: 24 hours
- Remember me: 30 days

**Cleanup Strategy**: Periodic job deletes expired sessions (expires_at < NOW())

**Example**:
```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "token": "a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456",
  "expires_at": "2025-12-19T10:00:00Z",
  "created_at": "2025-12-18T10:00:00Z",
  "updated_at": "2025-12-18T10:00:00Z"
}
```

---

### 3. Software Background

**Table Name**: `software_background`

**Description**: User's software development experience and preferences for personalization.

**Columns**:

| Column                | Type      | Constraints                                    | Description                                |
|-----------------------|-----------|------------------------------------------------|--------------------------------------------|
| id                    | UUID      | PRIMARY KEY, NOT NULL                          | Unique background record identifier        |
| user_id               | UUID      | NOT NULL, UNIQUE, FOREIGN KEY (users)          | One-to-one reference to user               |
| experience_level      | VARCHAR(20)| NOT NULL, CHECK IN ('beginner', 'intermediate', 'advanced') | Software experience level |
| preferred_languages   | TEXT[]    | DEFAULT '{}'                                   | Array of programming languages (e.g., ['Python', 'JavaScript']) |
| preferred_frameworks  | TEXT[]    | DEFAULT '{}'                                   | Array of frameworks (e.g., ['React', 'FastAPI']) |
| created_at            | TIMESTAMP | NOT NULL, DEFAULT NOW()                        | Background data creation timestamp         |
| updated_at            | TIMESTAMP | NOT NULL, DEFAULT NOW()                        | Last background data update timestamp      |

**Indexes**:
- Primary key index on `id`
- Unique index on `user_id` (enforces one-to-one relationship)

**Validation Rules**:
- experience_level must be one of: 'beginner', 'intermediate', 'advanced'
- preferred_languages and preferred_frameworks are optional arrays
- Foreign key constraint ensures user existence

**Personalization Mapping**:
- **beginner**: Simple language, detailed explanations, basic examples
- **intermediate**: Moderate complexity, balanced theory and practice
- **advanced**: Technical terminology, advanced concepts, minimal hand-holding

**Example**:
```json
{
  "id": "770e8400-e29b-41d4-a716-446655440002",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "experience_level": "intermediate",
  "preferred_languages": ["Python", "JavaScript", "TypeScript"],
  "preferred_frameworks": ["React", "FastAPI", "Node.js"],
  "created_at": "2025-12-18T10:05:00Z",
  "updated_at": "2025-12-18T10:05:00Z"
}
```

---

### 4. Hardware Background

**Table Name**: `hardware_background`

**Description**: User's hardware/robotics experience and preferences for personalization.

**Columns**:

| Column              | Type      | Constraints                                    | Description                                  |
|---------------------|-----------|------------------------------------------------|----------------------------------------------|
| id                  | UUID      | PRIMARY KEY, NOT NULL                          | Unique background record identifier          |
| user_id             | UUID      | NOT NULL, UNIQUE, FOREIGN KEY (users)          | One-to-one reference to user                 |
| experience_level    | VARCHAR(20)| NOT NULL, CHECK IN ('beginner', 'intermediate', 'advanced') | Hardware experience level |
| preferred_platforms | TEXT[]    | DEFAULT '{}'                                   | Array of hardware platforms (e.g., ['Raspberry Pi', 'Jetson']) |
| device_types        | TEXT[]    | DEFAULT '{}'                                   | Array of device types (e.g., ['Mobile', 'Embedded', 'Desktop']) |
| created_at          | TIMESTAMP | NOT NULL, DEFAULT NOW()                        | Background data creation timestamp           |
| updated_at          | TIMESTAMP | NOT NULL, DEFAULT NOW()                        | Last background data update timestamp        |

**Indexes**:
- Primary key index on `id`
- Unique index on `user_id` (enforces one-to-one relationship)

**Validation Rules**:
- experience_level must be one of: 'beginner', 'intermediate', 'advanced'
- preferred_platforms and device_types are optional arrays
- Foreign key constraint ensures user existence

**Personalization Mapping**:
- **beginner**: Simplified hardware concepts, analogies to familiar devices
- **intermediate**: Technical specifications, component integration patterns
- **advanced**: Low-level details, optimization techniques, edge cases

**Example**:
```json
{
  "id": "880e8400-e29b-41d4-a716-446655440003",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "experience_level": "advanced",
  "preferred_platforms": ["NVIDIA Jetson", "Raspberry Pi", "Arduino"],
  "device_types": ["Embedded", "Mobile", "IoT"],
  "created_at": "2025-12-18T10:05:00Z",
  "updated_at": "2025-12-18T10:05:00Z"
}
```

---

## Non-Persistent Entities (Runtime Only)

### 5. Personalization Request

**Description**: Runtime data structure for personalization requests (NOT stored in database).

**Structure**:
```typescript
interface PersonalizationRequest {
  chapter_content: string;           // Original markdown content
  user_profile: {
    software_experience: 'beginner' | 'intermediate' | 'advanced';
    hardware_experience: 'beginner' | 'intermediate' | 'advanced';
    preferred_languages?: string[];
    preferred_frameworks?: string[];
    preferred_platforms?: string[];
  };
  learning_style?: 'visual' | 'auditory' | 'reading_writing' | 'kinesthetic' | 'multimodal';
}
```

**Validation Rules**:
- chapter_content must not be empty
- user_profile fields must match database enum values
- learning_style is optional, defaults to 'multimodal'

---

### 6. Personalized Content

**Description**: Runtime-generated adapted content (cached in frontend localStorage, NOT in database).

**Structure**:
```typescript
interface PersonalizedContent {
  chapter_id: string;               // Unique chapter identifier
  original_content: string;         // Original markdown (for comparison)
  personalized_content: string;     // Adapted markdown
  personalization_applied: {
    software_level: string;
    hardware_level: string;
    learning_style: string;
  };
  generated_at: string;             // ISO 8601 timestamp
  cache_key: string;                // Hash of chapter_id + user_profile
}
```

**Caching Strategy**:
- Storage: Frontend localStorage
- Key: `bookPersonalizedContents.${chapterId}`
- TTL: Session-based (cleared on logout or manual clear)
- Invalidation: Manual or on profile update

---

### 7. Translation Request

**Description**: Runtime data structure for translation requests (NOT stored in database).

**Structure**:
```typescript
interface TranslationRequest {
  content: string;                  // Original content (markdown)
  target_language: 'ur' | 'ar' | 'es' | 'fr' | 'de';  // Target language code
  preserve_code_blocks: boolean;    // Default: true
  preserve_technical_terms: boolean; // Default: true
}
```

**Validation Rules**:
- content must not be empty
- target_language must be supported ('ur' for Urdu is primary)
- preserve_code_blocks and preserve_technical_terms default to true

---

### 8. Translated Content

**Description**: Runtime-generated translated content (optionally cached).

**Structure**:
```typescript
interface TranslatedContent {
  chapter_id: string;               // Unique chapter identifier
  original_content: string;         // Original markdown
  translated_content: string;       // Translated markdown
  target_language: string;          // Language code
  generated_at: string;             // ISO 8601 timestamp
  processing_time_ms: number;       // AI processing time
}
```

**Caching Strategy**:
- Optional backend cache (Redis or in-memory)
- Optional frontend cache (localStorage)
- Cache key: `translation_${chapterId}_${targetLanguage}`

---

## Relationships

### User ↔ Session (1:N)
- One user can have multiple active sessions (e.g., logged in on multiple devices)
- Sessions belong to exactly one user
- Cascade delete: Deleting a user deletes all sessions

### User ↔ Software Background (1:1)
- One user has zero or one software background record
- Software background belongs to exactly one user
- Nullable: User can exist without background data (incomplete profile)

### User ↔ Hardware Background (1:1)
- One user has zero or one hardware background record
- Hardware background belongs to exactly one user
- Nullable: User can exist without background data (incomplete profile)

---

## Data Integrity Constraints

### Database Level
1. **Foreign Key Constraints**: All foreign keys have ON DELETE CASCADE
2. **Unique Constraints**: email (users), token (sessions), user_id (backgrounds)
3. **Check Constraints**: experience_level enums validated at database level
4. **Not Null Constraints**: Critical fields (id, email, password_hash, etc.)

### Application Level
1. **Email Validation**: RFC 5322 format validation
2. **Password Strength**: 8+ chars, uppercase, lowercase, digit
3. **Session Token Format**: 64-character hexadecimal
4. **Experience Level**: Enum validation ('beginner'|'intermediate'|'advanced')

---

## Migration Strategy

**Migration Files** (in `backend/migrations/`):
1. `001_create_users.sql` - Create users table
2. `002_create_sessions.sql` - Create sessions table with foreign key to users
3. `003_create_software_background.sql` - Create software_background table
4. `004_create_hardware_background.sql` - Create hardware_background table
5. `005_create_indexes.sql` - Add performance indexes
6. `006_create_triggers.sql` - Add updated_at triggers

**Migration Execution**: Run `python backend/migrations/run_migrations.py`

**Rollback Strategy**: Each migration includes DOWN statements for rollback

---

## Query Patterns

### Common Queries

**1. Authenticate User**:
```sql
SELECT id, email, password_hash
FROM users
WHERE email = ?;
```

**2. Validate Session**:
```sql
SELECT s.id, s.user_id, s.expires_at, u.email
FROM sessions s
JOIN users u ON s.user_id = u.id
WHERE s.token = ? AND s.expires_at > NOW();
```

**3. Get User Profile (Full)**:
```sql
SELECT
  u.id, u.email, u.created_at,
  sb.experience_level AS software_level,
  sb.preferred_languages,
  sb.preferred_frameworks,
  hb.experience_level AS hardware_level,
  hb.preferred_platforms,
  hb.device_types
FROM users u
LEFT JOIN software_background sb ON u.id = sb.user_id
LEFT JOIN hardware_background hb ON u.id = hb.user_id
WHERE u.id = ?;
```

**4. Cleanup Expired Sessions**:
```sql
DELETE FROM sessions
WHERE expires_at < NOW();
```

---

## Performance Considerations

### Indexes
- All foreign keys are indexed for join performance
- Unique columns (email, token) automatically indexed
- expires_at indexed for efficient session cleanup

### Connection Pooling
- Pool size: 5-20 connections (configured in backend/src/database/connection.py)
- Prevents connection exhaustion under load

### Query Optimization
- LEFT JOIN for optional backgrounds (nullable one-to-one)
- Indexed lookups for authentication (email, token)
- Batch deletion for session cleanup

---

## Security Notes

### Password Storage
- Never store plaintext passwords
- bcrypt with salt rounds = 10
- Password hash stored in `password_hash` column

### Session Token Security
- 256-bit cryptographically secure random tokens
- HttpOnly cookies prevent XSS access
- Secure flag enforces HTTPS
- SameSite=Lax prevents CSRF

### SQL Injection Prevention
- Parameterized queries throughout
- ORM-style query builders (psycopg2 with parameter binding)

---

## Conclusion

Data model is **fully implemented** with proper relational integrity, indexes, and security constraints. No schema changes required for this feature - all tables and relationships already exist. Implementation work focuses on:
1. Exposing personalization endpoint
2. Frontend UI components
3. Test coverage

**Next Steps**: Generate API contracts in OpenAPI format.
