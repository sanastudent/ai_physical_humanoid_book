# Architectural & Implementation Plan: BetterAuth Signup & Signin with User Background Collection

**Feature**: 002-betterauth-signup
**Branch**: `002-betterauth-signup`
**Created**: 2025-12-11
**Status**: Ready for Implementation
**Plan Type**: Full-Stack Authentication with Data Collection

---

## 1. Scope and Dependencies

### In Scope

1. **User Authentication System**
   - Email/password signup with BetterAuth
   - Email/password signin with session management
   - Secure session handling via HTTP-only cookies
   - Signout functionality

2. **User Background Collection**
   - Multi-step signup flow (credentials → background questions)
   - Software background questions (experience level, languages, frameworks)
   - Hardware background questions (experience level, platforms, devices)
   - Background data storage in Neon DB (PostgreSQL)

3. **Frontend Integration**
   - Signup/signin forms with BetterAuth React hooks
   - Background questions UI components
   - PersonalizationContext extension to include authentication state
   - Session-aware personalization button (show only to logged-in users)

4. **Backend API**
   - Authentication endpoints (`/auth/signup`, `/auth/signin`, `/auth/signout`, `/auth/session`)
   - Background data endpoints (`POST /background`, `GET /background`, `PUT /background`)
   - Session validation middleware
   - Database integration with Neon DB

5. **Database Schema**
   - `users` table (BetterAuth managed)
   - `sessions` table (BetterAuth managed)
   - `software_background` table (custom)
   - `hardware_background` table (custom)
   - Migrations and indexes

### Out of Scope

- Password reset functionality (future enhancement)
- Email verification during signup (future enhancement)
- Social OAuth providers (Google, GitHub, etc.)
- Two-factor authentication (2FA)
- Rate limiting for authentication endpoints (future enhancement)
- User profile pictures
- Account deletion flow

### External Dependencies

| Dependency | Ownership | Purpose | Risk Level |
|------------|-----------|---------|------------|
| **BetterAuth** | Third-party library | Authentication framework for React/Node | Low - well-documented, actively maintained |
| **Neon DB** | External service | PostgreSQL database hosting | Low - serverless Postgres with high availability |
| **FastAPI Backend** | Internal (existing) | Backend API framework | Low - already in use |
| **Docusaurus/React** | Internal (existing) | Frontend framework | Low - already in use |
| **PersonalizationContext** | Internal (existing) | User preferences state management | Medium - requires extension for auth |

---

## 2. Key Decisions and Rationale

### Decision 1: Multi-Step Signup Flow

**Options Considered**:
1. **Single-step form**: All credentials and background questions on one page
2. **Multi-step flow**: Step 1 = credentials, Step 2 = background questions
3. **Post-signup optional**: Allow users to skip background questions initially

**Trade-offs**:
- Single-step: Simple implementation, but high cognitive load and mobile UX issues
- Multi-step: Better UX, lower abandonment, but requires state management between steps
- Post-signup optional: Lowest friction, but may result in incomplete profiles

**Decision**: **Multi-step signup flow** (Step 1: Credentials → Step 2: Background Questions)

**Rationale**:
- Reduces initial friction (email/password only first)
- Industry best practice for conversion optimization
- Better mobile experience with focused steps
- Allows account creation before commitment to questions
- Can prompt later if background is incomplete

**Principles Applied**:
- **Measurable**: Track signup completion rate at each step
- **Reversible**: Can revert to single-step if data shows high step 2 abandonment
- **Smallest viable change**: Minimal modification to existing auth patterns

---

### Decision 2: Normalized Database Schema

**Options Considered**:
1. **JSONB storage**: Store all background data in single JSONB column
2. **Normalized tables**: Separate `software_background` and `hardware_background` tables
3. **Single background table**: Combined software and hardware fields in one table

**Trade-offs**:
- JSONB: Flexible schema, but harder to query and validate
- Normalized tables: Type-safe, efficient queries, but more complex joins
- Single table: Simpler queries, but mixed concerns and harder to extend

**Decision**: **Normalized relational schema** with separate `software_background` and `hardware_background` tables

**Rationale**:
- Structured data enables efficient personalization queries (e.g., "all Python developers")
- Type safety and validation at database level (CHECK constraints, NOT NULL)
- Clear separation of concerns (software vs. hardware background)
- PostgreSQL array types support flexible lists (languages, platforms)
- Easy to extend with new fields without schema migration

**Principles Applied**:
- **Reversible**: Can consolidate tables if proven unnecessary
- **Measurable**: Query performance benchmarked via database metrics
- **Smallest viable change**: Only adds two tables, no modification to existing schema

---

### Decision 3: Extend PersonalizationContext for Authentication

**Options Considered**:
1. **Separate AuthContext**: New context for authentication, keep PersonalizationContext unchanged
2. **Extend PersonalizationContext**: Add user session to existing context
3. **Global state management**: Use Redux/Zustand for both auth and personalization

**Trade-offs**:
- Separate AuthContext: Clean separation, but redundant state and sync issues
- Extend PersonalizationContext: Single source of truth, but mixed responsibilities
- Global state: Centralized, but overkill for current scope and adds dependency

**Decision**: **Extend PersonalizationContext** to include BetterAuth session and user background data

**Rationale**:
- Personalization already depends on user identity (preferences tied to user)
- Single source of truth reduces state synchronization bugs
- BetterAuth `useSession()` hook integrates cleanly into existing context
- Background data fetched once on login and cached in context
- Minimal changes to existing personalization button component

**Implementation Pattern**:
```typescript
// PersonalizationContext.tsx (extended)
const { session } = useSession(); // BetterAuth hook
const [userBackground, setUserBackground] = useState(null);

useEffect(() => {
  if (session?.user) {
    fetchUserBackground(session.user.id).then(setUserBackground);
  }
}, [session]);

return (
  <PersonalizationContext.Provider value={{
    user: session?.user,
    background: userBackground,
    preferences,
    setPreferences,
    // ... existing methods
  }}>
    {children}
  </PersonalizationContext.Provider>
);
```

**Principles Applied**:
- **Smallest viable change**: Extends existing pattern rather than introducing new architecture
- **Reversible**: Can split contexts later if complexity warrants
- **Measurable**: Context re-render performance tracked via React DevTools

---

### Decision 4: BetterAuth for Authentication

**Options Considered**:
1. **BetterAuth**: TypeScript-first authentication library
2. **NextAuth.js**: Mature, widely adopted authentication solution
3. **Custom JWT implementation**: Build from scratch with jsonwebtoken

**Trade-offs**:
- BetterAuth: Modern, TypeScript-native, but smaller ecosystem
- NextAuth.js: Battle-tested, but more Next.js-centric
- Custom JWT: Full control, but high implementation risk and security concerns

**Decision**: **BetterAuth** (as mandated by project constraints)

**Rationale**:
- **Constraint**: Specification explicitly requires BetterAuth
- Native TypeScript support aligns with frontend (React/TypeScript)
- Built-in session management via HTTP-only cookies (secure by default)
- React hooks (`useSession`, `signIn`, `signUp`) integrate seamlessly
- bcrypt password hashing included (no manual crypto implementation)
- CSRF protection and secure session tokens out-of-the-box

**Principles Applied**:
- **Constraint-driven**: Decision pre-determined by requirements
- **Security-first**: Leverages library's built-in security features rather than custom implementation

---

## 3. Interfaces and API Contracts

### Authentication Endpoints

#### `POST /auth/signup`

**Purpose**: Create new user account with email/password

**Request**:
```json
{
  "email": "user@example.com",
  "password": "SecurePass123"
}
```

**Response** (201 Created):
```json
{
  "user": {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "email": "user@example.com",
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  },
  "message": "Account created successfully"
}
```

**Response** (409 Conflict - Email exists):
```json
{
  "error": "Email already registered"
}
```

**Errors**:
- `400 Bad Request`: Invalid email format, weak password
- `409 Conflict`: Email already exists
- `500 Internal Server Error`: Database connection failure

**Idempotency**: Not idempotent (duplicate emails return 409)

**Timeouts**: 5 seconds (database write operation)

**Retries**: No auto-retry (user should fix validation errors)

---

#### `POST /auth/signin`

**Purpose**: Authenticate user and create session

**Request**:
```json
{
  "email": "user@example.com",
  "password": "SecurePass123",
  "rememberMe": false
}
```

**Response** (200 OK):
```json
{
  "user": {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "email": "user@example.com",
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  },
  "message": "Signed in successfully"
}
```

**Response** (401 Unauthorized):
```json
{
  "error": "Invalid email or password"
}
```

**Errors**:
- `400 Bad Request`: Missing email or password
- `401 Unauthorized`: Invalid credentials
- `500 Internal Server Error`: Database connection failure

**Session Cookie**: `Set-Cookie: session_token=<token>; HttpOnly; Secure; SameSite=Lax; Max-Age=<duration>`

**Timeouts**: 3 seconds (session creation)

**Retries**: No auto-retry (user should check credentials)

---

#### `POST /auth/signout`

**Purpose**: Invalidate current session

**Request**: None (session token from cookie)

**Response** (200 OK):
```json
{
  "message": "Signed out successfully"
}
```

**Errors**:
- `401 Unauthorized`: No active session
- `500 Internal Server Error`: Session deletion failure

**Session Cookie Cleared**: `Set-Cookie: session_token=; Max-Age=0`

**Timeouts**: 2 seconds (session deletion)

---

#### `GET /auth/session`

**Purpose**: Retrieve current authenticated user session

**Request**: None (session token from cookie)

**Response** (200 OK):
```json
{
  "user": {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "email": "user@example.com",
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  },
  "session": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "123e4567-e89b-12d3-a456-426614174000",
    "expiresAt": "2025-12-12T10:30:00Z",
    "createdAt": "2025-12-11T10:30:00Z"
  }
}
```

**Response** (401 Unauthorized - No session):
```json
{
  "user": null,
  "session": null
}
```

**Errors**:
- `401 Unauthorized`: Session expired or invalid
- `500 Internal Server Error`: Database query failure

**Timeouts**: 1 second (session lookup)

---

### Background Data Endpoints

#### `POST /background`

**Purpose**: Create software and hardware background data for authenticated user

**Request**:
```json
{
  "software": {
    "experienceLevel": "intermediate",
    "preferredLanguages": ["Python", "JavaScript", "Go"],
    "preferredFrameworks": ["FastAPI", "React", "Fiber"]
  },
  "hardware": {
    "experienceLevel": "beginner",
    "preferredPlatforms": ["desktop", "mobile"],
    "deviceTypes": ["laptop", "smartphone"]
  }
}
```

**Response** (201 Created):
```json
{
  "userId": "123e4567-e89b-12d3-a456-426614174000",
  "software": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "experienceLevel": "intermediate",
    "preferredLanguages": ["Python", "JavaScript", "Go"],
    "preferredFrameworks": ["FastAPI", "React", "Fiber"],
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  },
  "hardware": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "experienceLevel": "beginner",
    "preferredPlatforms": ["desktop", "mobile"],
    "deviceTypes": ["laptop", "smartphone"],
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  }
}
```

**Errors**:
- `400 Bad Request`: Invalid experience level, empty arrays, invalid values
- `401 Unauthorized`: No active session
- `409 Conflict`: Background data already exists (use PUT to update)
- `500 Internal Server Error`: Database insert failure

**Validation Rules**:
- `experienceLevel`: Must be one of `beginner`, `intermediate`, `advanced`
- `preferredLanguages`: At least one language required
- `preferredPlatforms`: At least one platform required
- Arrays must contain values from predefined lists (validated server-side)

**Timeouts**: 5 seconds (database transaction)

**Idempotency**: Not idempotent (returns 409 if already exists)

---

#### `GET /background`

**Purpose**: Retrieve user's background data

**Request**: None (user ID from session)

**Response** (200 OK):
```json
{
  "userId": "123e4567-e89b-12d3-a456-426614174000",
  "software": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "experienceLevel": "intermediate",
    "preferredLanguages": ["Python", "JavaScript", "Go"],
    "preferredFrameworks": ["FastAPI", "React", "Fiber"],
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  },
  "hardware": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "experienceLevel": "beginner",
    "preferredPlatforms": ["desktop", "mobile"],
    "deviceTypes": ["laptop", "smartphone"],
    "createdAt": "2025-12-11T10:30:00Z",
    "updatedAt": "2025-12-11T10:30:00Z"
  }
}
```

**Response** (404 Not Found):
```json
{
  "error": "Background data not found"
}
```

**Errors**:
- `401 Unauthorized`: No active session
- `404 Not Found`: Background data not created yet
- `500 Internal Server Error`: Database query failure

**Timeouts**: 2 seconds (database join query)

**Caching**: Frontend should cache in PersonalizationContext after first fetch

---

#### `PUT /background`

**Purpose**: Update user's background data

**Request**: Same as `POST /background`

**Response** (200 OK): Same as `POST /background`

**Errors**:
- `400 Bad Request`: Invalid data
- `401 Unauthorized`: No active session
- `404 Not Found`: Background data not created yet (use POST)
- `500 Internal Server Error`: Database update failure

**Timeouts**: 5 seconds (database transaction)

**Idempotency**: Idempotent (multiple identical updates produce same result)

---

### Error Taxonomy

| Status Code | Error Type | Description | User Action |
|-------------|------------|-------------|-------------|
| 400 | `validation_error` | Invalid input data | Fix form errors |
| 401 | `unauthorized` | No session or invalid credentials | Sign in again |
| 404 | `not_found` | Resource doesn't exist | Create resource first |
| 409 | `conflict` | Resource already exists | Update instead of create |
| 500 | `internal_error` | Server/database failure | Retry or contact support |

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance

**Target Metrics**:
- **Signup latency (p95)**: < 1000ms (account creation + session start)
- **Signin latency (p95)**: < 500ms (credential validation + session start)
- **Background data fetch (p95)**: < 300ms (JOIN query with indexes)
- **Session validation (p95)**: < 50ms (indexed token lookup)

**Throughput**:
- Signup: ~10 TPS (low volume expected)
- Signin: ~50 TPS (moderate volume)
- Session validation: ~500 TPS (every authenticated request)

**Resource Caps**:
- Database connection pool: Max 20 connections
- API request timeout: 30 seconds (prevents hanging requests)
- Session storage: Max 10,000 active sessions (cleanup job removes expired)

**Performance Budget**:
- Signup flow: 3 seconds end-to-end (includes UI + network + backend)
- Personalization button latency: No degradation (background fetch cached)

---

### Reliability

**SLOs** (Service Level Objectives):
- Availability: 99.5% uptime (excluding maintenance windows)
- Error rate: < 1% of authentication requests fail
- Data durability: 100% (PostgreSQL ACID guarantees)

**Error Budget**:
- 3.6 hours downtime per month allowed
- 1000 failed requests per 100,000 requests

**Degradation Strategy**:
- **Database unavailable**: Return 503 Service Unavailable, display "Try again later" message
- **Session lookup slow**: Use cached session data if available (stale data acceptable for 5 minutes)
- **Background data fetch fails**: Allow personalization with default preferences, prompt to complete profile

**Graceful Degradation**:
- If authentication fails, personalization button shows "Sign in to personalize" (no crash)
- If background data incomplete, use default preferences (beginner, generic examples)

---

### Security

**Authentication & Authorization**:
- **Password Storage**: bcrypt with salt rounds=10 (BetterAuth default)
- **Session Tokens**: Cryptographically random (256-bit), stored in HTTP-only cookies
- **Cookie Flags**: `HttpOnly`, `Secure` (HTTPS only), `SameSite=Lax` (CSRF protection)

**Data Handling**:
- **Passwords**: Never logged, never sent in responses, only hashed values stored
- **Session Tokens**: Never exposed in API responses, only sent via Set-Cookie header
- **User Background**: Accessible only to authenticated user (ownership check via user_id)

**Secrets Management**:
- `BETTERAUTH_SECRET`: Stored in `.env`, used for session encryption
- `NEON_DB_URL`: Stored in `.env`, includes credentials for database access
- Never commit `.env` to version control (`.gitignore` enforced)

**Input Validation**:
- **Email**: RFC 5322 format validation (client + server)
- **Password**: Min 8 chars, 1 uppercase, 1 lowercase, 1 number (client + server)
- **SQL Injection**: Prevented via parameterized queries (SQLAlchemy/psycopg2)
- **XSS Prevention**: React's built-in escaping, no `dangerouslySetInnerHTML` for user input

**Audit Logging** (Future Enhancement):
- Log all authentication events (signup, signin, signout) with timestamps
- Log failed login attempts for rate limiting analysis
- Log background data updates with user_id and timestamp

---

### Cost

**Unit Economics**:
- **Neon DB**: ~$0.10 per 1M queries (serverless Postgres pricing)
- **Compute**: Existing FastAPI backend (no additional infrastructure)
- **BetterAuth**: Free open-source library (no SaaS costs)

**Expected Monthly Cost** (1000 active users):
- Signup: 1000 signups × 3 queries = 3,000 queries (~$0.0003)
- Signin: 10,000 logins × 2 queries = 20,000 queries (~$0.002)
- Session validation: 500,000 requests × 1 query = 500,000 queries (~$0.05)
- Background fetch: 5,000 personalizations × 1 query = 5,000 queries (~$0.0005)

**Total**: < $0.10/month (negligible cost)

**Cost Optimization**:
- Session validation uses indexed token lookup (fastest query)
- Background data cached in frontend (reduces repeated fetches)

---

## 5. Data Management and Migration

### Source of Truth

- **User Accounts**: `users` table in Neon DB (managed by BetterAuth)
- **Sessions**: `sessions` table in Neon DB (managed by BetterAuth)
- **Background Data**: `software_background` and `hardware_background` tables (custom)
- **User Preferences**: Stored in `PersonalizationContext` and localStorage (frontend)

**Canonical Data Flow**:
1. User signs up → `users` table created
2. User completes background questions → `software_background` + `hardware_background` inserted
3. User logs in → Session token stored in `sessions` table
4. Frontend fetches background → Cached in `PersonalizationContext`
5. User clicks personalization → Background data sent to `/personalize` endpoint

---

### Schema Evolution

**Current Schema Version**: 1.0.0

**Migration Strategy**:
- Use database migration tool (e.g., Alembic for Python, Prisma for TypeScript)
- BetterAuth provides migration scripts for `users` and `sessions` tables
- Custom migrations for `software_background` and `hardware_background` tables

**Backward Compatibility**:
- New columns must have `DEFAULT` values (allow existing rows to remain valid)
- Never drop columns without deprecation period (at least 1 release cycle)
- Use feature flags for schema changes that affect application logic

**Example Migration** (Adding "years_of_experience" to software_background):
```sql
-- Migration 004: Add years_of_experience column
ALTER TABLE software_background
ADD COLUMN years_of_experience INTEGER DEFAULT 0 NOT NULL;

-- Backfill existing rows based on experience_level
UPDATE software_background
SET years_of_experience = CASE
  WHEN experience_level = 'beginner' THEN 1
  WHEN experience_level = 'intermediate' THEN 3
  WHEN experience_level = 'advanced' THEN 5
  ELSE 0
END;
```

---

### Data Retention

**User Data**:
- **Active Users**: Retained indefinitely while account is active
- **Inactive Users**: No automatic deletion (user must request account deletion)
- **Sessions**: Expired sessions deleted automatically (cleanup job runs hourly)

**Cleanup Policy**:
- Expired sessions (older than `expires_at`) deleted every hour
- Failed signup attempts (partial data) rolled back immediately

**GDPR Compliance** (User Deletion):
- User can request account deletion (future enhancement)
- `ON DELETE CASCADE` ensures all related data deleted (sessions, background data)
- No backups retained after deletion (hard delete, not soft delete)

---

### Migration and Rollback

**Initial Deployment**:
1. Run BetterAuth migrations (creates `users` and `sessions` tables)
2. Run custom migrations (creates `software_background` and `hardware_background` tables)
3. Deploy backend API with new authentication endpoints
4. Deploy frontend with signup/signin forms and PersonalizationContext changes

**Rollback Plan** (If Critical Bug Discovered):
1. Revert frontend deployment (users see "Sign in to personalize" but no signup forms)
2. Keep database schema (data retained for retry)
3. Fix bugs in development
4. Re-deploy with fixes

**Data Migration** (If Schema Changes Required):
- Use database transactions for migrations (all-or-nothing)
- Test migrations on staging database first
- Run migrations during low-traffic window
- Monitor error logs for constraint violations

**Validation Query** (Check Data Integrity):
```sql
-- Ensure no orphaned background records (CASCADE should prevent this)
SELECT 'Orphaned software_background' AS issue, COUNT(*)
FROM software_background sb
LEFT JOIN users u ON sb.user_id = u.id
WHERE u.id IS NULL

UNION ALL

SELECT 'Orphaned hardware_background', COUNT(*)
FROM hardware_background hb
LEFT JOIN users u ON hb.user_id = u.id
WHERE u.id IS NULL;

-- Expected result: 0 for both (no orphans)
```

---

## 6. Operational Readiness

### Observability

**Logs**:
- **Authentication Events**: Signup, signin, signout (with user_id, timestamp, success/failure)
- **Validation Errors**: Failed signups with error details (email format, weak password)
- **Database Errors**: Connection failures, query timeouts, constraint violations
- **Session Errors**: Expired tokens, invalid tokens, missing cookies

**Log Format** (Structured JSON):
```json
{
  "timestamp": "2025-12-11T10:30:00Z",
  "level": "INFO",
  "event": "user_signup",
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "success": true,
  "latency_ms": 450
}
```

**Metrics**:
- Signup success rate (per hour)
- Signin success rate (per hour)
- Session validation latency (p50, p95, p99)
- Background data fetch latency (p50, p95, p99)
- Database connection pool utilization

**Traces** (Optional, Future Enhancement):
- Distributed tracing for multi-step signup flow (frontend → backend → database)
- Track latency breakdown (network, API processing, database query)

---

### Alerting

**Critical Alerts** (Page On-Call):
- Authentication success rate < 95% (5-minute window)
- Database connection failures (any failure triggers alert)
- API error rate > 5% (5-minute window)

**Warning Alerts** (Slack/Email):
- Session validation latency > 100ms (p95, 10-minute window)
- Signup latency > 2000ms (p95, 10-minute window)
- Expired session cleanup job failed

**Alert Owners**:
- Backend Team: API errors, database issues
- Frontend Team: Client-side validation errors
- DevOps Team: Infrastructure failures, deployment issues

---

### Runbooks

#### Runbook 1: High Authentication Failure Rate

**Symptoms**: Signup/signin success rate < 95%

**Investigation Steps**:
1. Check database connectivity: `SELECT 1 FROM users LIMIT 1`
2. Check BetterAuth logs for error patterns
3. Check recent deployments (rollback if recent change)
4. Verify environment variables (`BETTERAUTH_SECRET`, `NEON_DB_URL`)

**Resolution**:
- If database down: Escalate to Neon DB support
- If recent deployment: Rollback to previous version
- If environment variable missing: Update and restart service

---

#### Runbook 2: Session Validation Slow

**Symptoms**: Session validation latency > 100ms (p95)

**Investigation Steps**:
1. Check database query performance: `EXPLAIN ANALYZE SELECT * FROM sessions WHERE token = $1`
2. Verify index exists: `\d sessions` (should show index on `token`)
3. Check database connection pool saturation
4. Check for slow queries: `SELECT * FROM pg_stat_activity WHERE state = 'active'`

**Resolution**:
- If index missing: Create index `CREATE INDEX idx_sessions_token ON sessions(token)`
- If connection pool saturated: Increase pool size or optimize slow queries
- If database overloaded: Scale up Neon DB instance

---

### Deployment Strategy

**Deployment Method**: Blue-Green Deployment

**Steps**:
1. **Database Migration**: Run migrations on production database (non-destructive)
2. **Backend Deployment**: Deploy new API version with authentication endpoints
3. **Frontend Deployment**: Deploy new signup/signin forms and PersonalizationContext changes
4. **Health Check**: Verify `/health/ready` endpoint returns 200 OK
5. **Smoke Tests**: Create test account, sign in, complete background questions
6. **Rollback Plan**: If smoke tests fail, revert frontend and backend deployments

**Rollback Procedure**:
1. Revert frontend deployment (users see old UI without signup forms)
2. Revert backend deployment (remove authentication endpoints)
3. Database schema remains (data retained for retry)
4. Investigate failures in staging environment
5. Re-deploy with fixes after validation

---

### Feature Flags

**Flag 1: `auth_enabled`**
- **Purpose**: Enable/disable entire authentication system
- **Default**: `false` (until fully tested)
- **When to toggle**: After successful deployment and smoke tests

**Flag 2: `background_questions_required`**
- **Purpose**: Make background questions optional during signup
- **Default**: `true` (required)
- **When to toggle**: If high abandonment rate at Step 2

**Flag 3: `personalization_requires_auth`**
- **Purpose**: Require authentication for personalization button
- **Default**: `true` (after auth system deployed)
- **When to toggle**: During migration period (allow anonymous personalization)

**Feature Flag Storage**: Environment variables (`.env` file for backend, build-time config for frontend)

---

## 7. Risk Analysis and Mitigation

### Risk 1: High Signup Abandonment (Step 2)

**Severity**: Medium
**Likelihood**: Medium
**Blast Radius**: Impacts new user onboarding (reduces user base growth)

**Mitigation**:
- **Pre-launch**: A/B test multi-step vs. single-step flow in staging
- **Post-launch**: Track completion rate at each step (analytics)
- **Kill Switch**: Feature flag `background_questions_required` allows making Step 2 optional
- **Rollback**: Revert to single-step flow if Step 2 completion < 70%

**Monitoring**: Track signup funnel metrics (Step 1 complete → Step 2 complete)

---

### Risk 2: Database Connection Failures

**Severity**: Critical
**Likelihood**: Low
**Blast Radius**: All authentication features unavailable (no signup/signin)

**Mitigation**:
- **Connection Pool**: Use connection pooling (max 20 connections, min 5 idle)
- **Retry Logic**: Retry database queries on transient failures (max 3 retries, exponential backoff)
- **Health Checks**: `/health/ready` endpoint checks database connectivity
- **Fallback**: Display "Service temporarily unavailable" message to users

**Guardrails**:
- Alert if database connection failures detected
- Auto-restart backend service if connection pool exhausted

**Monitoring**: Database connection pool utilization, query latency, error rates

---

### Risk 3: Session Token Leakage

**Severity**: Critical
**Likelihood**: Very Low
**Blast Radius**: User account compromise (attacker gains access to user account)

**Mitigation**:
- **HTTP-Only Cookies**: Session tokens never accessible via JavaScript (prevents XSS attacks)
- **Secure Flag**: Cookies only sent over HTTPS (prevents man-in-the-middle attacks)
- **SameSite=Lax**: Cookies not sent on cross-site requests (prevents CSRF attacks)
- **Token Rotation**: Generate new token on every signin (invalidates old tokens)

**Guardrails**:
- Never log session tokens (redact in logs)
- Never expose tokens in API responses
- Expire sessions after inactivity (default 24 hours)

**Kill Switch**: If breach detected, invalidate all active sessions (`DELETE FROM sessions`)

**Monitoring**: Monitor for suspicious login patterns (multiple IPs for same user)

---

## 8. Evaluation and Validation

### Definition of Done

**Backend**:
- [ ] All API endpoints implemented (`/auth/signup`, `/auth/signin`, `/auth/signout`, `/auth/session`, `/background`)
- [ ] Database schema migrated (4 tables: users, sessions, software_background, hardware_background)
- [ ] Session validation middleware applied to protected endpoints
- [ ] Unit tests for authentication logic (>80% coverage)
- [ ] Integration tests for signup/signin flows (happy path + error cases)

**Frontend**:
- [ ] Signup form with email/password validation
- [ ] Background questions form (software + hardware)
- [ ] Signin form with "remember me" option
- [ ] PersonalizationContext extended with user session and background data
- [ ] Personalization button shows "Sign in to personalize" for anonymous users
- [ ] Unit tests for authentication forms (>80% coverage)

**End-to-End**:
- [ ] User can sign up, complete background questions, and log in automatically
- [ ] User can sign in, view personalized content, and sign out
- [ ] User can update background information via preferences page
- [ ] Session persists across page navigation
- [ ] Session expires after 24 hours (or 30 days with "remember me")

**Security**:
- [ ] Passwords hashed with bcrypt (salt rounds=10)
- [ ] Session tokens stored in HTTP-only cookies
- [ ] CSRF protection enabled (SameSite=Lax)
- [ ] SQL injection prevented (parameterized queries)
- [ ] XSS prevented (React escaping, no dangerouslySetInnerHTML)

**Performance**:
- [ ] Signup latency < 1000ms (p95)
- [ ] Signin latency < 500ms (p95)
- [ ] Session validation latency < 50ms (p95)
- [ ] Background data fetch latency < 300ms (p95)

**Documentation**:
- [ ] API documentation updated (OpenAPI spec)
- [ ] Environment variables documented (`.env.example`)
- [ ] Deployment runbook updated
- [ ] Architecture Decision Record (ADR) created for significant decisions

---

### Output Validation

**Functional Validation**:
1. **Signup**: Create account with valid email/password → Account created, session started, redirected to background questions
2. **Background Questions**: Complete software and hardware questions → Data saved, redirected to main content
3. **Signin**: Login with valid credentials → Session created, redirected to main content
4. **Session Persistence**: Navigate between pages → Session maintained (user stays logged in)
5. **Signout**: Click signout button → Session invalidated, redirected to signin page
6. **Personalization**: Click personalization button as logged-in user → Content personalized based on background

**Error Validation**:
1. **Duplicate Email**: Sign up with existing email → 409 error, "Email already registered" message
2. **Invalid Credentials**: Sign in with wrong password → 401 error, "Invalid email or password" message
3. **Expired Session**: Wait 24+ hours, make authenticated request → 401 error, redirected to signin page
4. **Missing Background**: Click personalization without completing background → Prompted to complete background questions

**Security Validation**:
1. **Password Hashing**: Check database → `password_hash` column contains bcrypt hash (not plaintext)
2. **Session Cookie**: Inspect browser cookies → `HttpOnly`, `Secure`, `SameSite=Lax` flags set
3. **SQL Injection**: Attempt SQL injection in email field → Query rejected, no database error
4. **XSS**: Attempt XSS in background questions → Input escaped, no script execution

---

## 9. Implementation Tasks and Dependencies

### Phase 1: Database Setup

**Tasks**:
1. Set up Neon DB instance (if not already exists)
2. Run BetterAuth migrations (create `users` and `sessions` tables)
3. Run custom migrations (create `software_background` and `hardware_background` tables)
4. Create indexes (email, token, user_id, arrays)
5. Verify schema with validation queries

**Dependencies**: None (can start immediately)

**Estimated Effort**: 2-3 hours

**Acceptance Criteria**:
- All 4 tables exist with correct columns and constraints
- Indexes created for performance
- No orphaned records (validation query returns 0)

---

### Phase 2: Backend API - Authentication Endpoints

**Tasks**:
1. Install BetterAuth backend library
2. Configure BetterAuth with Neon DB connection
3. Implement `/auth/signup` endpoint (account creation)
4. Implement `/auth/signin` endpoint (credential validation)
5. Implement `/auth/signout` endpoint (session invalidation)
6. Implement `/auth/session` endpoint (session retrieval)
7. Add session validation middleware
8. Write unit tests for authentication logic
9. Write integration tests for endpoints

**Dependencies**: Phase 1 (database schema)

**Estimated Effort**: 1-2 days

**Acceptance Criteria**:
- All endpoints return correct responses (tested with Postman/curl)
- Session cookies set correctly (HttpOnly, Secure, SameSite=Lax)
- Unit tests pass (>80% coverage)
- Integration tests pass (happy path + error cases)

---

### Phase 3: Backend API - Background Data Endpoints

**Tasks**:
1. Implement `POST /background` endpoint (create background data)
2. Implement `GET /background` endpoint (fetch background data)
3. Implement `PUT /background` endpoint (update background data)
4. Add ownership validation (user can only access own background)
5. Add input validation (experience level, array values)
6. Write unit tests for background logic
7. Write integration tests for endpoints

**Dependencies**: Phase 2 (authentication endpoints for session validation)

**Estimated Effort**: 1 day

**Acceptance Criteria**:
- Background data created, fetched, and updated successfully
- Ownership validation prevents cross-user access
- Input validation rejects invalid data
- Tests pass (>80% coverage)

---

### Phase 4: Frontend - Signup Flow

**Tasks**:
1. Install BetterAuth React library
2. Create `SignupForm` component (email/password fields)
3. Add client-side validation (email format, password strength)
4. Create `BackgroundQuestionsForm` component (software + hardware)
5. Add form state management (Step 1 → Step 2 flow)
6. Integrate with `/auth/signup` and `/background` endpoints
7. Add error handling (display validation errors, network errors)
8. Write unit tests for forms

**Dependencies**: Phase 2 (backend authentication endpoints)

**Estimated Effort**: 1-2 days

**Acceptance Criteria**:
- Users can complete signup and background questions
- Validation errors displayed inline
- Network errors shown as toast notifications
- Form tests pass (>80% coverage)

---

### Phase 5: Frontend - Signin Flow

**Tasks**:
1. Create `SigninForm` component (email/password, remember me)
2. Add client-side validation
3. Integrate with `/auth/signin` endpoint
4. Add error handling (invalid credentials, network errors)
5. Redirect to main content on successful signin
6. Write unit tests for signin form

**Dependencies**: Phase 2 (backend authentication endpoints)

**Estimated Effort**: 0.5-1 day

**Acceptance Criteria**:
- Users can sign in with valid credentials
- Invalid credentials show error message
- Remember me extends session to 30 days
- Tests pass (>80% coverage)

---

### Phase 6: Frontend - PersonalizationContext Extension

**Tasks**:
1. Extend `PersonalizationContext` to include BetterAuth `useSession()` hook
2. Add `userBackground` state to context
3. Fetch background data on login (call `GET /background`)
4. Cache background data in context
5. Update `PersonalizationButton` to check authentication state
6. Show "Sign in to personalize" for anonymous users
7. Write unit tests for context

**Dependencies**: Phase 3 (background data endpoints), Phase 4 (signup flow)

**Estimated Effort**: 1 day

**Acceptance Criteria**:
- PersonalizationContext includes user session and background
- Background data fetched once on login and cached
- Personalization button shows correct state (authenticated vs. anonymous)
- Tests pass (>80% coverage)

---

### Phase 7: End-to-End Testing

**Tasks**:
1. Write E2E test for signup flow (create account → complete background → auto-login)
2. Write E2E test for signin flow (login → navigate → personalize)
3. Write E2E test for session persistence (refresh page, navigate)
4. Write E2E test for signout flow (sign out → redirected to signin)
5. Write E2E test for error scenarios (duplicate email, invalid credentials)
6. Run all E2E tests in CI pipeline

**Dependencies**: Phase 4, 5, 6 (all frontend and backend implementation complete)

**Estimated Effort**: 1 day

**Acceptance Criteria**:
- All E2E tests pass
- Tests run automatically in CI/CD pipeline
- Smoke tests pass before production deployment

---

### Phase 8: Deployment and Monitoring

**Tasks**:
1. Update environment variables (`.env` for backend, build config for frontend)
2. Deploy database migrations to production
3. Deploy backend API to production
4. Deploy frontend to production
5. Run smoke tests (create test account, sign in, personalize)
6. Set up monitoring (logs, metrics, alerts)
7. Enable feature flag `auth_enabled`

**Dependencies**: Phase 7 (all tests passing)

**Estimated Effort**: 0.5-1 day

**Acceptance Criteria**:
- Deployment successful (no errors)
- Smoke tests pass
- Monitoring dashboards show healthy metrics
- Feature flag enabled, authentication system live

---

## 10. Architectural Decision Record (ADR) Suggestions

Based on this plan, the following architectural decisions meet the significance criteria (impact, alternatives, scope):

### ADR 1: Multi-Step Signup Flow vs. Single-Step Form

**Context**: Need to collect user credentials and background information during signup

**Decision**: Implement multi-step signup flow (Step 1: Credentials → Step 2: Background Questions)

**Rationale**: Reduces cognitive load, improves mobile UX, aligns with industry best practices

**Alternatives Considered**: Single-step form, post-signup optional questions

**Consequences**: Requires state management between steps, potential abandonment at Step 2

**Document with**: `/sp.adr multi-step-signup-flow`

---

### ADR 2: Normalized Database Schema for Background Data

**Context**: Need to store user software and hardware background information

**Decision**: Use normalized relational schema with separate `software_background` and `hardware_background` tables

**Rationale**: Type-safe, efficient queries, clear separation of concerns, PostgreSQL array types

**Alternatives Considered**: JSONB storage, single background table

**Consequences**: More complex joins, but better query performance and data integrity

**Document with**: `/sp.adr normalized-background-schema`

---

### ADR 3: Extend PersonalizationContext for Authentication

**Context**: Need to integrate user authentication state with existing personalization features

**Decision**: Extend existing PersonalizationContext to include BetterAuth session and background data

**Rationale**: Single source of truth, reduces state sync issues, minimal changes to existing code

**Alternatives Considered**: Separate AuthContext, global state management (Redux/Zustand)

**Consequences**: Mixed responsibilities in context, but simpler implementation

**Document with**: `/sp.adr extend-personalization-context-for-auth`

---

## 11. Follow-Ups and Risks

### Follow-Up 1: Password Reset Flow

**Why Not Included**: Complexity and scope (email service integration, token expiration, security)

**When to Implement**: After MVP validated, user requests for password reset feature

**Effort Estimate**: 2-3 days (email service, reset token generation, UI)

---

### Follow-Up 2: Email Verification

**Why Not Included**: Not required for MVP, adds friction to signup flow

**When to Implement**: If spam accounts become an issue, or if required by regulations

**Effort Estimate**: 1-2 days (email service, verification token, UI)

---

### Follow-Up 3: Social OAuth Providers (Google, GitHub)

**Why Not Included**: Not in scope for MVP (spec requires email/password only)

**When to Implement**: If user feedback shows demand for social login

**Effort Estimate**: 1-2 days per provider (OAuth flow, BetterAuth integration)

---

### Risk: PersonalizationContext Performance

**Issue**: Adding authentication state to PersonalizationContext may cause excessive re-renders

**Mitigation**: Use React.memo for child components, useMemo for derived state

**Monitoring**: React DevTools profiler to track re-render frequency

**Rollback**: Split into separate contexts if performance degrades

---

## Conclusion

This architectural plan provides a comprehensive roadmap for implementing BetterAuth signup and signin with user background collection. The multi-step signup flow balances conversion optimization with data collection needs. The normalized database schema enables efficient personalization queries while maintaining data integrity. Integration with existing personalization features is minimally invasive via PersonalizationContext extension.

**Next Steps**:
1. Review plan with team and stakeholders
2. Get approval for architectural decisions
3. Create Architecture Decision Records (ADRs) for significant decisions
4. Begin implementation with Phase 1 (Database Setup)
5. Track progress with task list and update as needed

**Plan Approval**: Ready for implementation after team review and ADR documentation.
