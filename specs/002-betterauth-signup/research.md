# Research: BetterAuth Integration & User Background Collection

**Feature**: 002-betterauth-signup
**Date**: 2025-12-11
**Purpose**: Technical research and decision documentation for authentication implementation

## Research Questions

### 1. BetterAuth Integration Pattern
**Question**: How should BetterAuth be integrated into the existing Docusaurus frontend?

**Research Findings**:
- BetterAuth is a full-stack authentication library for TypeScript applications
- Supports email/password, OAuth providers, session management
- Provides React hooks for frontend integration (`useSession`, `signIn`, `signUp`, `signOut`)
- Requires both client-side (React components) and server-side (API routes) setup
- Session management via HTTP-only cookies (secure by default)

**Decision**: Use BetterAuth React hooks for signup/signin forms with dedicated API routes

**Rationale**:
- Native React integration fits existing Docusaurus/React stack
- HTTP-only cookie sessions provide security without localStorage vulnerabilities
- Built-in CSRF protection and secure password hashing (bcrypt)
- Minimal boilerplate compared to custom authentication implementation

**Alternatives Considered**:
- NextAuth.js: More Next.js-specific, less suitable for Docusaurus
- Custom JWT implementation: More complex, security risks, violates constraint to use BetterAuth
- Auth0/Clerk: Third-party SaaS, additional cost and external dependency

---

### 2. Neon DB Schema Design
**Question**: What database schema structure should be used for user accounts and background data?

**Research Findings**:
- Neon DB is a serverless Postgres database
- Supports standard PostgreSQL features (JSONB, constraints, indexes)
- BetterAuth provides schema recommendations for user/session tables
- Background questions can be stored as structured JSONB or normalized tables

**Decision**: Use normalized relational schema with separate tables for users, software_background, and hardware_background

**Rationale**:
- Structured data enables efficient querying for personalization
- Type safety and validation at database level
- Easy to add new background questions without schema migration
- Clear data model for frontend forms

**Schema Structure**:
```sql
-- Core BetterAuth tables (managed by library)
users (id, email, password_hash, created_at, updated_at)
sessions (id, user_id, token, expires_at, created_at)

-- Custom background tables
software_background (
  id,
  user_id (FK → users.id),
  experience_level (enum: beginner/intermediate/advanced),
  preferred_languages (text[]),
  preferred_frameworks (text[]),
  created_at,
  updated_at
)

hardware_background (
  id,
  user_id (FK → users.id),
  experience_level (enum: beginner/intermediate/advanced),
  preferred_platforms (text[]),
  device_types (text[]),
  created_at,
  updated_at
)
```

**Alternatives Considered**:
- JSONB storage: Less structured, harder to query specific fields
- Single background table with all fields: Harder to extend, mixed concerns
- Document database (MongoDB): Violates constraint to use Neon DB (Postgres)

---

### 3. Frontend Form Flow
**Question**: Should background questions be collected during signup or as a separate post-signup step?

**Research Findings**:
- Industry best practices suggest minimizing signup friction
- Multi-step forms reduce abandonment vs. long single forms
- BetterAuth supports custom user data during registration

**Decision**: Multi-step signup flow - Step 1: Credentials, Step 2: Background Questions

**Rationale**:
- Reduces initial cognitive load (email/password only first)
- Allows users to complete authentication before committing to questions
- Can mark background as "incomplete" and prompt later if abandoned
- Better mobile UX with focused steps

**Flow**:
1. User enters email/password → Account created + auto-login
2. Immediately redirected to background questions form
3. On submit → Save background data + redirect to main content
4. If skipped → Show reminder banner on personalization button

**Alternatives Considered**:
- Single-step form: Higher abandonment rate, poor mobile experience
- Post-signup optional: May never complete, reduces personalization value
- Social OAuth only: Not in scope for MVP, violates email/password requirement

---

### 4. Session Management & Personalization Integration
**Question**: How should the existing personalization button detect authenticated users?

**Research Findings**:
- BetterAuth provides `useSession()` hook returning current user or null
- Existing personalization button likely checks for user context
- Background data needs to be fetched when personalization is triggered

**Decision**: Extend existing PersonalizationContext to include user session from BetterAuth

**Rationale**:
- Minimal changes to existing personalization logic
- Centralized user state management via React Context
- PersonalizationContext can fetch background data on mount if user is logged in
- Personalization button simply checks `user !== null` from context

**Implementation Pattern**:
```javascript
// PersonalizationContext.tsx (extended)
const { session } = useSession(); // BetterAuth hook
const [userBackground, setUserBackground] = useState(null);

useEffect(() => {
  if (session?.user) {
    fetchUserBackground(session.user.id).then(setUserBackground);
  }
}, [session]);

return (
  <PersonalizationContext.Provider value={{ user: session?.user, background: userBackground }}>
    {children}
  </PersonalizationContext.Provider>
);
```

**Alternatives Considered**:
- Separate AuthContext + PersonalizationContext: Redundant state, harder to sync
- Fetch background on every personalization click: Slower UX, unnecessary API calls
- Store background in session cookie: Data too large, security concerns

---

### 5. Error Handling & Edge Cases
**Question**: How should validation errors, network failures, and edge cases be handled?

**Research Findings**:
- BetterAuth provides error codes for common auth failures
- Neon DB connection failures need graceful degradation
- Form validation should happen both client and server-side

**Decision**: Client-side validation with toast notifications + server-side enforcement with error responses

**Error Handling Strategy**:
- **Validation Errors**: Inline form errors with field-specific messages
- **Duplicate Email**: BetterAuth returns specific error, show "Email already registered" message
- **Network Failures**: Toast notification with retry button
- **DB Connection Issues**: Generic error message, log to backend for debugging
- **Partial Data**: Wrap signup in transaction, rollback on failure

**Client Validation Rules**:
- Email: Valid format (RFC 5322)
- Password: Min 8 chars, 1 uppercase, 1 lowercase, 1 number
- Background questions: At least one language and platform selected

**Alternatives Considered**:
- Server-only validation: Poor UX, slow feedback
- No transaction safety: Risk of partial user records
- Alert dialogs for errors: Intrusive, poor accessibility

---

## Technology Stack Summary

| Component | Technology | Justification |
|-----------|------------|---------------|
| Authentication | BetterAuth | Required by constraints, full-stack TypeScript solution |
| Database | Neon DB (Postgres) | Required by constraints, serverless Postgres |
| Frontend Forms | React Hook Form | Industry standard, validation, minimal re-renders |
| State Management | React Context | Existing pattern, sufficient for auth state |
| API Layer | FastAPI (Python) | Existing backend, extends current API |
| Session Storage | HTTP-only cookies | BetterAuth default, secure, CSRF-protected |
| Password Hashing | bcrypt | BetterAuth default, industry standard |
| Validation | Zod | Type-safe validation, integrates with React Hook Form |

---

## Performance Considerations

**Session Lookup**:
- BetterAuth uses indexed session tokens
- Expected latency: <50ms for session validation
- Caching strategy: Session cached in memory for duration

**Background Data Fetch**:
- Fetched once on login, cached in React Context
- Lazy load on first personalization click as fallback
- Expected latency: <100ms for user background query

**Signup Transaction**:
- Single transaction for user + background inserts
- Expected latency: <500ms for full signup flow
- Rollback on any failure ensures consistency

---

## Security Considerations

**Password Storage**:
- bcrypt with salt rounds=10 (BetterAuth default)
- Never stored in plaintext or reversible encryption

**Session Security**:
- HTTP-only cookies prevent XSS access
- Secure flag for HTTPS-only transmission
- SameSite=Lax prevents CSRF attacks

**Input Validation**:
- All user input sanitized server-side
- SQL injection prevented via parameterized queries (Postgres)
- XSS prevented via React's built-in escaping

**Rate Limiting** (Future Enhancement):
- Signup: Max 5 attempts per IP per hour
- Signin: Max 10 attempts per email per 15 minutes
- Not in MVP scope but documented for later

---

## Deployment Considerations

**Environment Variables Required**:
- `BETTERAUTH_SECRET`: Random string for session encryption
- `NEON_DB_URL`: Neon database connection string
- `FRONTEND_URL`: CORS configuration for API

**Database Migrations**:
- Use Prisma/Drizzle for schema migrations
- BetterAuth provides migration scripts for user/session tables
- Custom migrations for software_background and hardware_background tables

**Backward Compatibility**:
- Existing personalization code works without auth (no breaking changes)
- Personalization button shows "Sign in to personalize" for anonymous users
- All existing routes remain accessible without authentication

---

## Open Questions & Future Enhancements

**Resolved for MVP**:
- ✅ Password reset flow → Documented as future enhancement
- ✅ Email verification → Not required for MVP
- ✅ Social OAuth → Not in scope

**Future Considerations** (Not MVP):
- Password reset via email tokens
- Email verification for signup
- OAuth providers (Google, GitHub)
- Two-factor authentication
- Remember me checkbox (30-day sessions)
- Profile picture uploads
- Account deletion flow

---

## Conclusion

All technical decisions have been documented with clear rationale. The approach leverages BetterAuth's built-in security and session management while extending the database schema to support user background collection. The multi-step signup flow balances conversion optimization with data collection needs. Integration with existing personalization features is minimally invasive via React Context extension.

**Ready for Phase 1**: Data model and API contracts can now be designed based on these research findings.
