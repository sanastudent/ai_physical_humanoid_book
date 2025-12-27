---
id: 0027
title: BetterAuth Completion Spec
stage: spec
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-betterauth-completion
branch: 003-betterauth-completion
user: User
command: /sp.specify
labels: [feature, spec, auth, betterauth, security]
links:
  spec: specs/003-betterauth-completion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-betterauth-completion/spec.md
  - specs/003-betterauth-completion/checklists/requirements.md
tests:
  - specs/003-betterauth-completion/checklists/requirements.md
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.specify` in the triggering message **is** the feature description. Assume you always have it available in this conversation even if `$ARGUMENTS` appears literally below. Do not ask the user to repeat it unless they provided an empty command.

Given that feature description, do this:

1. **Generate a concise short name** (2-4 words) for the branch:
   - Analyze the feature description and extract the most meaningful keywords
   - Create a 2-4 word short name that captures the essence of the feature
   - Use action-noun format when possible (e.g., "add-user-auth", "fix-payment-bug")
   - Preserve technical terms and acronyms (OAuth2, API, JWT, etc.)
   - Keep it concise but descriptive enough to understand the feature at a glance
   - Examples:
     - "I want to add user authentication" → "user-auth"
     - "Implement OAuth2 integration for the API" → "oauth2-api-integration"
     - "Create a dashboard for analytics" → "analytics-dashboard"
     - "Fix payment processing timeout bug" → "fix-payment-timeout"

2. **Check for existing branches before creating new one**:

   a. First, fetch all remote branches to ensure we have the latest information:
      ```bash
      git fetch --all --prune
      ```

   b. Find the highest feature number across all sources for the short-name:
      - Remote branches: `git ls-remote --heads origin | grep -E 'refs/heads/[0-9]+-<short-name>$'`
      - Local branches: `git branch | grep -E '^[* ]*[0-9]+-<short-name>$'`
      - Specs directories: Check for directories matching `specs/[0-9]+-<short-name>`

   c. Determine the next available number:
      - Extract all numbers from all three sources
      - Find the highest number N
      - Use N+1 for the new branch number

   d. Run the script `.specify/scripts/powershell/create-new-feature.ps1 -Json "$ARGUMENTS"` with the calculated number and short-name:
      - Pass `--number N+1` and `--short-name "your-short-name"` along with the feature description
      - Bash example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "$ARGUMENTS" --json --number 5 --short-name "user-auth" "Add user authentication"`
      - PowerShell example: `.specify/scripts/powershell/create-new-feature.ps1 -Json "$ARGUMENTS" -Json -Number 5 -ShortName "user-auth" "Add user authentication"`

   **IMPORTANT**:
   - Check all three sources (remote branches, local branches, specs directories) to find the highest number
   - Only match branches/directories with the exact short-name pattern
   - If no existing branches/directories found with this short-name, start with number 1
   - You must only ever run this script once per feature
   - The JSON is provided in the terminal as output - always refer to it to get the actual content you're looking for
   - The JSON output will contain BRANCH_NAME and SPEC_FILE paths
   - For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot")

3. Load `.specify/templates/spec-template.md` to understand required sections.

4. Follow this execution flow:

    1. Parse user description from Input
       If empty: ERROR "No feature description provided"
    2. Extract key concepts from description
       Identify: actors, actions, data, constraints
    3. For unclear aspects:
       - Make informed guesses based on context and industry standards
       - Only mark with [NEEDS CLARIFICATION: specific question] if:
         - The choice significantly impacts feature scope or user experience
         - Multiple reasonable interpretations exist with different implications
         - No reasonable default exists
       - **LIMIT: Maximum 3 [NEEDS CLARIFICATION] markers total**
       - Prioritize clarifications by impact: scope > security/privacy > user experience > technical details
    4. Fill User Scenarios & Testing section
       If no clear user flow: ERROR "Cannot determine user scenarios"
    5. Generate Functional Requirements
       Each requirement must be testable
       Use reasonable defaults for unspecified details (document assumptions in Assumptions section)
    6. Define Success Criteria
       Create measurable, technology-agnostic outcomes
       Include both quantitative metrics (time, performance, volume) and qualitative measures (user satisfaction, task completion)
       Each criterion must be verifiable without implementation details
    7. Identify Key Entities (if data involved)
    8. Return: SUCCESS (spec ready for planning)

5. Write the specification to SPEC_FILE using the template structure, replacing placeholders with concrete details derived from the feature description (arguments) while preserving section order and headings.

6. **Specification Quality Validation**: After writing the initial spec, validate it against quality criteria:

   a. **Create Spec Quality Checklist**: Generate a checklist file at `FEATURE_DIR/checklists/requirements.md` using the checklist template structure with these validation items:

      ```markdown
      # Specification Quality Checklist: [FEATURE NAME]

      **Purpose**: Validate specification completeness and quality before proceeding to planning
      **Created**: [DATE]
      **Feature**: [Link to spec.md]

      ## Content Quality

      - [ ] No implementation details (languages, frameworks, APIs)
      - [ ] Focused on user value and business needs
      - [ ] Written for non-technical stakeholders
      - [ ] All mandatory sections completed

      ## Requirement Completeness

      - [ ] No [NEEDS CLARIFICATION] markers remain
      - [ ] Requirements are testable and unambiguous
      - [ ] Success criteria are measurable
      - [ ] Success criteria are technology-agnostic (no implementation details)
      - [ ] All acceptance scenarios are defined
      - [ ] Edge cases are identified
      - [ ] Scope is clearly bounded
      - [ ] Dependencies and assumptions identified

      ## Feature Readiness

      - [ ] All functional requirements have clear acceptance criteria
      - [ ] User scenarios cover primary flows
      - [ ] Feature meets measurable outcomes defined in Success Criteria
      - [ ] No implementation details leak into specification

      ## Notes

      - Items marked incomplete require spec updates before `/sp.clarify` or `/sp.plan`
      ```

   b. **Run Validation Check**: Review the spec against each checklist item:
      - For each item, determine if it passes or fails
      - Document specific issues found (quote relevant spec sections)

   c. **Handle Validation Results**:

      - **If all items pass**: Mark checklist complete and proceed to step 6

      - **If items fail (excluding [NEEDS CLARIFICATION])**:
        1. List the failing items and specific issues
        2. Update the spec to address each issue
        3. Re-run validation until all items pass (max 3 iterations)
        4. If still failing after 3 iterations, document remaining issues in checklist notes and warn user

      - **If [NEEDS CLARIFICATION] markers remain**:
        1. Extract all [NEEDS CLARIFICATION: ...] markers from the spec
        2. **LIMIT CHECK**: If more than 3 markers exist, keep only the 3 most critical (by scope/security/UX impact) and make informed guesses for the rest
        3. For each clarification needed (max 3), present options to user in this format:

           ```markdown
           ## Question [N]: [Topic]

           **Context**: [Quote relevant spec section]

           **What we need to know**: [Specific question from NEEDS CLARIFICATION marker]

           **Suggested Answers**:

           | Option | Answer | Implications |
           |--------|--------|--------------|
           | A      | [First suggested answer] | [What this means for the feature] |
           | B      | [Second suggested answer] | [What this means for the feature] |
           | C      | [Third suggested answer] | [What this means for the feature] |
           | Custom | Provide your own answer | [Explain how to provide custom input] |

           **Your choice**: _[Wait for user response]_
           ```

        4. **CRITICAL - Table Formatting**: Ensure markdown tables are properly formatted:
           - Use consistent spacing with pipes aligned
           - Each cell should have spaces around content: `| Content |` not `|Content|`
           - Header separator must have at least 3 dashes: `|--------|`
           - Test that the table renders correctly in markdown preview
        5. Number questions sequentially (Q1, Q2, Q3 - max 3 total)
        6. Present all questions together before waiting for responses
        7. Wait for user to respond with their choices for all questions (e.g., "Q1: A, Q2: Custom - [details], Q3: B")
        8. Update the spec by replacing each [NEEDS CLARIFICATION] marker with the user's selected or provided answer
        9. Re-run validation after all clarifications are resolved

   d. **Update Checklist**: After each validation iteration, update the checklist file with current pass/fail status

7. Report completion with branch name, spec file path, checklist results, and readiness for the next phase (`/sp.clarify` or `/sp.plan`).

**NOTE:** The script creates and checks out the new branch and initializes the spec file before writing.

## General Guidelines

## Quick Guidelines

- Focus on **WHAT** users need and **WHY**.
- Avoid HOW to implement (no tech stack, APIs, code structure).
- Written for business stakeholders, not developers.
- DO NOT create any checklists that are embedded in the spec. That will be a separate command.

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation

When creating this spec from a user prompt:

1. **Make informed guesses**: Use context, industry standards, and common patterns to fill gaps
2. **Document assumptions**: Record reasonable defaults in the Assumptions section
3. **Limit clarifications**: Maximum 3 [NEEDS CLARIFICATION] markers - use only for critical decisions that:
   - Significantly impact feature scope or user experience
   - Have multiple reasonable interpretations with different implications
   - Lack any reasonable default
4. **Prioritize clarifications**: scope > security/privacy > user experience > technical details
5. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
6. **Common areas needing clarification** (only if no reasonable default exists):
   - Feature scope and boundaries (include/exclude specific use cases)
   - User types and permissions (if multiple conflicting interpretations possible)
   - Security/compliance requirements (when legally/financially significant)

**Examples of reasonable defaults** (don't ask about these):

- Data retention: Industry-standard practices for the domain
- Performance targets: Standard web/mobile app expectations unless specified
- Error handling: User-friendly messages with appropriate fallbacks
- Authentication method: Standard session-based or OAuth2 for web apps
- Integration patterns: RESTful APIs unless specified otherwise

### Success Criteria Guidelines

Success criteria must be:

1. **Measurable**: Include specific metrics (time, percentage, count, rate)
2. **Technology-agnostic**: No mention of frameworks, languages, databases, or tools
3. **User-focused**: Describe outcomes from user/business perspective, not system internals
4. **Verifiable**: Can be tested/validated without knowing implementation details

**Good examples**:

- "Users can complete checkout in under 3 minutes"
- "System supports 10,000 concurrent users"
- "95% of searches return results in under 1 second"
- "Task completion rate improves by 40%"

**Bad examples** (implementation-focused):

- "API response time is under 200ms" (too technical, use "Users see results instantly")
- "Database can handle 1000 TPS" (implementation detail, use user-facing metric)
- "React components render efficiently" (framework-specific)
- "Redis cache hit rate above 80%" (technology-specific)

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

# Feature Specification: BetterAuth Integration Completion

**Feature Branch**: `003-betterauth-completion`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "Complete BetterAuth integration to address gaps identified in analysis. Ensure backend endpoints are fully compatible with BetterAuth frontend library, implement proper session management, and verify all authentication flows work correctly with BetterAuth standards."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - BetterAuth-Compatible Authentication Flow (Priority: P1)

A user interacts with the authentication system expecting standard BetterAuth behavior. The backend must properly support BetterAuth's session management, user verification, and authentication flows to ensure seamless frontend integration.

**Why this priority**: This is the foundational requirement to ensure the backend is truly BetterAuth-compatible as specified in the original requirements. Without proper BetterAuth integration, the authentication system doesn't meet the specified requirements.

**Independent Test**: Can be fully tested by using BetterAuth frontend components to signup, signin, and verify sessions, confirming that all authentication flows work as expected with BetterAuth standards.

**Acceptance Scenarios**:

1. **Given** a user attempts to signup via BetterAuth frontend components, **When** they submit valid credentials, **Then** the backend properly processes the request and creates a user account following BetterAuth standards
2. **Given** a user attempts to signin via BetterAuth frontend components, **When** they submit valid credentials, **Then** the backend properly authenticates and establishes a BetterAuth-compatible session
3. **Given** a user has an active session, **When** they make authenticated requests, **Then** the backend properly validates their BetterAuth session tokens
4. **Given** a user attempts to signout, **When** they trigger the signout flow, **Then** the backend properly invalidates their BetterAuth session

---

### User Story 2 - Session Management and Security (Priority: P1)

A logged-in user expects their session to be managed securely according to BetterAuth standards, with proper token handling, expiration, and security measures implemented.

**Why this priority**: Security and proper session management are critical for user trust and system integrity. BetterAuth provides specific security patterns that must be followed to ensure secure authentication.

**Independent Test**: Can be tested by creating a session, verifying token security, testing session expiration, and confirming secure session handling according to BetterAuth security standards.

**Acceptance Scenarios**:

1. **Given** a user successfully authenticates, **When** a session is created, **Then** the session token follows BetterAuth security standards (HttpOnly, Secure, SameSite flags)
2. **Given** a user has an active session, **When** the session reaches expiration time, **Then** the session is properly invalidated and requires re-authentication
3. **Given** a user makes authenticated requests, **When** the system validates their session, **Then** the session validation follows BetterAuth's security protocols
4. **Given** a security event occurs (potential breach, suspicious activity), **When** an admin needs to invalidate sessions, **Then** the system can properly terminate BetterAuth sessions for specific users

---

### User Story 3 - API Endpoint Compatibility (Priority: P2)

A frontend application using BetterAuth expects to communicate with backend endpoints that follow BetterAuth's API contract standards for authentication operations.

**Why this priority**: API compatibility ensures that frontend components can properly interact with the backend using standard BetterAuth patterns, reducing integration complexity and ensuring maintainability.

**Independent Test**: Can be tested by making API calls using BetterAuth's standard endpoints and verifying that the backend responds with the expected data structures and follows BetterAuth's API contract.

**Acceptance Scenarios**:

1. **Given** a frontend makes BetterAuth-compatible API calls, **When** requests are sent to authentication endpoints, **Then** the backend responds with properly formatted BetterAuth-compliant responses
2. **Given** a user needs to verify their authentication status, **When** they call the session verification endpoint, **Then** the backend returns proper user session information in BetterAuth format
3. **Given** an authenticated user makes requests, **When** they include BetterAuth session tokens, **Then** the backend properly processes these tokens according to BetterAuth standards
4. **Given** an API error occurs, **When** the backend returns error responses, **Then** errors follow BetterAuth's error handling patterns and data structures

---

### User Story 4 - Migration from Custom Auth to BetterAuth (Priority: P2)

An existing user with an account created through the custom authentication system needs to have their account properly integrated with BetterAuth's user management system.

**Why this priority**: Ensures backward compatibility and smooth transition for existing users when BetterAuth integration is completed, preventing account loss or access issues.

**Independent Test**: Can be tested by verifying existing accounts are properly recognized by BetterAuth systems and that user data is accessible through BetterAuth's user management interfaces.

**Acceptance Scenarios**:

1. **Given** an existing user account created with custom auth, **When** BetterAuth integration is completed, **Then** the account is accessible through BetterAuth's user management
2. **Given** a user had a session with the custom system, **When** they attempt to use BetterAuth, **Then** they can either continue with new BetterAuth session or migrate appropriately
3. **Given** existing user data exists in Neon DB, **When** BetterAuth integration is complete, **Then** user data remains accessible through BetterAuth's data interfaces

---

### Edge Cases

- What happens when a user tries to authenticate but the BetterAuth configuration is incomplete? (System should provide clear error messages and guidance)
- How does the system handle BetterAuth API rate limits or service outages? (Implement fallback mechanisms or clear user notifications)
- What if BetterAuth's data model conflicts with existing user data? (Implement proper data migration or mapping)
- How are concurrent sessions handled with BetterAuth? (Follow BetterAuth's session management policies)
- What happens during the transition period when both custom auth and BetterAuth might be partially active? (Ensure clear separation and migration path)
- How does the system handle users who signed up with custom auth but need to transition to BetterAuth? (Provide migration workflow)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-039**: System MUST implement BetterAuth-compatible backend endpoints that support standard authentication flows (signup, signin, signout, session verification)
- **FR-040**: System MUST handle BetterAuth session tokens with proper security flags (HttpOnly, Secure, SameSite=Lax) as per BetterAuth standards
- **FR-041**: System MUST validate user sessions using BetterAuth's authentication mechanisms rather than custom validation
- **FR-042**: System MUST store user credentials and session data in a format compatible with BetterAuth's data expectations
- **FR-043**: System MUST provide API endpoints that return user data in BetterAuth-compliant formats
- **FR-044**: System MUST implement proper error handling that follows BetterAuth's error response patterns
- **FR-045**: System MUST support BetterAuth's password hashing and verification mechanisms
- **FR-046**: System MUST integrate with BetterAuth's user management APIs for user creation, updates, and deletion
- **FR-047**: System MUST maintain backward compatibility for existing user accounts during the transition
- **FR-048**: System MUST provide configuration options to properly connect with BetterAuth frontend components
- **FR-049**: System MUST implement proper session timeout and refresh mechanisms according to BetterAuth standards
- **FR-050**: System MUST validate all authentication requests against BetterAuth's security requirements

### Key Entities *(include if feature involves data)*

- **BetterAuth User**: User account that follows BetterAuth's data model and authentication standards, including proper ID, email, and metadata fields
- **BetterAuth Session**: Authentication session that follows BetterAuth's token format and security requirements with proper expiration and validation
- **BetterAuth Configuration**: Backend configuration settings that enable proper communication with BetterAuth frontend components
- **User Migration Record**: Temporary data structure to track users transitioning from custom auth to BetterAuth system
- **Authentication Token**: Secure token that follows BetterAuth's format and security standards for session management

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-012**: BetterAuth frontend components can successfully authenticate users with 100% compatibility
- **SC-013**: Authentication requests complete within 1.5 seconds under normal load conditions
- **SC-014**: Session validation occurs in under 500ms with 99.9% success rate
- **SC-015**: 100% of existing user accounts remain accessible after BetterAuth integration
- **SC-016**: Password reset and account recovery flows work according to BetterAuth standards
- **SC-017**: Security audit confirms no vulnerabilities in authentication implementation
- **SC-018**: 99% of authentication requests result in successful user sessions
- **SC-019**: Session tokens remain valid for the configured duration without unexpected expiration
- **SC-020**: Error handling provides clear messages to users without exposing system internals
- **SC-021**: Migration from custom auth to BetterAuth completes successfully for all existing accounts

## Assumptions *(optional)*

- BetterAuth's backend adapter can be properly configured to work with Neon DB
- Existing user data can be migrated to BetterAuth's expected format without loss
- BetterAuth's security standards align with the project's security requirements
- The frontend can be updated to use BetterAuth's frontend components without major refactoring
- BetterAuth's session management approach is compatible with the existing application architecture
- Network connectivity between frontend and backend supports BetterAuth's authentication flows
- BetterAuth's API remains stable during the integration period

## Constraints *(optional)*

- Must maintain BetterAuth's security standards and best practices throughout implementation
- All existing user accounts must remain accessible during and after the transition
- Authentication performance must not degrade below current levels during transition
- Backend must remain compatible with existing frontend components during migration
- User session data must not be lost during the BetterAuth integration process
- Implementation must follow BetterAuth's official documentation and best practices
- Migration process must be reversible if issues are discovered during implementation

## Dependencies *(optional)*

- **BetterAuth Backend Adapter**: Backend component that provides BetterAuth compatibility with Neon DB
- **BetterAuth Frontend Components**: Frontend libraries that communicate with the BetterAuth-compatible backend
- **Neon DB Schema**: Database schema must support BetterAuth's expected user and session data structures
- **Existing User Data**: Current user accounts and session data that need to be migrated or integrated
- **Frontend Authentication Components**: UI components that will interact with BetterAuth-compatible endpoints
- **Security Configuration**: SSL/TLS setup and other security infrastructure required by BetterAuth

## Outcome

- ✅ Impact: Created comprehensive specification for BetterAuth integration completion addressing gaps identified in analysis
- 🧪 Tests: Defined testable acceptance scenarios for all user stories
- 📁 Files: Created spec.md and requirements checklist in specs/003-betterauth-completion/
- 🔁 Next prompts: Ready for planning phase with clear requirements and success criteria
- 🧠 Reflection: Specification addresses the BetterAuth requirement gap identified in previous analysis

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase with this specification