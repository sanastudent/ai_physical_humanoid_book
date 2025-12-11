# Feature Specification: Chapter Translation to Urdu

**Feature Branch**: `001-urdu-translation`
**Created**: 2025-12-11
**Status**: Draft
**Input**: User description: "Logged-in users can translate any chapter into Urdu by pressing a button. Translation uses Docusaurus i18n (ur locale). Translation toggle appears at the top of each chapter. On click, page switches between 'english' and 'urdu' versions."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Toggle Chapter Translation (Priority: P1)

A logged-in user reading any chapter wants to view the content in Urdu to better understand the material in their preferred language. They click a translation toggle button at the top of the chapter, and the entire chapter content seamlessly switches from English to Urdu (or vice versa).

**Why this priority**: This is the core value proposition - enabling bilingual readers to access content in their preferred language, making the book accessible to Urdu-speaking users.

**Independent Test**: Can be fully tested by logging in, navigating to any chapter, clicking the translation toggle, and verifying the page displays Urdu content. Delivers immediate value by providing content in the user's native language.

**Acceptance Scenarios**:

1. **Given** a logged-in user is viewing a chapter in English, **When** they click the Urdu translation toggle button at the top of the chapter, **Then** the entire chapter content switches to display in Urdu
2. **Given** a logged-in user is viewing a chapter in Urdu, **When** they click the English translation toggle button, **Then** the chapter content switches back to display in English
3. **Given** a logged-in user switches a chapter to Urdu, **When** they navigate to a different chapter, **Then** the new chapter also displays in Urdu (language preference persists across navigation)

---

### User Story 2 - Visual Feedback for Translation State (Priority: P2)

A user wants clear visual indication of which language they are currently viewing so they always know whether they're reading English or Urdu content.

**Why this priority**: Ensures users have clear awareness of the current language state, preventing confusion and improving usability.

**Independent Test**: Can be tested by toggling between languages and verifying the translation button clearly indicates the current language state through visual design (text label, icon, or color).

**Acceptance Scenarios**:

1. **Given** a user is viewing content in English, **When** they look at the translation toggle button, **Then** it clearly indicates "English" or shows an appropriate visual indicator for English content
2. **Given** a user is viewing content in Urdu, **When** they look at the translation toggle button, **Then** it clearly indicates "Urdu" (اردو) or shows an appropriate visual indicator for Urdu content

---

### User Story 3 - Non-Logged-In User Behavior (Priority: P3)

A non-logged-in user viewing a chapter should not see the translation toggle button, as translation is a feature reserved for authenticated users.

**Why this priority**: Maintains feature access control and encourages user registration, while ensuring the UI remains clean for anonymous visitors.

**Independent Test**: Can be tested by viewing any chapter without logging in and verifying the translation toggle is not visible or accessible.

**Acceptance Scenarios**:

1. **Given** a non-logged-in user is viewing a chapter, **When** they look at the top of the chapter, **Then** the translation toggle button is not displayed
2. **Given** a non-logged-in user attempts to access a translated chapter URL directly, **Then** they are redirected to the English version or shown a login prompt

---

### Edge Cases

- What happens when a chapter has not been translated to Urdu yet? (Display a message indicating translation is unavailable, or hide the toggle for that specific chapter)
- How does the system handle navigation between chapters while in Urdu mode? (Language preference should persist across all chapter navigation)
- What if a user bookmarks a Urdu chapter URL - does it respect the logged-in requirement? (Redirect to login or English version if not authenticated)
- How does right-to-left (RTL) text rendering work for Urdu content? (Docusaurus i18n should handle RTL automatically with proper locale configuration)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-021**: System MUST display a translation toggle button at the top of each chapter for logged-in users
- **FR-022**: System MUST switch the chapter display between English and Urdu when the translation toggle is clicked
- **FR-023**: System MUST persist the user's language preference across chapter navigation within the same session
- **FR-024**: System MUST NOT display the translation toggle for non-logged-in users
- **FR-025**: System MUST use Docusaurus i18n framework with the 'ur' (Urdu) locale configuration
- **FR-026**: System MUST handle right-to-left (RTL) text rendering for Urdu content
- **FR-027**: System MUST provide clear visual indication of the currently selected language (English or Urdu)

### Key Entities *(include if feature involves data)*

- **Language Preference**: User's selected language (English or Urdu) for viewing chapters, persisted during the session
- **Chapter Content**: Each chapter exists in both English (default) and Urdu (translated) versions, managed through Docusaurus i18n locales
- **User Session**: Authentication state that determines whether the translation toggle is accessible

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Logged-in users can switch between English and Urdu for any chapter with a single click
- **SC-002**: Language preference persists across all chapter navigation within a session, eliminating the need to re-toggle on each page
- **SC-003**: Translation toggle loads and responds within 500ms, providing instant language switching
- **SC-004**: 100% of chapters display correctly in both languages with proper formatting and RTL support for Urdu
- **SC-005**: Non-logged-in users have no access to the translation feature, maintaining authentication boundaries

## Assumptions *(optional)*

- All chapters have been translated to Urdu and are available in the Docusaurus i18n structure under the 'ur' locale
- Docusaurus i18n is properly configured with the 'ur' locale in the project configuration
- The existing authentication system provides a reliable way to determine if a user is logged in
- Language preference uses browser session storage or URL-based locale switching (standard Docusaurus i18n pattern)
- RTL (right-to-left) rendering for Urdu text is handled automatically by Docusaurus i18n locale configuration

## Constraints *(optional)*

- Translation is handled entirely client-side using Docusaurus i18n; no backend API changes required
- Feature must work within the existing Docusaurus framework and not require custom internationalization logic
- Translation toggle must be visually consistent with the existing UI design system
- Language switching must not cause full page reloads (should use Docusaurus's built-in locale switching)

## Dependencies *(optional)*

- Docusaurus i18n configuration must be properly set up with 'ur' locale
- All chapter markdown files must exist in both English (`docs/`) and Urdu (`i18n/ur/docusaurus-plugin-content-docs/current/`) directories
- User authentication system must expose login status to the frontend
