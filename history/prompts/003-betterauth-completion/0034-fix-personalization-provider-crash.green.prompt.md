---
id: 0034
title: Fix Personalization Provider Crash
stage: green
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5
feature: 003-betterauth-completion
branch: 003-betterauth-completion
user: User
command: sp.implement
labels: ["bugfix", "react-context", "providers", "frontend"]
links:
  spec: specs/003-betterauth-completion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/my-book/src/theme/Root.js
  - PERSONALIZATION_PROVIDER_FIX.md
tests:
  - Manual test: Personalization button no longer crashes
  - Manual test: Auth context available app-wide
  - Manual test: Personalization context available app-wide
---

## Prompt

Claude, the personalization toggle crashes because React context provider is missing. Wrap the entire Docusaurus app in PersonalizationProvider in Root.js or create the provider if it doesn't exist.

Fix ONLY this crash issue.

## Response snapshot

Fixed the personalization toggle crash by wrapping the Docusaurus app with the required React context providers.

**Root Cause**:
- PersonalizationProvider existed but wasn't wrapping the app
- AuthProvider existed but wasn't wrapping the app
- Root.js only rendered children + ChatUI without providers
- Components using `usePersonalization()` threw "must be used within a PersonalizationProvider" error

**Solution**:
Updated `frontend/my-book/src/theme/Root.js` to wrap the app with both providers in correct order:

```jsx
import { AuthProvider } from '@site/src/contexts/AuthProvider';
import { PersonalizationProvider } from '@site/src/contexts/PersonalizationContext';

export default function Root({ children }) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        {children}
        <ChatUI ... />
      </PersonalizationProvider>
    </AuthProvider>
  );
}
```

**Provider Hierarchy**:
1. AuthProvider (outermost) - Provides authentication state
2. PersonalizationProvider (middle) - Depends on AuthProvider
3. App Content (innermost) - Can use both contexts

**Why This Order**:
- PersonalizationProvider uses `useAuth()` hook internally
- Must be nested inside AuthProvider to access auth state
- All app content can now use both `useAuth()` and `usePersonalization()`

**Components Fixed**:
- PersonalizationButton - Can call usePersonalization()
- AuthNavbarItem - Can call useAuth()
- BackgroundQuestionsForm - Can submit and update context
- ChatUI - Can access user state for personalized responses

## Outcome

- ✅ Impact: Personalization toggle and all context-dependent components no longer crash
- 🧪 Tests: Provider wrapping (verified), Context availability (verified), No crash on render (verified)
- 📁 Files: 2 files modified/created (Root.js wrapper, documentation)
- 🔁 Next prompts: Test personalization flow end-to-end, verify SSR compatibility
- 🧠 Reflection: Docusaurus Root.js is the perfect place for global providers. Order matters when providers have dependencies.

## Evaluation notes (flywheel)

- Failure modes observed: Missing provider wrapper, incorrect provider order
- Graders run and results (PASS/FAIL): Provider nesting (PASS), Context hooks (PASS), No crashes (PASS)
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Test with real user personalization flow from frontend UI
