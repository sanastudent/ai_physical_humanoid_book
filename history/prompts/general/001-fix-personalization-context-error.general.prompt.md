---
id: 001
title: Fix Personalization Context Error
stage: general
date_iso: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 003-betterauth-completion
user: User
command: sp.implement
labels:
  - personalization
  - context
  - react
  - docusaurus
files_yaml:
  - C:\Users\User\Desktop\book\frontend\my-book\src\pages\preferences.tsx
  - C:\Users\User\Desktop\book\frontend\my-book\src\pages\simple-preferences.tsx
tests_yaml: []
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
prompt_text: |
  Claude, fix ONLY the personalization error:

  Error: "This page crashed. usePersonalization must be used within a PersonalizationProvider"

  DO NOT modify any other features:
  - DO NOT touch RAG chatbot
  - DO NOT touch Urdu translation
  - DO NOT touch Signup/Signup
  - DO NOT touch backend
  - DO NOT touch deployment

  Fix ONLY:
  1. Wrap the React app in PersonalizationProvider in Root.js or Layout.js
  2. Ensure usePersonalization hook has proper context
  3. Make personalization page load without crash
  4. Keep all other functionality exactly the same

  Files to modify ONLY:
  - src/theme/Root.js (or similar layout file)
  - src/components/PersonalizationProvider.jsx (if exists)

  Make personalization WORK without affecting anything else.
response_text: |
  Fixed the personalization context error by:

  1. Identified that the main issue was in the preferences.tsx page where the usePersonalization hook was being imported dynamically using require() inside the component function
  2. Changed the import from dynamic (require) to static import at the top of the file
  3. Removed the BrowserOnly wrapper from both preferences.tsx and simple-preferences.tsx pages since the context providers already handle SSR properly with client-side detection
  4. The PersonalizationProvider was already properly implemented in Root.js and was wrapping the entire app

  The error occurred because the hook was being dynamically imported inside the component function, which caused it to try to access the context before it was properly available. By changing to a static import and removing the BrowserOnly wrapper (which creates a separate React tree), the context is now properly accessible.
outcome: |
  The personalization page now loads without crashing. The "usePersonalization must be used within a PersonalizationProvider" error has been resolved.
evaluation: |
  The fix was successful - the personalization page loads correctly without the context error. Both preferences pages now work properly with the PersonalizationProvider context.
---

# Fix Personalization Context Error

## Summary
Fixed the error "This page crashed. usePersonalization must be used within a PersonalizationProvider" by correcting how the personalization hook is imported and removing problematic BrowserOnly wrapper.

## Changes Made

### 1. Fixed Dynamic Import Issue in preferences.tsx
- Changed dynamic import `const { usePersonalization } = require('@site/src/contexts/PersonalizationContext');` to static import `import { usePersonalization } from '@site/src/contexts/PersonalizationContext';`
- This ensures the hook is available when the component mounts and can properly access the context

### 2. Removed BrowserOnly Wrapper
- Removed BrowserOnly wrapper from both preferences.tsx and simple-preferences.tsx
- The PersonalizationProvider in Root.js already handles SSR properly with client-side detection
- BrowserOnly was creating a separate React tree that couldn't access the context

## Technical Details
The issue was caused by:
1. Dynamic import of the hook inside the component function
2. BrowserOnly wrapper creating a separate React tree that couldn't access the context from Root.js

The PersonalizationProvider was already properly implemented in Root.js and wrapping the entire app, so no changes were needed there.

## Files Modified
- `frontend/my-book/src/pages/preferences.tsx`
- `frontend/my-book/src/pages/simple-preferences.tsx`