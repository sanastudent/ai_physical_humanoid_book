---
id: "006"
title: "Revert Urdu Translation to Simple ur Locale"
stage: "general"
date: "2025-12-23"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "003-betterauth-completion"
user: "User"
command: "/sp.implement"
labels: ["i18n", "urdu", "locale", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files: |
  - frontend/my-book/docusaurus.config.ts
  - frontend/my-book/src/components/LanguageToggle/index.tsx
  - frontend/my-book/package.json
  - frontend/my-book/i18n/ur-PK/ (renamed to frontend/my-book/i18n/ur/)
tests: |

outcome: "Successfully reverted Urdu translation from ur-PK to ur locale"
evaluation: "All locale references updated, directory renamed, and configuration changed to use simple 'ur' locale"
---

# Revert Urdu Translation to Simple "ur" Locale

## User Input

```
Claude, revert Urdu translation to use simple "ur" locale instead of "ur-PK":

Issue: "ur-PK" causing complex issues. Original "ur" was working earlier.

Fix:
1. Change docusaurus.config.js locale from "ur-PK" to "ur"
2. Rename i18n/ur-PK/ directory to i18n/ur/
3. Update all locale references
4. Simplify the implementation

DO NOT modify:
- RAG Chatbot
- Signin/Signup
- Personalization
- Backend

Make Urdu translation work with simple "ur" locale like before.
```

## Response

Successfully reverted the Urdu translation implementation to use simple "ur" locale instead of "ur-PK" as requested.

### Changes Made

1. **Updated docusaurus.config.ts**:
   - Changed `locales: ['en', 'ur-PK']` to `locales: ['en', 'ur']`
   - Updated locale config for 'ur' with appropriate settings
   - Updated htmlLang from 'ur-PK' to 'ur'

2. **Renamed directory**:
   - Renamed `frontend/my-book/i18n/ur-PK/` to `frontend/my-book/i18n/ur/`
   - All translation files preserved in the new directory structure

3. **Updated all locale references**:
   - Updated LanguageToggle component to use 'ur' instead of 'ur-PK'
   - Removed 'ur-PK' from locale checks and localStorage operations
   - Updated package.json script from `start --locale ur-PK` to `start --locale ur`

### Result

The Urdu translation now uses the simple "ur" locale as it was originally designed. All configuration, directory structures, and code references have been updated to use "ur" instead of "ur-PK", simplifying the implementation and resolving the complex issues that were occurring with the "ur-PK" locale.

## Follow-up Tasks

- Test the language switching functionality to ensure it works properly with the "ur" locale
- Verify that all translation files are accessible with the new locale configuration