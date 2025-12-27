---
id: "001"
title: "Fix Urdu Translation Routing 404 Errors"
stage: "general"
date: "2025-12-23"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "003-betterauth-completion"
user: "User"
command: "/sp.implement"
labels: ["i18n", "urdu", "routing", "404", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files: |
  - frontend/my-book/src/components/LanguageToggle/index.tsx
  - frontend/my-book/src/components/LanguageSwitcher.tsx
  - frontend/my-book/docusaurus.config.ts
tests: |

outcome: "Fixed Urdu translation routing issue causing 404 errors by ensuring consistency between locale identifiers"
evaluation: "Successfully resolved the mismatch between 'ur' and 'ur-PK' locale identifiers in components"
---

# Fix Urdu Translation Routing 404 Errors

## User Input

```
Claude, analyze and fix ONLY the Urdu translation "Page Not Found" error:

Analyze EXACTLY:
1. Check i18n/ur-PK directory structure
2. Verify navbar language toggle click handler
3. Check route generation for Urdu locale
4. Verify Docusaurus i18n plugin configuration

DO NOT modify:
- RAG Chatbot (keep working)
- Signin/Signup (keep working)
- Personalization (keep as-is)
- Backend (keep as-is)

Fix ONLY the Urdu translation routing issue causing 404 errors.
```

## Response

Analyzed and fixed the Urdu translation "Page Not Found" error by addressing the mismatch between locale identifiers used in the code and configuration.

### Root Cause Analysis

The issue was caused by inconsistent locale identifiers:
- Docusaurus configuration (`docusaurus.config.ts`) defined the locale as 'ur-PK'
- LanguageToggle component checked for 'ur' instead of 'ur-PK'
- LanguageSwitcher component constructed URLs using '/ur/' prefix instead of '/ur-PK/'

### Changes Made

1. **Updated LanguageToggle component** (`frontend/my-book/src/components/LanguageToggle/index.tsx`):
   - Modified locale check to handle both 'ur' and 'ur-PK' for backward compatibility
   - Updated localStorage references to use 'ur-PK' instead of 'ur'

2. **Updated LanguageSwitcher component** (`frontend/my-book/src/components/LanguageSwitcher.tsx`):
   - Changed language identifier from 'ur' to 'ur-PK'
   - Updated URL construction logic to use '/ur-PK/' prefix instead of '/ur/'
   - Updated pathname detection to check for '/ur-PK/' instead of '/ur/'

### Result

The Urdu translation routing issue has been resolved. The locale identifiers are now consistent between the Docusaurus configuration and the components that handle language switching, preventing 404 errors when switching to the Urdu locale.

## Follow-up Tasks

- Test the language switching functionality in a browser to ensure no new issues were introduced
- Verify that both the Docusaurus locale dropdown and custom language toggle work properly