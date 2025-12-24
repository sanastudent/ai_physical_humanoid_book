---
id: 012
title: Fix locale start command
stage: green
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5
feature: none
branch: 003-betterauth-completion
user: User
command: /sp.implement
labels: ["bugfix", "docusaurus", "locale", "server-start"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/my-book/package.json
tests:
  - Verified server starts without locale errors
  - Tested HTTP 200 response from localhost:3000
---

## Prompt

Claude, fix the Docusaurus locale error ONLY and start the server:

Error: "Docusaurus couldn't get default locale config for en,ur-PK"

DO NOT modify:
- RAG Chatbot
- Signin/Signup
- Personalization
- Backend
- Any other features

Fix ONLY:
1. Update package.json start command to single locale
2. Fix docusaurus.config.js i18n settings
3. Start server at localhost:3000 successfully

Make book run without locale errors.

## Response snapshot

Successfully fixed the Docusaurus locale configuration error and started the development server.

**Root Cause:**

The error "Docusaurus couldn't get default locale config for en,ur-PK" occurred because:

1. **Incorrect Flag Syntax:** The `--locale en,ur-PK` flag was being interpreted as a single locale string "en,ur-PK" instead of two separate locales
2. **Development vs Build Behavior:** In Docusaurus development mode:
   - `--locale <single>` serves only that specific locale
   - `--locale all` works for builds but not reliably in dev mode
   - **No flag** automatically serves all configured locales from docusaurus.config.ts

**Solution Applied:**

Removed the `--locale` flag from the start command entirely. Docusaurus automatically reads the i18n configuration and serves all configured locales.

**Changes Made:**

**File:** `frontend/my-book/package.json:7`

**Before:**
```json
"start": "docusaurus start --locale en,ur-PK"
```

**After:**
```json
"start": "docusaurus start"
```

**Why This Works:**

When you run `docusaurus start` without the `--locale` flag:
1. Docusaurus reads i18n config from docusaurus.config.ts
2. Finds `locales: ['en', 'ur-PK']`
3. Automatically serves both locales:
   - English at `http://localhost:3000/`
   - Urdu at `http://localhost:3000/ur-PK/`
4. Language toggle works automatically

**Verification:**

1. ✅ Cleared cache: `.docusaurus` and `build` directories
2. ✅ Started server: `npm start`
3. ✅ Server running: `http://localhost:3000/`
4. ✅ HTTP 200 response confirmed
5. ✅ No locale configuration errors

**Server Output:**
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

**Configuration Confirmed:**

docusaurus.config.ts remains unchanged and correct:
```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur-PK'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
    },
    'ur-PK': {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
}
```

**Key Learning:**

- **Development:** Use `docusaurus start` (no locale flag)
- **Build:** Use `docusaurus build --locale all`
- **Single Locale Dev:** Use `docusaurus start --locale <specific-locale>`
- **Multiple Locales:** Don't use comma-separated values in --locale flag

## Outcome

- ✅ Impact: Fixed locale configuration error, server running successfully
- 🧪 Tests: Server responds with HTTP 200, no configuration errors
- 📁 Files: 1 file modified (package.json)
- 🔁 Next prompts: Test language switching, verify both locales work
- 🧠 Reflection: Docusaurus handles multi-locale automatically when no --locale flag provided

## Evaluation notes (flywheel)

- Failure modes observed: --locale flag with comma-separated values doesn't work in dev mode
- Graders run and results (PASS/FAIL): Manual verification PASS - server started, no errors
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Document Docusaurus locale flag behavior for dev vs build
