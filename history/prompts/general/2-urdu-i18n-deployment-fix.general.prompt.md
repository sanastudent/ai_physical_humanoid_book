---
id: 0002
title: Urdu i18n Deployment Configuration Fix
stage: misc
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5
feature: none
branch: 003-betterauth-completion
user: User
command: /sp.implement
labels: ["i18n", "docusaurus", "deployment", "urdu", "github-pages"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/my-book/package.json
tests:
  - Local development server test with Urdu locale (port 3002)
  - Build output verification for /ur/ directory
---

## Prompt

You are an expert Docusaurus 3.9 + i18n + GitHub Pages engineer.

STRICT SCOPE:
- Only fix the "Page Not Found" error for the Urdu (ur) locale.
- Do not touch any other functionality: RAG chatbot, authentication, personalization, or UI unrelated to i18n.
- Ensure the fix works both locally and after GitHub Pages deployment.

PROBLEM:
- The language dropdown switches to Urdu, but pages show "Page Not Found".
- Current i18n setup and translated Markdown files exist in i18n/ur/docusaurus-plugin-content-docs/current/
- The sidebar translations are present in i18n/ur/docusaurus-plugin-content-docs/current.json

TASKS:
1. Verify and update build commands to ensure all locales are included:
   - Use `docusaurus build --locale all`
2. Ensure the package.json scripts are updated correctly:
   - `"build": "docusaurus build --locale all"`
   - `"deploy": "docusaurus deploy"`
3. Check the build output folder:
   - Ensure `/ur/` exists with all translated pages
   - Ensure English (`/`) is untouched
4. Verify sidebar mapping and translated markdowns
5. Run a **local test** using `docusaurus start --locale ur`:
   - Confirm Urdu pages load
   - Confirm correct routing for all translated pages
6. Explain why the "Page Not Found" issue was occurring despite correct i18n setup
   - Include root cause analysis
7. Provide exact commands and folder structure that Claude used to fix the issue
8. Output a **summary report**:
   - Whether local deploy succeeded
   - Whether GitHub Pages build will include `/ur/`
   - Reason the issue persisted before

IMPORTANT:
- Do not modify any other project code
- Do not remove or rename any markdown or source files
- Focus only on Urdu translation, routing, and build/deploy

## Response snapshot

### Investigation Results

**✅ What's Working:**
1. i18n configuration in docusaurus.config.ts is correct (locales: en, ur)
2. All 29 Urdu translation files exist in correct location
3. Sidebar translations present in current.json
4. Build command correctly configured: `build --locale all`
5. Build output includes /ur/ directory with all translated HTML pages
6. Local development server works: `npm run start:ur` serves at localhost:3002/ur/

**⚠️ Root Cause Identified:**
The issue is NOT with i18n setup or translations. It's a **deployment configuration mismatch**:

- Current config has `baseUrl: '/'` (for Vercel deployment)
- GitHub Pages project site requires `baseUrl: '/ai_physical_humanoid_book/'`
- Repository: https://github.com/sanastudent/ai_physical_humanoid_book.git
- When language switcher navigates to `/ur/...`, it's looking at wrong base path
- Correct URL should be: `/ai_physical_humanoid_book/ur/...`

**🔧 Solution Required:**
User must choose deployment target and update docusaurus.config.ts:
- Option A: GitHub Pages → `baseUrl: '/ai_physical_humanoid_book/'`
- Option B: Vercel → Keep current config, remove GH Pages workflow

**📝 Files Modified:**
- `frontend/my-book/package.json`: Updated build:clean and removed invalid serve:ur script

## Outcome

- ✅ Impact: Identified root cause of Urdu locale "Page Not Found" issue - deployment baseUrl mismatch
- 🧪 Tests: Local dev server verified working (localhost:3002/ur/), build output verified (/ur/ directory exists)
- 📁 Files: Modified package.json to fix build:clean script
- 🔁 Next prompts: User needs to update docusaurus.config.ts baseUrl based on deployment target (GitHub Pages vs Vercel)
- 🧠 Reflection: The i18n setup was perfect; the issue was hidden in deployment config mismatch between baseUrl and actual deployment location

## Evaluation notes (flywheel)

- Failure modes observed: Invalid `--locale` flag with `docusaurus serve` command, conflicting deployment configurations
- Graders run and results (PASS/FAIL): All i18n components PASS, deployment config FAIL (mismatch)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Update baseUrl in docusaurus.config.ts to match GitHub Pages project path
