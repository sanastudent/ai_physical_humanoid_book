# Docusaurus i18n Locale Fix Complete

**Date:** 2025-12-23
**Status:** ✅ Complete

## Summary

Fixed the "RangeError: Incorrect locale information provided" error by changing the Urdu locale from `ur` to the more specific `ur-PK` (Urdu - Pakistan) format.

## Problem

Docusaurus was throwing a RangeError when trying to use the `ur` locale. This error occurs because:

1. Node.js's `Intl` API (used by Docusaurus for date formatting, number formatting, etc.) prefers locale codes in the BCP 47 format with region identifiers
2. While `ur` is technically valid, it lacks regional context which can cause issues on some systems
3. The more specific `ur-PK` (Urdu - Pakistan) locale has better system support

## Changes Made

### 1. Updated docusaurus.config.ts

Changed locale from `ur` to `ur-PK` and added `htmlLang` property.

### 2. Renamed i18n Directory

Renamed: `i18n/ur/` → `i18n/ur-PK/`

### 3. Updated package.json Scripts

Updated npm scripts to use `ur-PK` instead of `ur`:
- `start`: `--locale en,ur-PK`
- `start:ur`: `--locale ur-PK`

### 4. Updated Urdu Page Redirects

Changed all redirects from `/ur/` to `/ur-PK/` in:
- `i18n/ur-PK/docusaurus-plugin-content-pages/signup.tsx`
- `i18n/ur-PK/docusaurus-plugin-content-pages/signin.tsx`

## Why ur-PK Instead of ur?

### BCP 47 Language Tags

The BCP 47 standard defines language tags in the format: `language-REGION`

- `ur-PK` (Urdu - Pakistan) - Better system support
- `ur-IN` (Urdu - India) - Alternative option
- `ur` (generic) - Less reliable across systems

### Why Region Matters

1. **Number Formatting:** Different regions use different formats
2. **Date Formatting:** Date order and calendar systems vary
3. **Currency:** Currency symbols differ by region
4. **System Support:** Region-specific locales have better support in Node.js Intl API

## URLs After Fix

- English: `http://localhost:3000/`
- Urdu: `http://localhost:3000/ur-PK/`

## Testing

```bash
cd frontend/my-book
npm install
npm start
```

Visit http://localhost:3000 and use the language dropdown to switch to اردو.

## Files Modified

1. `frontend/my-book/docusaurus.config.ts` - Updated i18n configuration
2. `frontend/my-book/package.json` - Updated npm scripts
3. `frontend/my-book/i18n/ur-PK/docusaurus-plugin-content-pages/signup.tsx` - Updated redirect
4. `frontend/my-book/i18n/ur-PK/docusaurus-plugin-content-pages/signin.tsx` - Updated redirect

## Directories Renamed

- `frontend/my-book/i18n/ur/` → `frontend/my-book/i18n/ur-PK/`

## No Changes Made To

- RAG Chatbot (unchanged)
- Signin/Signup auth logic (unchanged)
- Backend (unchanged)
- Translation content (only directory name changed)

The locale error is now fixed and Urdu translation will work properly.
