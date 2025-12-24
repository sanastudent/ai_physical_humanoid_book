# Quick Start Guide - Docusaurus Book Fixed and Ready

## What Was Fixed

✅ **"process is not defined"** client-side errors completely resolved
✅ Main page and all chapter pages now working
✅ Both English and Urdu locales functional
✅ All interactive features (ChatUI, Auth, Personalization, Translation) working

## Quick Commands (Windows PowerShell)

### 1. Local Development

```powershell
# Navigate to project
cd C:\Users\User\Desktop\book\frontend\my-book

# Clear cache (if needed)
npm run clear

# Install dependencies (if needed)
npm install

# Start development server
npm run start
```

**Access**: http://localhost:3000

### 2. Build for Production

```powershell
# Clean build
npm run build

# Serve locally to test
npm run serve
```

**Access**: http://localhost:3000

### 3. Deploy to Vercel

**Option A - Automatic (Recommended)**:
1. Go to https://vercel.com
2. Click "Import Project"
3. Connect GitHub: `sanastudent/ai_physical_humanoid_book`
4. Vercel auto-detects `vercel.json` configuration
5. Click "Deploy"
6. Done! Auto-deploys on every push to `main`

**Option B - Manual**:
```powershell
# Install Vercel CLI (first time only)
npm i -g vercel

# Login to Vercel
vercel login

# Deploy from project root
cd C:\Users\User\Desktop\book
vercel --prod
```

### 4. Deploy to GitHub Pages

```powershell
cd C:\Users\User\Desktop\book\frontend\my-book

# First, ensure docusaurus.config.ts has correct GitHub settings
# organizationName: 'sanastudent'
# projectName: 'ai_physical_humanoid_book'

# Deploy
npm run deploy
```

## Project Structure

```
book/
├── frontend/my-book/         # Docusaurus frontend
│   ├── docs/                 # English documentation
│   ├── i18n/ur/             # Urdu translations
│   ├── src/
│   │   ├── components/       # React components
│   │   ├── contexts/         # React contexts
│   │   ├── pages/           # Custom pages
│   │   ├── utils/           # Utility functions (config.ts fixed here)
│   │   └── theme/           # Theme customizations
│   ├── docusaurus.config.ts  # Main configuration
│   └── package.json
├── backend/                  # FastAPI backend
├── vercel.json              # Vercel deployment config
└── README.md
```

## Key Files Modified

### Fixed Files (process.env errors resolved):
- `src/utils/config.ts` - New safe configuration utility ⭐
- `src/components/ChatUI/index.tsx`
- `src/components/Auth/SignupForm.tsx`
- `src/components/Auth/SigninForm.tsx`
- `src/components/Auth/BackgroundQuestionsForm.tsx`
- `src/components/TranslateButton/index.tsx`
- `src/components/PersonalizationButton/index.tsx`
- `src/contexts/PersonalizationContext.tsx`
- `src/contexts/AuthProvider.tsx`

## Available Scripts

From `frontend/my-book/package.json`:

```powershell
npm run start          # Start dev server (English)
npm run start:ur       # Start dev server (Urdu)
npm run build          # Build for production
npm run build:clean    # Clear cache and build
npm run serve          # Serve production build
npm run serve:ur       # Serve production build (Urdu)
npm run clear          # Clear Docusaurus cache
npm run deploy         # Deploy to GitHub Pages
```

## Environment Variables (Optional)

### For Local Development:
Create `.env` in `frontend/my-book/`:
```env
REACT_APP_BACKEND_URL=http://localhost:8000
```

### For Vercel Production:
Add in Vercel Dashboard → Project Settings → Environment Variables:
```
REACT_APP_BACKEND_URL=https://your-backend-url.vercel.app
```

### For Backend URL in docusaurus.config.ts:
```typescript
customFields: {
  backendUrl: process.env.BACKEND_URL || 'https://your-backend.com',
}
```

## Verification Checklist

Before deployment, verify:

- [ ] `npm run build` succeeds without errors
- [ ] English locale builds successfully
- [ ] Urdu locale builds successfully
- [ ] No console errors when running `npm run serve`
- [ ] Main page loads at http://localhost:3000
- [ ] Chapter pages load (e.g., /docs/chapters/module1-intro)
- [ ] Language switcher works (English ↔ Urdu)
- [ ] Dark/Light theme toggle works

## Features Available

### Book Content
- **Module 1**: ROS 2 Introduction
- **Module 2**: Digital Twin Technology
- **Module 3**: NVIDIA Isaac Platform
- **Module 4**: Vision-Language-Action Models
- Glossary, Introduction, References, Summary

### Interactive Features
- **RAG ChatUI**: Ask questions about book content
- **Authentication**: Signup/Signin with Better-Auth
- **Personalization**: Customize chapters based on user background
- **Translation**: Translate chapters to Urdu on-demand

### Technical Features
- **i18n Support**: English and Urdu
- **Dark Mode**: Default theme (customizable)
- **Responsive**: Mobile, tablet, desktop
- **Search**: Built-in documentation search
- **SEO Optimized**: Meta tags, sitemaps

## Troubleshooting

### Port 3000 already in use:
```powershell
# Find and kill process
netstat -ano | findstr :3000
taskkill /PID <PID> /F

# Or use different port
npm run start -- --port 3001
```

### Build cache issues:
```powershell
npm run clear
rm -rf node_modules
npm install
npm run build
```

### Git errors with 'nul' file:
```powershell
# Remove problematic files
rm frontend/my-book/nul
rm nul

# Stage only source code
git add frontend/my-book/src/
git commit -m "Your commit message"
```

## Support & Resources

- **GitHub Repo**: https://github.com/sanastudent/ai_physical_humanoid_book
- **Docusaurus Docs**: https://docusaurus.io/docs
- **Vercel Docs**: https://vercel.com/docs
- **Issue Tracker**: GitHub Issues tab

## Success! 🎉

Your Docusaurus book is now:
- ✅ Fixed and functional
- ✅ Built successfully
- ✅ Committed to GitHub
- ✅ Ready for deployment

**Next Step**: Deploy to Vercel for live hosting!

---

**Last Updated**: 2025-12-18
**Status**: All issues resolved, ready for production deployment
