# Vercel Deployment Guide

This guide covers deploying your AI-Driven Book project to Vercel. The project supports dual deployment to both **Vercel** (frontend) and **Railway** (backend).

## 📋 Table of Contents

1. [Deployment Architecture](#deployment-architecture)
2. [Prerequisites](#prerequisites)
3. [Frontend Deployment (Vercel)](#frontend-deployment-vercel)
4. [Backend Options](#backend-options)
5. [Environment Variables](#environment-variables)
6. [Post-Deployment Configuration](#post-deployment-configuration)
7. [Troubleshooting](#troubleshooting)

---

## 🏗️ Deployment Architecture

### Recommended Setup

```
┌─────────────────────────────────────────┐
│  Frontend (Docusaurus)                  │
│  → Deployed to Vercel                   │
│  → Static site with React components    │
│  → https://your-project.vercel.app      │
└─────────────────────────────────────────┘
                   │
                   │ API Calls
                   ▼
┌─────────────────────────────────────────┐
│  Backend (FastAPI)                      │
│  → Deployed to Railway (Recommended)    │
│  → OR Vercel Serverless (Alternative)   │
│  → https://your-app.up.railway.app      │
└─────────────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        ▼                     ▼
┌──────────────┐    ┌──────────────────┐
│ Qdrant Cloud │    │ Neon Postgres    │
│ (Embeddings) │    │ (Auth & Users)   │
└──────────────┘    └──────────────────┘
```

**Why Railway for Backend?**
- FastAPI works better with persistent connections (Qdrant, Neon DB)
- No cold starts for RAG queries
- Better for long-running operations
- Simpler deployment configuration

---

## 📦 Prerequisites

Before deploying, ensure you have:

1. **Vercel Account**: Sign up at [vercel.com](https://vercel.com)
2. **API Keys**:
   - Google AI API key (for Gemini)
   - Anthropic API key (for Claude)
   - OpenAI API key (optional)
   - Qdrant Cloud credentials
   - Neon Postgres connection string
3. **GitHub Repository**: Your code should be in a GitHub repo
4. **Node.js 20+**: For local testing

---

## 🚀 Frontend Deployment (Vercel)

### Option 1: Deploy via Vercel Dashboard (Recommended)

1. **Import Project**
   ```
   1. Go to https://vercel.com/new
   2. Click "Import Git Repository"
   3. Select your GitHub repository
   4. Vercel will auto-detect the configuration
   ```

2. **Configure Build Settings**
   - **Framework Preset**: Other
   - **Root Directory**: `./` (leave as root)
   - **Build Command**: `cd frontend/my-book && npm install && npm run build`
   - **Output Directory**: `frontend/my-book/build`
   - **Install Command**: `npm install --prefix frontend/my-book`

3. **Add Environment Variables** (See [Environment Variables](#environment-variables) section)

4. **Deploy**
   - Click "Deploy"
   - Wait 2-3 minutes for build to complete
   - Your site will be live at `https://your-project.vercel.app`

### Option 2: Deploy via Vercel CLI

```bash
# Install Vercel CLI
npm i -g vercel

# Login to Vercel
vercel login

# Deploy from project root
vercel

# Follow the prompts:
# - Set up and deploy? Yes
# - Which scope? [Your account]
# - Link to existing project? No
# - Project name? [your-project-name]
# - Directory? ./
# - Override settings? No

# Production deployment
vercel --prod
```

### Verify Frontend Deployment

1. Visit your Vercel URL: `https://your-project.vercel.app`
2. Check that:
   - Homepage loads correctly
   - Navigation works
   - Dark mode toggle functions
   - Locale switcher (EN/UR) works
   - Book content displays properly

---

## 🔧 Backend Options

### Option A: Railway (Recommended)

**Your backend is already configured for Railway with `Procfile`.**

1. Keep using Railway for backend
2. Update frontend environment variables to point to Railway URL
3. See `RAILWAY_DEPLOYMENT_GUIDE.md` for Railway-specific instructions

### Option B: Vercel Serverless (Alternative)

**Note**: Vercel serverless has limitations for FastAPI:
- Cold starts (3-5s delay on first request)
- 60-second timeout limit (may affect long RAG queries)
- Stateless (connections reset between requests)

**To deploy backend to Vercel:**

1. Create a **separate Vercel project** for backend:
   ```bash
   cd backend
   vercel
   ```

2. Configure:
   - **Root Directory**: `backend`
   - **Framework**: Other
   - Vercel will use `backend/vercel.json` automatically

3. Add all environment variables (see below)

4. Deploy:
   ```bash
   vercel --prod
   ```

5. Update frontend env to use Vercel backend URL

---

## 🔐 Environment Variables

### Frontend Environment Variables (Vercel Dashboard)

Add these in: `Project Settings → Environment Variables`

```env
# Backend API URL
REACT_APP_BACKEND_URL=https://your-backend.up.railway.app

# Optional: Analytics, Search
# ALGOLIA_APP_ID=your-app-id
# ALGOLIA_API_KEY=your-api-key
```

### Backend Environment Variables (Railway or Vercel)

**For Railway**: Add via Railway Dashboard
**For Vercel**: Add via Vercel Project Settings

```env
# AI APIs
GOOGLE_API_KEY=your-google-api-key
ANTHROPIC_API_KEY=your-anthropic-api-key
OPENAI_KEY=your-openai-api-key

# Qdrant Vector Database
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536

# Neon Postgres Database
NEON_DB_URL=postgresql://user:password@host/database?sslmode=require

# BetterAuth
BETTERAUTH_SECRET=your-32-char-random-secret

# FastAPI Config
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
```

### Generating Secrets

```bash
# Generate BetterAuth secret
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

---

## ⚙️ Post-Deployment Configuration

### 1. Update Frontend to Use Backend API

After deploying backend, update the frontend environment:

```bash
# In Vercel Dashboard → Your Frontend Project → Settings → Environment Variables
REACT_APP_BACKEND_URL=https://your-backend.up.railway.app
# or
REACT_APP_BACKEND_URL=https://your-backend.vercel.app
```

**Redeploy frontend** to apply changes:
```bash
vercel --prod
```

### 2. Configure CORS on Backend

Ensure your backend's `main.py` allows your Vercel domain:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://your-project.vercel.app",
        "http://localhost:3000"  # for local dev
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 3. Test Integration

1. Open your Vercel frontend
2. Try the chatbot
3. Test personalization features
4. Test authentication (signup/signin)
5. Check translation features

---

## 🐛 Troubleshooting

### Issue: "Failed to fetch" errors in chatbot

**Solution**: Check CORS configuration and `REACT_APP_BACKEND_URL`

```javascript
// In browser console
console.log(process.env.REACT_APP_BACKEND_URL)
// Should show your backend URL
```

### Issue: Build fails on Vercel

**Common causes**:
1. **Node version mismatch**
   - Solution: Ensure `NODE_VERSION=20` in build env

2. **Missing dependencies**
   - Solution: Check `package.json` in `frontend/my-book`

3. **TypeScript errors**
   - Solution: Run `npm run typecheck` locally first

### Issue: Backend cold starts on Vercel

**Solution**: Use Railway for backend instead. Vercel serverless has cold starts.

### Issue: Environment variables not working

**Solution**:
1. Verify variables are set in Vercel dashboard
2. Redeploy after adding variables
3. Check variable names match exactly (case-sensitive)

### Issue: API calls failing

**Debug steps**:
```bash
# 1. Check backend is running
curl https://your-backend.up.railway.app/health

# 2. Check CORS headers
curl -I -X OPTIONS https://your-backend.up.railway.app/api/rag/query \
  -H "Origin: https://your-project.vercel.app"

# 3. Check environment variables
# In Vercel dashboard → Deployments → Latest → View Function Logs
```

---

## 📚 Additional Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Docusaurus Deployment](https://docusaurus.io/docs/deployment#deploying-to-vercel)
- [Railway Documentation](https://docs.railway.app)
- [FastAPI Deployment Guide](https://fastapi.tiangolo.com/deployment/)

---

## 🔄 Continuous Deployment

Both Vercel and Railway support automatic deployments:

- **Vercel**: Auto-deploys on every push to `main` branch
- **Railway**: Auto-deploys on every push (configure in Railway dashboard)

To deploy specific branches:
```bash
# Vercel - Deploy specific branch
vercel --prod --branch feature-branch

# Railway - Configure in dashboard under "Deploy" settings
```

---

## ✅ Deployment Checklist

- [ ] Frontend deployed to Vercel
- [ ] Backend deployed to Railway (or Vercel)
- [ ] All environment variables configured
- [ ] CORS configured correctly
- [ ] Frontend points to correct backend URL
- [ ] Chatbot working
- [ ] Authentication working
- [ ] Translation features working
- [ ] Personalization working
- [ ] Custom domain configured (optional)
- [ ] SSL certificate active (auto by Vercel/Railway)

---

**Need help?** Check the troubleshooting section or open an issue in the repository.
