# Quick Deploy Guide

Fast-track deployment instructions for your AI Book project.

## 🚀 Deploy in 10 Minutes

### Prerequisites
- [ ] Vercel account ([sign up](https://vercel.com))
- [ ] Railway account ([sign up](https://railway.app))
- [ ] All API keys ready (Google AI, Anthropic, Qdrant, Neon)

---

## Step 1: Deploy Frontend to Vercel (5 min)

### Option A: Vercel Dashboard (Easiest)
1. Go to [vercel.com/new](https://vercel.com/new)
2. Import your GitHub repository
3. Vercel auto-detects configuration ✅
4. Add environment variable:
   ```
   REACT_APP_BACKEND_URL=https://your-app.up.railway.app
   ```
5. Click **Deploy**
6. Done! Your site is live 🎉

### Option B: Vercel CLI
```bash
npm i -g vercel
vercel login
vercel --prod
```

---

## Step 2: Deploy Backend to Railway (5 min)

### Via Railway Dashboard
1. Go to [railway.app/new](https://railway.app/new)
2. Select "Deploy from GitHub repo"
3. Choose your repository
4. Set **Root Directory**: `backend`
5. Add environment variables (see below)
6. Railway auto-detects `Procfile` ✅
7. Deploy!

### Environment Variables for Railway
```env
GOOGLE_API_KEY=your-key
ANTHROPIC_API_KEY=your-key
OPENAI_KEY=your-key
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=your-key
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536
NEON_DB_URL=postgresql://user:pass@host/db?sslmode=require
BETTERAUTH_SECRET=your-32-char-secret
```

---

## Step 3: Connect Frontend to Backend

1. Copy your Railway backend URL
2. Update Vercel environment variable:
   ```
   REACT_APP_BACKEND_URL=https://your-app.up.railway.app
   ```
3. Redeploy frontend in Vercel dashboard

---

## Step 4: Test Integration

1. Open your Vercel URL: `https://your-project.vercel.app`
2. Try the chatbot
3. Test authentication (signup/signin)
4. Verify translation works

---

## ✅ Deployment Checklist

- [ ] Frontend deployed to Vercel
- [ ] Backend deployed to Railway
- [ ] Environment variables configured
- [ ] CORS enabled on backend
- [ ] Frontend connected to backend
- [ ] Chatbot works
- [ ] Authentication works
- [ ] Translation works

---

## 🐛 Quick Troubleshooting

**Chatbot not working?**
```bash
# Check backend is up
curl https://your-app.up.railway.app/health

# Verify REACT_APP_BACKEND_URL in frontend
# Vercel Dashboard → Project → Settings → Environment Variables
```

**Build failing?**
- Ensure Node.js 20+ in Vercel settings
- Check `requirements.txt` has all dependencies
- Review build logs in dashboard

**CORS errors?**
Update `backend/src/main.py`:
```python
allow_origins=["https://your-project.vercel.app"]
```

---

## 📚 Full Documentation

- [Vercel Deployment Guide](./VERCEL_DEPLOYMENT_GUIDE.md)
- [Platform Comparison](./DEPLOYMENT_PLATFORMS_COMPARISON.md)
- [Environment Variables](./.env.example)

---

## 🎯 What's Deployed?

```
┌────────────────────────────────────┐
│  Your Vercel URL                   │
│  https://your-project.vercel.app   │
│  ├─ Book content (Docusaurus)      │
│  ├─ Authentication UI              │
│  ├─ Personalization features       │
│  └─ Translation (EN/UR)            │
└────────────────────────────────────┘
                │
                │ API Calls
                ▼
┌────────────────────────────────────┐
│  Your Railway URL                  │
│  https://your-app.up.railway.app   │
│  ├─ FastAPI backend                │
│  ├─ RAG chatbot                    │
│  ├─ Qdrant integration             │
│  └─ Neon DB (auth)                 │
└────────────────────────────────────┘
```

---

**Ready to deploy?** Start with Step 1! 🚀
