# Railway Deployment - Quick Reference Card

## 🚀 Copy-Paste Values for Railway

### 1. Root Directory Setting
**Settings → General → Root Directory:**
```
backend
```

---

### 2. Build Command
**Settings → Deploy → Custom Build Command:**
```bash
pip install --upgrade pip && pip install -r requirements.txt && python migrations/run_migrations.py
```

---

### 3. Start Command
**Settings → Deploy → Custom Start Command:**
```bash
uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

---

### 4. Environment Variables
**Variables Tab - Add these:**

```bash
GOOGLE_API_KEY=YOUR_GOOGLE_API_KEY
OPENAI_API_KEY=YOUR_OPENAI_API_KEY
ANTHROPIC_API_KEY=YOUR_ANTHROPIC_API_KEY
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=YOUR_QDRANT_API_KEY
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536
NEON_DB_URL=postgresql://user:password@host.neon.tech/dbname
BETTERAUTH_SECRET=YOUR_RANDOM_SECRET_32_CHARS_MIN
BACKEND_HOST=0.0.0.0
PORT=${{PORT}}
BOOK_TITLE=Embodied AI: Building Intelligent Robotic Systems
```

**⚠️ IMPORTANT:** Use `PORT=${{PORT}}` exactly as shown (Railway syntax)

---

### 5. Python Version (Optional)
**Settings → Environment → Python Version:**
```
3.11
```
*(Already configured via `runtime.txt`)*

---

## 🔍 Quick Test After Deployment

Once deployed, test your Railway URL:

```bash
# Replace YOUR_URL with your Railway domain
curl https://YOUR_URL.railway.app/health
```

Expected response:
```json
{
  "status": "healthy",
  "service": "AI Book RAG API",
  "version": "1.0.0"
}
```

---

## ⚡ Common Issues & Quick Fixes

### Port Error
✅ **Fix:** Ensure you use `PORT=${{PORT}}` in environment variables

### Migration Failed
✅ **Fix:** Verify `NEON_DB_URL` is correct in Railway variables

### Build Timeout
✅ **Fix:** Remove `&& python migrations/run_migrations.py` from build command temporarily, deploy, then run migrations manually

---

## 📝 Deployment Checklist

- [ ] Set root directory to `backend`
- [ ] Add build command
- [ ] Add start command
- [ ] Copy all environment variables
- [ ] Deploy and wait for success
- [ ] Test `/health` endpoint
- [ ] Update frontend with Railway URL

---

## 🌐 Update Frontend

After successful deployment, update your frontend `.env`:

```bash
REACT_APP_BACKEND_URL=https://YOUR_PROJECT.railway.app
```

Replace `YOUR_PROJECT` with your actual Railway domain.

---

## 📚 Full Documentation

See `RAILWAY_DEPLOYMENT_GUIDE.md` for detailed explanations, troubleshooting, and advanced configuration.
