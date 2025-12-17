# Railway Deployment Guide for AI Book Backend

## Project Overview
- **Framework**: FastAPI
- **Entry Point**: `backend/src/main.py`
- **Python Version**: 3.8+
- **Dependencies**: `backend/requirements.txt`
- **Database**: Neon PostgreSQL (requires migrations)
- **External Services**: Qdrant Cloud, OpenAI, Google Gemini

---

## Pre-Deployment Checklist

- [ ] Neon DB instance created and connection string ready
- [ ] Qdrant Cloud instance created with API key
- [ ] OpenAI API key ready
- [ ] Google Gemini API key ready
- [ ] BetterAuth secret generated (32+ characters)

---

## Railway Setup Instructions

### 1. Create New Railway Project

1. Go to [Railway.app](https://railway.app)
2. Click **"New Project"**
3. Select **"Deploy from GitHub repo"**
4. Choose your repository: `C:\Users\User\Desktop\book`
5. Railway will auto-detect the project

### 2. Configure Root Directory

Since your backend is in a subdirectory:

1. Go to **Settings** → **General**
2. Set **Root Directory** to: `backend`
3. Click **Save**

### 3. Set Environment Variables

Go to **Variables** tab and add the following:

```bash
# AI API Keys
GOOGLE_API_KEY=YOUR_GOOGLE_API_KEY
OPENAI_API_KEY=YOUR_OPENAI_API_KEY
ANTHROPIC_API_KEY=YOUR_ANTHROPIC_API_KEY

# Qdrant Cloud
QDRANT_URL=https://your-instance.cloud.qdrant.io
QDRANT_API_KEY=YOUR_QDRANT_API_KEY
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536

# Neon Database
NEON_DB_URL=postgresql://user:password@host.neon.tech/dbname

# BetterAuth
BETTERAUTH_SECRET=YOUR_RANDOM_SECRET_32_CHARS_MIN

# Backend Configuration
BACKEND_HOST=0.0.0.0
PORT=${{PORT}}

# Book Configuration
BOOK_TITLE=Embodied AI: Building Intelligent Robotic Systems
```

**IMPORTANT**:
- Railway automatically provides `PORT` variable - use `${{PORT}}` syntax
- Remove `BACKEND_PORT` from your env vars
- Use `PORT=${{PORT}}` instead

### 4. Configure Build & Start Commands

Go to **Settings** → **Deploy**

**Custom Build Command:**
```bash
pip install --upgrade pip && pip install -r requirements.txt && python migrations/run_migrations.py
```

**Custom Start Command:**
```bash
uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

### 5. Set Python Version (Optional but Recommended)

Create a `runtime.txt` file in `backend/` directory:

```
python-3.11.7
```

Or use Railway's Python version setting:
1. Go to **Settings** → **Environment**
2. Set **Python Version** to `3.11` or `3.10`

---

## Post-Deployment Steps

### 1. Verify Deployment

After deployment completes, check the logs:

```bash
# Should see:
✓ Qdrant collection ready
✓ All AI agents initialized and ready
✓ INFO: Uvicorn running on http://0.0.0.0:PORT
```

### 2. Test Health Endpoints

Railway will assign you a domain like `https://your-app.railway.app`

Test these endpoints:

```bash
# Basic health check
curl https://your-app.railway.app/health

# Readiness check (checks all dependencies)
curl https://your-app.railway.app/health/ready

# Component-specific checks
curl https://your-app.railway.app/health/backend
curl https://your-app.railway.app/health/qdrant
curl https://your-app.railway.app/health/embeddings
```

Expected response:
```json
{
  "status": "healthy",
  "service": "AI Book RAG API",
  "version": "1.0.0"
}
```

### 3. Test RAG Query Endpoint

```bash
curl -X POST https://your-app.railway.app/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is embodied AI?",
    "mode": "global"
  }'
```

---

## Troubleshooting

### Issue: Port Binding Error

**Error**: `Address already in use`

**Solution**: Make sure you're using `$PORT` environment variable:
```python
# In main.py (already configured correctly)
port = int(os.getenv("PORT", "8000"))
```

### Issue: Database Migration Failed

**Error**: `NEON_DB_URL not found`

**Solution**:
1. Check Railway environment variables
2. Ensure `NEON_DB_URL` is set correctly
3. Test connection manually:
   ```bash
   psql $NEON_DB_URL -c "SELECT version();"
   ```

### Issue: Qdrant Connection Failed

**Error**: `Failed to connect to Qdrant`

**Solution**:
1. Verify `QDRANT_URL` includes `https://`
2. Check `QDRANT_API_KEY` is correct
3. Test with health endpoint: `/health/qdrant`

### Issue: OpenAI/Gemini API Errors

**Error**: `Authentication failed`

**Solution**:
1. Verify API keys in Railway environment variables
2. Check API key is not expired
3. Ensure no extra spaces in the key

### Issue: Build Timeout

**Error**: `Build timed out`

**Solution**:
1. Railway default timeout is 10 minutes
2. If migrations take too long, run them separately first
3. Remove migration from build command temporarily:
   ```bash
   pip install --upgrade pip && pip install -r requirements.txt
   ```
4. Run migrations manually after first deploy

---

## Custom Domain (Optional)

1. Go to **Settings** → **Domains**
2. Click **Generate Domain** or **Add Custom Domain**
3. Update your frontend `.env`:
   ```
   REACT_APP_BACKEND_URL=https://your-custom-domain.com
   ```

---

## Monitoring & Logs

### View Live Logs
1. Go to Railway dashboard
2. Click on your service
3. Go to **Deployments** → **View Logs**

### Performance Metrics
Access built-in performance monitoring:
```bash
# Performance stats
curl https://your-app.railway.app/performance

# Slow requests (>2 seconds)
curl https://your-app.railway.app/performance/slow-requests
```

---

## Security Best Practices

### 1. Rotate Secrets Regularly
- Regenerate `BETTERAUTH_SECRET` every 90 days
- Rotate API keys quarterly

### 2. Update CORS Settings
In `src/main.py`, replace `allow_origins=["*"]` with:
```python
allow_origins=[
    "https://your-frontend-domain.com",
    "https://your-custom-domain.com"
]
```

### 3. Enable HTTPS Only
Railway provides HTTPS by default - ensure all frontend calls use `https://`

---

## Cost Optimization

Railway pricing is based on usage:
- **Starter Plan**: $5/month (500 hours)
- **Developer Plan**: $20/month (500 hours + priority support)

**Tips to reduce costs:**
1. Use Railway's **sleep mode** for non-production apps
2. Monitor resource usage in dashboard
3. Optimize database queries to reduce CPU usage

---

## Scaling

When traffic increases:

1. **Vertical Scaling**:
   - Go to **Settings** → **Resources**
   - Increase memory/CPU allocation

2. **Horizontal Scaling**:
   - Railway doesn't support multiple instances yet
   - Consider using Railway for backend + separate CDN for static assets

---

## Backup Strategy

### Database Backups
Neon provides automatic backups:
1. Go to Neon console
2. Navigate to **Backups**
3. Configure daily snapshots

### Qdrant Backups
1. Use Qdrant's snapshot API
2. Schedule weekly exports
3. Store in Railway volumes or S3

---

## Summary

### ✅ What You Need to Copy-Paste

**1. Railway Root Directory Setting:**
```
backend
```

**2. Railway Build Command:**
```bash
pip install --upgrade pip && pip install -r requirements.txt && python migrations/run_migrations.py
```

**3. Railway Start Command:**
```bash
uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

**4. Environment Variables:**
Copy all variables from `.env` to Railway (except `BACKEND_PORT` - use `PORT=${{PORT}}` instead)

**5. Python Version (create `backend/runtime.txt`):**
```
python-3.11.7
```

---

## Next Steps After Deployment

1. **Update Frontend**: Change `REACT_APP_BACKEND_URL` in frontend to your Railway URL
2. **Test All Endpoints**: Use the health checks and RAG query endpoints
3. **Monitor Logs**: Watch for any errors in the first hour
4. **Load Test**: Send sample queries to verify performance
5. **Set Up Alerts**: Configure Railway notifications for downtime

---

## Support

- **Railway Docs**: https://docs.railway.app
- **Railway Discord**: https://discord.gg/railway
- **Project Issues**: Check logs in Railway dashboard

---

**Deployment Checklist:**

- [ ] Added `psycopg2-binary` to `requirements.txt` ✅
- [ ] Set Railway root directory to `backend`
- [ ] Added all environment variables
- [ ] Configured build command with migrations
- [ ] Configured start command with uvicorn
- [ ] Created `runtime.txt` with Python version
- [ ] Tested health endpoints after deployment
- [ ] Updated frontend with Railway backend URL
- [ ] Verified RAG queries work
- [ ] Set up custom domain (optional)
- [ ] Configured monitoring alerts

---

**Your Railway URL will be:**
`https://[your-project-name].railway.app`

Use this URL in your frontend configuration!
