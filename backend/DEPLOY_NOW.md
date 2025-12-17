# Deploy to Railway NOW - Quick Start

## ✅ What's Ready

All files are prepared and ready to deploy:

- ✅ `Procfile` - Created with correct start command
- ✅ `requirements.txt` - Has uvicorn[standard]==0.27.0
- ✅ `src/main.py` - Has root endpoint and proper logging
- ✅ Migrations - Ready to run

---

## 🚀 3-Step Deployment

### **Step 1: Commit & Push (30 seconds)**

```bash
cd C:\Users\User\Desktop\book

git add backend/Procfile
git commit -m "Add Procfile for Railway - fix uvicorn module error"
git push origin main
```

---

### **Step 2: Railway Settings (1 minute)**

Go to Railway Dashboard → Your Project

#### **A. Settings → General**
```
Root Directory: backend
```

#### **B. Settings → Deploy**

**Custom Build Command:**
```bash
pip install --upgrade pip && pip install -r requirements.txt && python migrations/run_migrations.py
```

**Custom Start Command:**
```
(LEAVE EMPTY - delete if exists)
```

**⚠️ CRITICAL:** Remove any existing custom start command. Railway will use Procfile.

#### **C. Variables Tab**

Verify these are set:
```bash
PORT=${{PORT}}
BACKEND_HOST=0.0.0.0
```

Plus all your API keys and database URLs (from .env file).

---

### **Step 3: Deploy & Test (2-4 minutes)**

1. Click **"Redeploy"** in Railway
2. Watch logs for: `INFO: Uvicorn running on http://0.0.0.0:8080`
3. Test: `curl https://your-app.railway.app/`

---

## 🔍 Success Indicators

**In Railway Logs:**
```
✓ Successfully installed uvicorn-0.27.0
✓ INFO: Started server process
✓ INFO: Starting app on PORT: 8080
✓ INFO: 🚀 FastAPI app startup complete
✓ INFO: Uvicorn running on http://0.0.0.0:8080
```

**Test Your URL:**
```bash
curl https://your-app.railway.app/

# Expected:
{"status":"online","service":"AI Book RAG API","version":"1.0.0",...}
```

---

## 🚨 If Error Persists

**Still seeing "No module named uvicorn"?**

→ Railway is ignoring the Procfile

**Fix:**
1. Go to Settings → Deploy
2. Find "Custom Start Command"
3. **Delete it completely** (make it empty)
4. Save and redeploy

Railway will then use the Procfile.

---

## 📋 Procfile Content (for reference)

File: `backend/Procfile`
```
web: python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

**Why this works:**
- Uses `python -m` (module runner)
- No hardcoded paths
- Works with Railway's virtual environment

---

## 🎯 TL;DR

**3 Commands:**
```bash
git add backend/Procfile
git commit -m "Add Procfile for Railway"
git push
```

**1 Railway Setting:**
- Remove custom start command (let Railway use Procfile)

**Done!** 🎉

---

## 💡 Pro Tips

1. **Procfile must be in `backend/` directory** (your Railway root)
2. **File name is case-sensitive:** `Procfile` (capital P)
3. **No file extension** (not `Procfile.txt`)
4. **Remove custom start command** so Railway uses Procfile

---

## 📞 Need Help?

See detailed guide: `RAILWAY_UVICORN_FIX.md`

Full troubleshooting: `RAILWAY_TROUBLESHOOTING.md`

---

**Ready to deploy? Run the git commands above! 🚀**
