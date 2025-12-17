# Fix: Railway "No module named uvicorn" Error

## 🔍 Error Analysis

**Error:** `/app/.venv/bin/python: No module named uvicorn`

**Root Cause:**
1. Railway creates a virtual environment (`.venv`)
2. The start command is trying to use a hardcoded Python path
3. Uvicorn module isn't accessible in that environment
4. Or Railway's custom start command isn't using the right Python interpreter

---

## ✅ Complete Solution

### **1. Procfile (CREATED)**

File: `backend/Procfile`
```
web: python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

**Why this works:**
- `python -m uvicorn` uses Python's module runner (always finds uvicorn)
- Doesn't rely on uvicorn being in PATH
- Works with Railway's virtual environment
- `$PORT` uses Railway's assigned port

---

### **2. requirements.txt (VERIFIED)**

Your `requirements.txt` already includes:
```txt
uvicorn[standard]==0.27.0
```

**✅ No changes needed** - uvicorn is already listed.

---

### **3. main.py (ALREADY UPDATED)**

Your `src/main.py` already has:
- ✅ Root `/` endpoint for Railway health checks
- ✅ Non-blocking startup with error handling
- ✅ Proper logging for PORT and environment variables
- ✅ Resilient initialization (continues even if Qdrant/DB fails)

**✅ No changes needed** - all fixes are in place.

---

## 🚀 Deployment Instructions

### **Step 1: Commit the Procfile**

```bash
cd C:\Users\User\Desktop\book

# Add the Procfile
git add backend/Procfile

# Commit
git commit -m "Add Procfile for Railway deployment - fix uvicorn module error"

# Push to trigger Railway deployment
git push origin main
```

---

### **Step 2: Railway Configuration**

Go to Railway Dashboard → Your Project

#### **A. Settings → General**
```
Root Directory: backend
```

#### **B. Settings → Deploy**

**Remove custom start command** - Railway will use Procfile automatically

**Custom Build Command:**
```bash
pip install --upgrade pip && pip install -r requirements.txt && python migrations/run_migrations.py
```

**Custom Start Command:**
```
LEAVE EMPTY - Railway will use Procfile
```

**⚠️ IMPORTANT:** If you have a custom start command set, **delete it**. Railway prioritizes Procfile over custom commands.

---

#### **C. Variables Tab**

Ensure these environment variables are set:

```bash
PORT=${{PORT}}
BACKEND_HOST=0.0.0.0
GOOGLE_API_KEY=<your-actual-key>
OPENAI_API_KEY=<your-actual-key>
QDRANT_URL=https://7262bd29-47c1-4eeb-9376-3ac95ad2b9b6.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=<your-actual-key>
QDRANT_COLLECTION_NAME=my_1st_ai_book
QDRANT_VECTOR_SIZE=1536
NEON_DB_URL=<your-actual-connection-string>
BETTERAUTH_SECRET=<your-actual-secret>
BOOK_TITLE=Embodied AI: Building Intelligent Robotic Systems
```

---

### **Step 3: Redeploy**

1. Click **"Redeploy"** in Railway dashboard
2. Or push to git (Railway auto-deploys)

---

### **Step 4: Monitor Logs**

Railway Dashboard → Deployments → **View Logs**

**Success indicators:**
```
✓ [build] Successfully installed uvicorn-0.27.0
✓ [build] Running migrations...
✓ [build] All migrations completed successfully!
✓ [deploy] INFO: Started server process
✓ [deploy] INFO: Starting app on PORT: 8080
✓ [deploy] INFO: ✓ Qdrant collection ready
✓ [deploy] INFO: 🚀 FastAPI app startup complete
✓ [deploy] INFO: Uvicorn running on http://0.0.0.0:8080
```

**If you still see the error:**
```
❌ /app/.venv/bin/python: No module named uvicorn
```

This means Railway is still using a custom start command. **Solution:**
1. Go to Settings → Deploy
2. **Delete** the custom start command
3. Make sure it's empty
4. Railway will then use the Procfile
5. Redeploy

---

### **Step 5: Test Your Deployment**

```bash
# Replace with your Railway domain
curl https://your-app.railway.app/

# Expected response:
{
  "status": "online",
  "service": "AI Book RAG API",
  "version": "1.0.0",
  "message": "Backend is running. Use /health for detailed health checks."
}
```

**Test health endpoint:**
```bash
curl https://your-app.railway.app/health
```

---

## 🔧 Alternative Solutions

### **Option 1: Use Procfile (RECOMMENDED ✅)**
```
web: python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

**Pros:**
- Standard Railway practice
- Automatically detected
- Works with virtual environments
- Clean separation of concerns

---

### **Option 2: Custom Start Command**

If you prefer not to use Procfile, set this in Railway Settings → Deploy:

```bash
python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

**⚠️ Note:** You must use `python -m uvicorn`, NOT just `uvicorn`.

---

### **Option 3: nixpacks.toml (Advanced)**

Create `backend/nixpacks.toml`:

```toml
[phases.setup]
nixPkgs = ["python39"]

[phases.install]
cmds = ["pip install --upgrade pip", "pip install -r requirements.txt"]

[phases.build]
cmds = ["python migrations/run_migrations.py"]

[start]
cmd = "python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT"
```

**Pros:**
- Full control over build process
- Can specify Python version
- Advanced configuration options

---

## 📊 Why "python -m uvicorn" Fixes the Error

### **The Problem:**
```bash
# Railway tries to run:
uvicorn src.main:app

# But uvicorn executable isn't in PATH or .venv/bin
# Result: "No module named uvicorn"
```

### **The Solution:**
```bash
# Use Python's module runner:
python -m uvicorn src.main:app

# This tells Python to find and run uvicorn as a module
# Works regardless of PATH or virtual environment setup
```

**Comparison:**

| Command | Issue | Status |
|---------|-------|--------|
| `uvicorn src.main:app` | Requires uvicorn in PATH | ❌ Fails |
| `/app/.venv/bin/uvicorn` | Hardcoded path may not exist | ❌ Fails |
| `python -m uvicorn` | Uses Python module system | ✅ Always works |

---

## 🚨 Common Mistakes to Avoid

### ❌ **Mistake 1: Using "uvicorn" directly**
```bash
# Wrong:
uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

### ✅ **Correct:**
```bash
# Right:
python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT
```

---

### ❌ **Mistake 2: Hardcoding Python path**
```bash
# Wrong:
/app/.venv/bin/python -m uvicorn src.main:app
```

### ✅ **Correct:**
```bash
# Right:
python -m uvicorn src.main:app
```

---

### ❌ **Mistake 3: Both Procfile and Custom Start Command**
- Railway prioritizes custom start command over Procfile
- If you have both, **remove the custom start command**

### ✅ **Correct:**
- Use **either** Procfile **or** custom start command
- **Recommended:** Use Procfile (more standard)

---

### ❌ **Mistake 4: Wrong working directory**
```bash
# Wrong (if Procfile is in root):
cd backend && python -m uvicorn src.main:app
```

### ✅ **Correct:**
```bash
# Right (Railway already sets working directory to "backend"):
python -m uvicorn src.main:app
```

---

## 🎯 Quick Checklist

Before deploying, verify:

- [ ] **Procfile created** in `backend/` directory
- [ ] **Procfile content:** `web: python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- [ ] **Railway root directory:** Set to `backend`
- [ ] **Custom start command:** Removed (let Railway use Procfile)
- [ ] **Environment variables:** All set, especially `PORT=${{PORT}}`
- [ ] **requirements.txt:** Contains `uvicorn[standard]==0.27.0`
- [ ] **Git committed:** Procfile added and pushed

---

## 🔍 Debugging Tips

### **1. Check Railway Detection**

In deployment logs, look for:
```
✓ Detected Procfile
✓ Using Procfile for start command
```

If you don't see this, Railway isn't detecting the Procfile.

**Solutions:**
- Ensure Procfile is in the `backend/` directory (your root directory)
- Check file name is exactly `Procfile` (capital P, no extension)
- Remove custom start command from Railway settings

---

### **2. Verify uvicorn Installation**

In build logs:
```
✓ Successfully installed uvicorn-0.27.0 watchfiles-0.21.0 websockets-12.0
```

If missing, check `requirements.txt` includes `uvicorn[standard]`.

---

### **3. Check Python Path**

In deployment logs, you should see:
```
INFO: Started server process
INFO: Uvicorn running on http://0.0.0.0:8080
```

If you see `/app/.venv/bin/python: No module named uvicorn`, Railway is still using the wrong command.

---

### **4. Use Railway Shell**

Railway Dashboard → Settings → Shell → Connect

```bash
# Check if uvicorn is installed
python -c "import uvicorn; print(uvicorn.__version__)"

# Should output: 0.27.0

# Check Python location
which python

# Should output: /app/.venv/bin/python or similar

# Test running uvicorn
python -m uvicorn --version

# Should output: Running uvicorn 0.27.0
```

---

## 📚 Additional Resources

- **Procfile Docs:** https://docs.railway.app/deploy/deployments#procfile
- **Railway Python Guide:** https://docs.railway.app/guides/python
- **Uvicorn Docs:** https://www.uvicorn.org/

---

## 🎯 Expected Outcome

After following these steps:

1. ✅ Procfile created and committed
2. ✅ Railway uses Procfile automatically
3. ✅ uvicorn starts without "No module named" error
4. ✅ App responds on Railway's assigned PORT
5. ✅ Root `/` endpoint returns 200 OK
6. ✅ Health checks pass

**Deployment time:** 2-4 minutes from push to live

---

## 💡 Summary

**The Error:**
```
/app/.venv/bin/python: No module named uvicorn
```

**The Fix:**
1. Create `Procfile` with: `web: python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT`
2. Remove custom start command from Railway settings
3. Commit and push
4. Railway uses Procfile → ✅ Works!

**Why It Works:**
- `python -m uvicorn` uses Python's module system
- Doesn't depend on PATH or hardcoded paths
- Works in any virtual environment
- Railway standard practice

---

**Your deployment should now work! 🎉**

Commit the Procfile, push, and watch the Railway logs.
