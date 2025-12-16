# Railway Deployment Guide - GitHub Dashboard Method

This guide shows you how to deploy your FastAPI backend to Railway using the **GitHub integration through the Railway dashboard** (no CLI required, works even on limited/trial plans).

## Why This Method Works

- ✅ **No CLI limits** - Bypasses the `railway up` restriction
- ✅ **Works on trial plans** - GitHub integration is available
- ✅ **Auto-deploys** - Automatically deploys when you push to GitHub
- ✅ **Easy rollbacks** - One-click rollback to previous deployments
- ✅ **Better logs** - Visual dashboard for monitoring

## Prerequisites

1. ✅ GitHub account
2. ✅ Your code pushed to GitHub
3. ✅ Railway account (you're already logged in as tafs4920@outlook.com)
4. ✅ Environment variables ready (Neon DB, Qdrant, Gemini API key)

## Step 1: Push Code to GitHub

If you haven't already pushed your code to GitHub:

```bash
# Navigate to project root
cd /mnt/d/PhysicalAi_HumanoidRobotics_TEXTBOOK

# Check if remote exists
git remote -v

# If no remote exists, add one
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# Add and commit the Railway configuration files
git add backend/railway.json backend/Procfile backend/runtime.txt backend/nixpacks.toml
git commit -m "Add Railway deployment configuration"

# Push to GitHub
git push -u origin main
```

## Step 2: Open Railway Dashboard

1. Go to: https://railway.app/dashboard
2. You should already be logged in as **tafs4920@outlook.com**
3. Click on your existing project: **bountiful-reprieve**
   - Or create a new project by clicking **"+ New Project"**

## Step 3: Connect GitHub Repository

### Option A: Using Existing Project (bountiful-reprieve)

1. Open your project: https://railway.com/project/0ab94e0d-e929-42ed-b28c-0de20e8d6b06
2. Click **"+ New"** button
3. Select **"GitHub Repo"**
4. If prompted, click **"Configure GitHub App"** to authorize Railway
5. Select your repository: `PhysicalAi_HumanoidRobotics_TEXTBOOK`
6. Click **"Add variables"** (we'll do this next)

### Option B: Creating New Project

1. Click **"+ New Project"**
2. Select **"Deploy from GitHub repo"**
3. If this is your first time:
   - Click **"Configure GitHub App"**
   - Select which repositories Railway can access
   - Choose: **"Only select repositories"**
   - Select: `PhysicalAi_HumanoidRobotics_TEXTBOOK`
   - Click **"Install & Authorize"**
4. Back in Railway, select your repository
5. Railway will detect it's a Python project

## Step 4: Configure Root Directory (IMPORTANT)

Since your backend is in a subdirectory:

1. In the service settings, find **"Root Directory"**
2. Set it to: `backend`
3. Click **"Save"**

This tells Railway to deploy only the `backend` folder.

## Step 5: Set Environment Variables

Click on **"Variables"** tab and add these:

### Required Variables:

```bash
# Database
POSTGRES_URL=postgresql://your-neon-database-url

# Qdrant Vector Database
QDRANT_URL=https://your-qdrant-cluster.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# LLM Provider
GEMINI_API_KEY=your-gemini-api-key
MODEL_PROVIDER=gemini
EMBEDDING_PROVIDER=gemini

# FastAPI Configuration
HOST=0.0.0.0
PORT=${{PORT}}
DEBUG=false

# CORS (use Railway's generated domain)
BACKEND_CORS_ORIGINS=["https://${{RAILWAY_PUBLIC_DOMAIN}}"]

# Performance
MAX_CONCURRENT_USERS=10
RESPONSE_TIMEOUT=30
RETRIEVAL_TOP_K=5
```

**Tips:**
- Railway automatically provides `$PORT` variable
- Use `${{RAILWAY_PUBLIC_DOMAIN}}` for the auto-generated domain
- Click **"+ Add Variable"** for each one
- Or use **"Raw Editor"** to paste all at once

## Step 6: Configure Build Settings (Optional)

Railway should auto-detect Python, but you can verify:

1. Click **"Settings"** tab
2. Under **"Build"**:
   - **Builder**: Should show "Nixpacks" (auto-detected)
   - **Build Command**: `pip install -r requirements.txt` (from railway.json)
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT` (from Procfile)

3. Under **"Deploy"**:
   - **Health Check Path**: `/health` (from railway.json)
   - **Restart Policy**: ON_FAILURE
   - **Max Retries**: 10

## Step 7: Deploy!

Railway will automatically start deploying after you save the variables.

You can also manually trigger a deployment:
1. Click **"Deployments"** tab
2. Click **"Deploy"** button
3. Watch the build logs in real-time

## Step 8: Monitor Deployment

### Watch Build Logs:
- Click on the active deployment
- View real-time logs
- Look for:
  ```
  Building...
  Installing dependencies...
  Starting application...
  Uvicorn running on http://0.0.0.0:8000
  Application startup complete
  ```

### Check Health:
- Once deployed, Railway gives you a URL like:
  ```
  https://bountiful-reprieve-production.up.railway.app
  ```
- Click **"View Logs"** to see runtime logs
- Click **"Open App"** to test in browser

## Step 9: Test Your Deployment

### Test endpoints:

```bash
# Replace with your Railway domain
RAILWAY_URL="https://bountiful-reprieve-production.up.railway.app"

# Health check
curl $RAILWAY_URL/health

# Root endpoint
curl $RAILWAY_URL/

# API docs
curl $RAILWAY_URL/docs
```

Or open in browser:
- Health: `https://your-app.up.railway.app/health`
- API Docs: `https://your-app.up.railway.app/docs`
- OpenAPI: `https://your-app.up.railway.app/openapi.json`

## Step 10: Enable Auto-Deployments (Recommended)

1. Go to **"Settings"** → **"Service"**
2. Find **"Deployment Triggers"**
3. Enable **"Automatically deploy on push to main"**
4. Now every `git push` automatically deploys!

## Troubleshooting

### Issue: "Your account is on a limited plan"

**Solution**: This shouldn't happen with GitHub integration. If it does:
1. Delete the CLI-created project
2. Create a new project via dashboard
3. Use GitHub integration method

### Issue: Build fails with "Module not found"

**Solution**:
1. Check **Root Directory** is set to `backend`
2. Verify `requirements.txt` exists
3. Check build logs for specific missing module
4. Ensure `railway.json` has correct build command

### Issue: "Application failed to start"

**Solution**:
1. Check runtime logs: **Deployments** → **View Logs**
2. Common causes:
   - Missing environment variables (check Variables tab)
   - Port mismatch (ensure using `$PORT` variable)
   - Database connection failure (verify `POSTGRES_URL`)
   - Qdrant connection failure (verify `QDRANT_URL` and `QDRANT_API_KEY`)

### Issue: Health check fails

**Solution**:
1. Verify `/health` endpoint works locally
2. Check **Settings** → **Health Check Path** is `/health`
3. Increase **Health Check Timeout** to 100 seconds
4. View logs to see actual error

### Issue: CORS errors from frontend

**Solution**:
1. Update `BACKEND_CORS_ORIGINS` variable:
   ```
   BACKEND_CORS_ORIGINS=["https://your-frontend-domain.com","https://${{RAILWAY_PUBLIC_DOMAIN}}"]
   ```
2. Redeploy the service

## Managing Your Deployment

### View Logs:
```
Deployments → [Active Deployment] → View Logs
```

### Restart Service:
```
Settings → Restart
```

### Rollback Deployment:
```
Deployments → [Previous Deployment] → Redeploy
```

### Update Environment Variables:
```
Variables → Edit → Save (auto-redeploys)
```

### Custom Domain (Optional):
```
Settings → Domains → Add Custom Domain
```

## Railway Free Tier Limits (Trial Plan)

- **$5 free credits/month**
- **500 hours of usage**
- **100 GB egress**
- **No credit card required for trial**

**To stay within limits:**
- Monitor usage in dashboard
- Use health checks to auto-restart on failure
- Consider upgrading to Hobby plan ($5/month) for production use

## CI/CD Workflow (Auto-Deploy on Push)

Once set up, your workflow is:

```bash
# Make changes locally
git add .
git commit -m "Update feature"
git push origin main

# Railway automatically:
# 1. Detects push
# 2. Builds new image
# 3. Runs tests (if configured)
# 4. Deploys
# 5. Runs health checks
# 6. Switches traffic to new deployment
```

## Next Steps

1. ✅ Set up PostgreSQL database on Railway (if you want to move from Neon)
2. ✅ Configure custom domain
3. ✅ Set up monitoring and alerts
4. ✅ Add staging environment
5. ✅ Configure GitHub Actions for testing before deploy

## Support

- Railway Docs: https://docs.railway.app
- Discord Community: https://discord.gg/railway
- Your Project: https://railway.com/project/0ab94e0d-e929-42ed-b28c-0de20e8d6b06

## Summary

**Benefits of Dashboard Deployment:**
- ✅ No CLI required
- ✅ Works on trial plan
- ✅ Visual deployment monitoring
- ✅ Easy environment variable management
- ✅ One-click rollbacks
- ✅ Auto-deploy on git push
- ✅ Built-in logging and metrics

**You're now ready to deploy! Follow Steps 1-9 above.**
