#!/bin/bash

# Deploy Backend to Railway/Render/Vercel
# Usage: ./scripts/deploy-backend.sh [platform]
# Platform options: railway, render, vercel

set -e

PLATFORM=${1:-railway}
BACKEND_DIR="backend"

echo "🚀 Deploying backend to $PLATFORM..."

cd "$BACKEND_DIR"

case "$PLATFORM" in
  railway)
    echo "📦 Deploying to Railway..."
    if ! command -v railway &> /dev/null; then
      echo "❌ Railway CLI not found. Install with: npm install -g @railway/cli"
      exit 1
    fi

    echo "🔐 Setting environment variables..."
    railway variables set QUADRANT_URL="$(grep QUADRANT_URL ../.env | cut -d '=' -f2-)"
    railway variables set QUADRANT_API_KEY="$(grep QUADRANT_API_KEY ../.env | cut -d '=' -f2-)"
    railway variables set POSTGRES_URL="$(grep POSTGRES_URL ../.env | cut -d '=' -f2-)"
    railway variables set GEMINI_API_KEY="$(grep GEMINI_API_KEY ../.env | cut -d '=' -f2-)"
    railway variables set MODEL_PROVIDER="gemini"
    railway variables set DEBUG="false"

    echo "🚂 Deploying to Railway..."
    railway up

    echo "🌐 Getting deployment URL..."
    BACKEND_URL=$(railway domain)
    echo "✅ Backend deployed to: $BACKEND_URL"
    ;;

  render)
    echo "📦 Deploying to Render..."
    echo "⚠️  Please deploy via Render dashboard:"
    echo "   1. Go to https://render.com"
    echo "   2. Create a new Web Service"
    echo "   3. Connect your repository"
    echo "   4. Set root directory to 'backend'"
    echo "   5. Use render.yaml for configuration"
    echo "   6. Add environment variables from .env file"
    ;;

  vercel)
    echo "📦 Deploying to Vercel..."
    if ! command -v vercel &> /dev/null; then
      echo "❌ Vercel CLI not found. Install with: npm install -g vercel"
      exit 1
    fi

    echo "🚀 Deploying to Vercel..."
    vercel --prod

    echo "✅ Deployment complete! Check Vercel dashboard for URL."
    ;;

  *)
    echo "❌ Unknown platform: $PLATFORM"
    echo "Usage: $0 [railway|render|vercel]"
    exit 1
    ;;
esac

echo ""
echo "📋 Next steps:"
echo "   1. Note your backend URL"
echo "   2. Update BACKEND_CORS_ORIGINS to include your GitHub Pages URL"
echo "   3. Add backend URL as BACKEND_API_URL secret in GitHub repository"
echo "   4. Push to main branch to deploy frontend"
echo ""
echo "🎉 Deployment complete!"
