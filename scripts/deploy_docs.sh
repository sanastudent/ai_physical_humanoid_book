#!/bin/bash

# Deploy Docusaurus book to GitHub Pages

set -e

echo "===================================="
echo "Deploying Book to GitHub Pages"
echo "===================================="

# Check if Git is configured
if [ -z "$(git config user.name)" ]; then
    echo "Error: Git user.name is not configured"
    exit 1
fi

if [ -z "$(git config user.email)" ]; then
    echo "Error: Git user.email is not configured"
    exit 1
fi

# Navigate to frontend
cd frontend/my-book

# Build the site
echo "\n[1/3] Building Docusaurus site..."
npm run build

# Deploy to GitHub Pages
echo "\n[2/3] Deploying to GitHub Pages..."
GIT_USER=$(git config user.name) npm run deploy

echo "\n[3/3] Deployment complete!"
echo "\nYour book should be available at:"
echo "https://$(git config user.name).github.io/book/"
echo "\n===================================="
