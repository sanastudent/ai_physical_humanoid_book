#!/usr/bin/env python3
"""
Embed All Chapters Script
Reads all markdown files from docs/chapters and sends them to /embed endpoint
"""
import os
import requests
import json
from pathlib import Path

# Configuration
BACKEND_URL = "http://localhost:8000"
CHAPTERS_DIR = "frontend/my-book/docs/chapters"
BOOK_ID = "physical-ai-humanoid"

def get_chapter_files():
    """Get all markdown files from chapters directory"""
    chapters_path = Path(CHAPTERS_DIR)
    return sorted(chapters_path.glob("*.md"))

def read_chapter_content(file_path):
    """Read content from markdown file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

def extract_chapter_name(file_path):
    """Extract chapter name from file path"""
    return file_path.stem

def embed_chapter(chapter_name, content):
    """Send chapter content to /embed endpoint"""
    url = f"{BACKEND_URL}/embed"

    payload = {
        "content": content,
        "chapter": chapter_name,
        "book_id": BOOK_ID
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        print(f"Embedding: {chapter_name}...")
        response = requests.post(url, json=payload, headers=headers)
        response.raise_for_status()

        result = response.json()
        print(f"  ✓ Success: {result.get('chunks_created', 0)} chunks created")
        return True

    except requests.exceptions.RequestException as e:
        print(f"  ✗ Error: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"  Response: {e.response.text}")
        return False

def main():
    """Main function to embed all chapters"""
    print("="*60)
    print("EMBEDDING ALL CHAPTERS")
    print("="*60)
    print(f"Backend: {BACKEND_URL}")
    print(f"Chapters: {CHAPTERS_DIR}")
    print(f"Book ID: {BOOK_ID}")
    print("="*60)

    # Get all chapter files
    chapter_files = get_chapter_files()

    if not chapter_files:
        print(f"No chapter files found in {CHAPTERS_DIR}")
        return

    print(f"\nFound {len(chapter_files)} chapters\n")

    # Process each chapter
    success_count = 0
    failure_count = 0

    for file_path in chapter_files:
        chapter_name = extract_chapter_name(file_path)
        content = read_chapter_content(file_path)

        if embed_chapter(chapter_name, content):
            success_count += 1
        else:
            failure_count += 1

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Total chapters: {len(chapter_files)}")
    print(f"Successful: {success_count}")
    print(f"Failed: {failure_count}")
    print("="*60)

if __name__ == "__main__":
    main()
