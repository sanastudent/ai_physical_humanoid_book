#!/usr/bin/env python3
"""
Fixed Embedding Script for Book Chapters

This script reads all markdown files from docs/chapters and sends them
to the /embed endpoint. The backend now generates valid UUIDs for each
chunk, which Qdrant requires.

Key Changes:
1. Backend (embed.py) now generates UUID instead of string IDs
2. Each chunk gets a unique UUID that Qdrant accepts
3. Chapter name and book_id are stored in payload metadata
4. Better error handling and progress reporting
"""
import os
import requests
import json
from pathlib import Path
import time

# Configuration
BACKEND_URL = "http://localhost:8000"
CHAPTERS_DIR = "frontend/my-book/docs/chapters"
BOOK_ID = "physical-ai-humanoid"

def get_chapter_files():
    """Get all markdown files from chapters directory"""
    chapters_path = Path(CHAPTERS_DIR)
    if not chapters_path.exists():
        print(f"ERROR: Directory not found: {CHAPTERS_DIR}")
        return []

    files = sorted(chapters_path.glob("*.md"))
    return files

def read_chapter_content(file_path):
    """Read content from markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        return content
    except Exception as e:
        print(f"ERROR reading {file_path}: {e}")
        return None

def extract_chapter_name(file_path):
    """Extract chapter name from file path"""
    return file_path.stem

def embed_chapter(chapter_name, content):
    """
    Send chapter content to /embed endpoint

    The backend will:
    1. Chunk the content into 500-token pieces with 50-token overlap
    2. Generate a UUID for each chunk (Qdrant requirement)
    3. Create embeddings using OpenAI text-embedding-3-small
    4. Store in Qdrant with metadata: chapter, text, token_count, book_id

    Args:
        chapter_name: Chapter identifier (e.g., "module1-intro")
        content: Full markdown content of the chapter

    Returns:
        tuple: (success: bool, chunks_created: int, error_msg: str)
    """
    url = f"{BACKEND_URL}/embed"

    # Payload structure that backend expects
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
        print(f"  Content length: {len(content)} characters")

        response = requests.post(url, json=payload, headers=headers, timeout=60)

        # Check for HTTP errors
        if response.status_code != 200:
            error_detail = response.json().get('detail', 'Unknown error') if response.text else 'No response'
            return False, 0, f"HTTP {response.status_code}: {error_detail}"

        result = response.json()
        chunks_created = result.get('chunks_created', 0)

        print(f"  ✓ Success: {chunks_created} chunks created with UUIDs")
        return True, chunks_created, None

    except requests.exceptions.Timeout:
        return False, 0, "Request timeout (>60s)"
    except requests.exceptions.ConnectionError:
        return False, 0, "Connection refused - is backend running?"
    except requests.exceptions.RequestException as e:
        return False, 0, f"Request error: {str(e)}"
    except json.JSONDecodeError:
        return False, 0, f"Invalid JSON response: {response.text[:200]}"
    except Exception as e:
        return False, 0, f"Unexpected error: {str(e)}"

def main():
    """Main function to embed all chapters"""
    print("=" * 70)
    print("EMBEDDING ALL CHAPTERS (FIXED VERSION)")
    print("=" * 70)
    print(f"Backend URL: {BACKEND_URL}")
    print(f"Chapters Directory: {CHAPTERS_DIR}")
    print(f"Book ID: {BOOK_ID}")
    print()
    print("KEY FIX: Backend now generates UUIDs instead of string IDs")
    print("This resolves the Qdrant 'invalid point ID' error")
    print("=" * 70)

    # Get all chapter files
    chapter_files = get_chapter_files()

    if not chapter_files:
        print(f"\nNo chapter files found in {CHAPTERS_DIR}")
        print("Please check the directory path and ensure .md files exist")
        return

    print(f"\nFound {len(chapter_files)} chapters\n")

    # Process each chapter
    success_count = 0
    failure_count = 0
    total_chunks = 0
    failed_chapters = []

    start_time = time.time()

    for i, file_path in enumerate(chapter_files, 1):
        chapter_name = extract_chapter_name(file_path)
        content = read_chapter_content(file_path)

        if content is None:
            print(f"  ✗ Skipping {chapter_name} (read error)")
            failure_count += 1
            failed_chapters.append((chapter_name, "File read error"))
            continue

        print(f"[{i}/{len(chapter_files)}] ", end="")
        success, chunks, error = embed_chapter(chapter_name, content)

        if success:
            success_count += 1
            total_chunks += chunks
        else:
            failure_count += 1
            failed_chapters.append((chapter_name, error))
            print(f"  ✗ Error: {error}")

        # Small delay to avoid overwhelming the server
        time.sleep(0.5)

    elapsed_time = time.time() - start_time

    # Summary
    print("\n" + "=" * 70)
    print("EMBEDDING SUMMARY")
    print("=" * 70)
    print(f"Total chapters processed: {len(chapter_files)}")
    print(f"Successful: {success_count}")
    print(f"Failed: {failure_count}")
    print(f"Total chunks created: {total_chunks}")
    print(f"Time elapsed: {elapsed_time:.2f} seconds")

    if failed_chapters:
        print("\nFailed Chapters:")
        for chapter, error in failed_chapters:
            print(f"  - {chapter}: {error}")

    print("=" * 70)

    # Verification suggestion
    if success_count > 0:
        print("\nVerify embeddings in Qdrant:")
        print(f"  curl {BACKEND_URL}/health/qdrant")
        print("\nExpected response should show vectors_count > 0")

if __name__ == "__main__":
    main()
