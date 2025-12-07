"""
Script to embed book content into Qdrant vector database
"""
import os
import sys
import requests
import glob
from pathlib import Path

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'backend', 'src'))

def read_markdown_files(docs_dir):
    """Read all markdown files from docs directory"""
    book_content = {}

    docs_path = Path(docs_dir)

    # Read introduction
    intro_file = docs_path / 'introduction.md'
    if intro_file.exists():
        with open(intro_file, 'r', encoding='utf-8') as f:
            book_content['introduction'] = f.read()

    # Read chapters
    chapters_dir = docs_path / 'chapters'
    if chapters_dir.exists():
        for chapter_file in sorted(chapters_dir.glob('*.md')):
            chapter_name = chapter_file.stem
            with open(chapter_file, 'r', encoding='utf-8') as f:
                book_content[chapter_name] = f.read()

    # Read other files
    for file_name in ['summary.md', 'glossary.md', 'references.md']:
        file_path = docs_path / file_name
        if file_path.exists():
            with open(file_path, 'r', encoding='utf-8') as f:
                book_content[file_name.replace('.md', '')] = f.read()

    return book_content

def embed_book(backend_url, book_content):
    """Send book content to backend for embedding"""
    endpoint = f'{backend_url}/embed-book'

    print(f"Embedding {len(book_content)} chapters...")
    print(f"Chapters: {list(book_content.keys())}")

    try:
        response = requests.post(endpoint, json=book_content, timeout=300)
        response.raise_for_status()

        result = response.json()
        print(f"\nSuccess! Embedded {result['total_chunks']} chunks")
        print(f"Chapters processed: {result['chapters']}")

        return True
    except requests.exceptions.RequestException as e:
        print(f"\nError embedding book: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"Response: {e.response.text}")
        return False

def main():
    # Configuration
    backend_url = os.getenv('BACKEND_URL', 'http://localhost:8000')
    docs_dir = os.path.join(
        os.path.dirname(__file__),
        '..',
        'frontend',
        'my-book',
        'docs'
    )

    # Read book content
    print(f"Reading book content from {docs_dir}")
    book_content = read_markdown_files(docs_dir)

    if not book_content:
        print("Error: No book content found!")
        sys.exit(1)

    # Embed book
    success = embed_book(backend_url, book_content)

    if success:
        print("\n✓ Book embedding complete!")
        sys.exit(0)
    else:
        print("\n✗ Book embedding failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()
