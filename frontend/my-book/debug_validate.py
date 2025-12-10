from pathlib import Path
import os
import re

def extract_id_from_frontmatter(content):
    """Extract ID from YAML frontmatter if present."""
    frontmatter_pattern = r'^---\n(.*?)\n---'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if match:
        frontmatter = match.group(1)
        id_match = re.search(r'^id:\s*(.+)$', frontmatter, re.MULTILINE)
        if id_match:
            return id_match.group(1).strip().strip('"\'')
        slug_match = re.search(r'^slug:\s*(.+)$', frontmatter, re.MULTILINE)
        if slug_match:
            slug_value = slug_match.group(1).strip().strip('"\'')
            return f"slug:{slug_value}"

    return None

def get_doc_metadata(file_path, base_dir):
    """Get metadata from a doc file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    doc_id = extract_id_from_frontmatter(content)
    rel_path = os.path.relpath(file_path, base_dir).replace('\\', '/')

    return {
        'path': rel_path,
        'id': doc_id,
        'has_id': doc_id is not None
    }

# Scan docs
docs_dir = 'docs'
urdu_dir = 'i18n/ur/docusaurus-plugin-content-docs/current'

english_docs = {}
urdu_docs = {}

for md_file in Path(docs_dir).rglob('*.md'):
    if 'node_modules' not in str(md_file):
        metadata = get_doc_metadata(str(md_file), docs_dir)
        english_docs[metadata['path']] = metadata

for md_file in Path(urdu_dir).rglob('*.md'):
    metadata = get_doc_metadata(str(md_file), urdu_dir)
    urdu_docs[metadata['path']] = metadata

print(f"English docs: {len(english_docs)}")
print(f"Urdu docs: {len(urdu_docs)}")

print("\nFirst 5 English paths:")
for p in list(english_docs.keys())[:5]:
    print(f"  '{p}'")

print("\nFirst 5 Urdu paths:")
for p in list(urdu_docs.keys())[:5]:
    print(f"  '{p}'")

print("\nChecking if 'intro.md' exists:")
print(f"  In English: {'intro.md' in english_docs}")
print(f"  In Urdu: {'intro.md' in urdu_docs}")

if 'intro.md' in english_docs and 'intro.md' in urdu_docs:
    print(f"  English ID: {english_docs['intro.md']['id']}")
    print(f"  Urdu ID: {urdu_docs['intro.md']['id']}")
