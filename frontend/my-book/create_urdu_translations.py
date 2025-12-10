#!/usr/bin/env python3
"""
Script to create Urdu translation files from English docs with preserved metadata.
Preserves IDs, slugs, and sidebar positions while adding placeholder content.
"""

import os
import re
from pathlib import Path
import json

def extract_frontmatter(content):
    """Extract YAML frontmatter from markdown content."""
    frontmatter_pattern = r'^---\n(.*?)\n---\n(.*)$'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if match:
        frontmatter = match.group(1)
        body = match.group(2)
        return frontmatter, body
    return None, content

def create_urdu_doc(english_path, urdu_path):
    """Create Urdu translation file from English doc with preserved metadata."""
    # Read English file
    with open(english_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter and body
    frontmatter, body = extract_frontmatter(content)

    # Get relative path for display
    rel_path = os.path.relpath(english_path, 'docs')

    # Create Urdu content with preserved frontmatter
    if frontmatter:
        urdu_content = f"""---
{frontmatter}
---

<!--
  Urdu Translation Placeholder
  English source: {rel_path}

  TODO: Add Urdu translation below this comment.
  The frontmatter above (id, title, sidebar_position, etc.) has been preserved
  to ensure proper chapter ID matching and navigation.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

{body.strip()}
"""
    else:
        # No frontmatter, just add placeholder
        urdu_content = f"""<!--
  Urdu Translation Placeholder
  English source: {rel_path}

  TODO: Add Urdu translation below this comment.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

{content.strip()}
"""

    # Create directory if needed
    os.makedirs(os.path.dirname(urdu_path), exist_ok=True)

    # Write Urdu file
    with open(urdu_path, 'w', encoding='utf-8') as f:
        f.write(urdu_content)

    return {
        'english_path': rel_path,
        'urdu_path': os.path.relpath(urdu_path, '.'),
        'has_frontmatter': frontmatter is not None,
        'frontmatter': frontmatter if frontmatter else 'none'
    }

def scan_and_create_translations():
    """Scan all English docs and create Urdu translations."""
    docs_dir = Path('docs')
    urdu_dir = Path('i18n/ur/docusaurus-plugin-content-docs/current')

    results = []
    file_count = 0

    # Walk through all markdown files
    for md_file in docs_dir.rglob('*.md'):
        # Skip node_modules and hidden directories
        if 'node_modules' in str(md_file) or '/.' in str(md_file):
            continue

        # Calculate relative path and Urdu path
        rel_path = md_file.relative_to(docs_dir)
        urdu_file = urdu_dir / rel_path

        # Create Urdu translation
        result = create_urdu_doc(str(md_file), str(urdu_file))
        results.append(result)
        file_count += 1

        print(f"+ Created: {result['urdu_path']}")

    # Also handle .mdx files
    for mdx_file in docs_dir.rglob('*.mdx'):
        if 'node_modules' in str(mdx_file) or '/.' in str(mdx_file):
            continue

        rel_path = mdx_file.relative_to(docs_dir)
        urdu_file = urdu_dir / rel_path

        result = create_urdu_doc(str(mdx_file), str(urdu_file))
        results.append(result)
        file_count += 1

        print(f"+ Created: {result['urdu_path']}")

    # Save results to JSON
    summary = {
        'total_files': file_count,
        'files': results,
        'timestamp': str(Path.cwd())
    }

    with open('urdu_translation_summary.json', 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"\nCreated {file_count} Urdu translation files")
    print(f"Summary saved to: urdu_translation_summary.json")

    return summary

if __name__ == '__main__':
    print("Creating Urdu translation files with preserved metadata...\n")
    summary = scan_and_create_translations()

    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Total files created: {summary['total_files']}")
    print("\nFiles with preserved frontmatter:")
    for item in summary['files']:
        if item['has_frontmatter']:
            print(f"  + {item['english_path']}")
