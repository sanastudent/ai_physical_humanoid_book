#!/usr/bin/env python3
"""
Fix broken image references in Urdu translation files.
"""

import os
import re
from pathlib import Path

def fix_image_references(file_path):
    """Remove or fix broken image references in markdown files."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern to match markdown images with relative paths
    # ![text](./img/file.png)
    pattern = r'!\[([^\]]*)\]\(\./img/[^\)]+\)'

    # Replace with a comment indicating image placeholder
    fixed_content = re.sub(
        pattern,
        r'<!-- Image: \1 (from English version) -->',
        content
    )

    if content != fixed_content:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(fixed_content)
        return True
    return False

def fix_all_urdu_docs():
    """Fix all Urdu doc files with broken image references."""
    urdu_dir = Path('i18n/ur/docusaurus-plugin-content-docs/current')
    fixed_count = 0

    for md_file in urdu_dir.rglob('*.md'):
        if fix_image_references(str(md_file)):
            print(f"Fixed: {md_file.relative_to(urdu_dir)}")
            fixed_count += 1

    for mdx_file in urdu_dir.rglob('*.mdx'):
        if fix_image_references(str(mdx_file)):
            print(f"Fixed: {mdx_file.relative_to(urdu_dir)}")
            fixed_count += 1

    print(f"\nFixed {fixed_count} files with broken image references")
    return fixed_count

if __name__ == '__main__':
    print("Fixing broken image references in Urdu translation files...\n")
    fix_all_urdu_docs()
