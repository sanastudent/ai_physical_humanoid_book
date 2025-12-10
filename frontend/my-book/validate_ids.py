#!/usr/bin/env python3
"""
Script to validate that Urdu translations have matching IDs with English docs.
"""

import os
import re
from pathlib import Path
import json

def extract_id_from_frontmatter(content):
    """Extract ID from YAML frontmatter if present."""
    frontmatter_pattern = r'^---\n(.*?)\n---'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if match:
        frontmatter = match.group(1)
        # Look for id field
        id_match = re.search(r'^id:\s*(.+)$', frontmatter, re.MULTILINE)
        if id_match:
            return id_match.group(1).strip().strip('"\'')

        # Also check for slug
        slug_match = re.search(r'^slug:\s*(.+)$', frontmatter, re.MULTILINE)
        if slug_match:
            slug_value = slug_match.group(1).strip().strip('"\'')
            return f"slug:{slug_value}"

    return None

def get_doc_metadata(file_path):
    """Get metadata from a doc file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    doc_id = extract_id_from_frontmatter(content)
    base_dir = 'docs' if 'docs' in str(file_path) else 'i18n/ur/docusaurus-plugin-content-docs/current'
    rel_path = os.path.relpath(file_path, base_dir)
    # Normalize path separators to forward slashes
    rel_path = rel_path.replace('\\', '/')

    return {
        'path': rel_path,
        'id': doc_id,
        'has_id': doc_id is not None
    }

def validate_id_matching():
    """Validate that English and Urdu docs have matching IDs."""
    docs_dir = Path('docs')
    urdu_dir = Path('i18n/ur/docusaurus-plugin-content-docs/current')

    english_docs = {}
    urdu_docs = {}

    print("Scanning English docs...")
    for md_file in docs_dir.rglob('*.md'):
        if 'node_modules' in str(md_file):
            continue
        try:
            metadata = get_doc_metadata(str(md_file))
            english_docs[metadata['path']] = metadata
        except Exception as e:
            print(f"Error reading {md_file}: {e}")

    for mdx_file in docs_dir.rglob('*.mdx'):
        if 'node_modules' in str(mdx_file):
            continue
        try:
            metadata = get_doc_metadata(str(mdx_file))
            english_docs[metadata['path']] = metadata
        except Exception as e:
            print(f"Error reading {mdx_file}: {e}")

    print(f"Found {len(english_docs)} English docs")

    print("\nScanning Urdu docs...")
    for md_file in urdu_dir.rglob('*.md'):
        try:
            metadata = get_doc_metadata(str(md_file))
            urdu_docs[metadata['path']] = metadata
        except Exception as e:
            print(f"Error reading {md_file}: {e}")

    for mdx_file in urdu_dir.rglob('*.mdx'):
        try:
            metadata = get_doc_metadata(str(mdx_file))
            urdu_docs[metadata['path']] = metadata
        except Exception as e:
            print(f"Error reading {mdx_file}: {e}")

    print(f"Found {len(urdu_docs)} Urdu docs")

    # Validate matching
    print("\n" + "="*60)
    print("VALIDATION RESULTS")
    print("="*60)

    mismatches = []
    matches = []
    missing_in_urdu = []

    for path, en_meta in english_docs.items():
        if path in urdu_docs:
            ur_meta = urdu_docs[path]
            if en_meta['id'] == ur_meta['id']:
                matches.append({
                    'path': path,
                    'id': en_meta['id'],
                    'status': 'match'
                })
            else:
                mismatches.append({
                    'path': path,
                    'english_id': en_meta['id'],
                    'urdu_id': ur_meta['id'],
                    'status': 'mismatch'
                })
        else:
            missing_in_urdu.append(path)

    # Report results
    print(f"\n+ Matching IDs: {len(matches)}")
    print(f"! Mismatched IDs: {len(mismatches)}")
    print(f"- Missing in Urdu: {len(missing_in_urdu)}")

    if mismatches:
        print("\nMismatched IDs:")
        for mismatch in mismatches:
            print(f"  {mismatch['path']}")
            print(f"    English: {mismatch['english_id']}")
            print(f"    Urdu: {mismatch['urdu_id']}")

    if missing_in_urdu:
        print("\nMissing in Urdu:")
        for path in missing_in_urdu:
            print(f"  - {path}")

    # Check for docs with explicit IDs
    docs_with_ids = [m for m in matches if m['id'] is not None]
    print(f"\nDocs with explicit IDs (preserved): {len(docs_with_ids)}")
    if docs_with_ids:
        for doc in docs_with_ids[:10]:  # Show first 10
            print(f"  + {doc['path']} (id: {doc['id']})")
        if len(docs_with_ids) > 10:
            print(f"  ... and {len(docs_with_ids) - 10} more")

    # Save validation report
    report = {
        'total_english_docs': len(english_docs),
        'total_urdu_docs': len(urdu_docs),
        'matches': len(matches),
        'mismatches': len(mismatches),
        'missing_in_urdu': len(missing_in_urdu),
        'docs_with_ids': len(docs_with_ids),
        'validation_passed': len(mismatches) == 0 and len(missing_in_urdu) == 0,
        'details': {
            'matches': matches,
            'mismatches': mismatches,
            'missing_in_urdu': missing_in_urdu
        }
    }

    with open('id_validation_report.json', 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n{'PASSED' if report['validation_passed'] else 'FAILED'}: ID validation")
    print(f"Report saved to: id_validation_report.json")

    return report

if __name__ == '__main__':
    print("Validating ID matching between English and Urdu docs...\n")
    report = validate_id_matching()
