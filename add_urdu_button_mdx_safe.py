#!/usr/bin/env python3
"""
Script to add MDX-safe 'Switch to Urdu' button at the top of each chapter in the Docusaurus book.
This script preserves all original content and adds only the JSX button with proper MDX syntax.
"""

import os
import re
import frontmatter
from pathlib import Path

def add_urdu_button_to_file(file_path, urdu_file_path):
    """Add MDX-safe Urdu switch button to a single file."""
    try:
        # Read the file with frontmatter
        with open(file_path, 'r', encoding='utf-8') as f:
            original_content = f.read()

        # Parse frontmatter and content
        post = frontmatter.loads(original_content)
        frontmatter_data = post.metadata
        content = post.content

        # Create the MDX-safe JSX button with the Urdu file path properly inserted
        # In MDX, to get onClick={() => {}} we need to use proper brace escaping in the template
        urdu_button_jsx_template = """import React from 'react';

<div style={{marginBottom: '20px', padding: '10px', backgroundColor: '#f0f8ff', border: '1px solid #b3d9ff', borderRadius: '4px'}}>
  <button
    style={{backgroundColor: 'green', color: 'white', padding: '5px 10px', borderRadius: '5px', cursor: 'pointer'}}
    onClick={() => {{
      const path = '%s';
      fetch(path, {{method: 'HEAD'}}).then(res => {{
        if (res.ok) window.location.href = path;
        else alert('Urdu translation is not available yet.');
      }}).catch(error => {{
        alert('Urdu translation is not available yet.');
      }});
    }}}
  >
    Urdu mein dekhein
  </button>
</div>

"""
        urdu_button_jsx = urdu_button_jsx_template % urdu_file_path

        # Combine everything: frontmatter, button, then original content
        new_content = urdu_button_jsx + content

        # Write the updated content back to the file
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(frontmatter.dumps(frontmatter.Post(new_content, **frontmatter_data)))

        return True, f"Successfully added Urdu button to {file_path}"

    except Exception as e:
        return False, f"Error processing {file_path}: {str(e)}"

def main():
    # Define the docs directory
    docs_dir = Path("frontend/my-book/docs")

    # Find all .md and .mdx files in the docs directory and subdirectories
    md_files = list(docs_dir.rglob("*.md")) + list(docs_dir.rglob("*.mdx"))

    print(f"Found {len(md_files)} markdown files to process")

    # Track results
    successful = []
    failed = []

    for md_file in md_files:
        # Determine the corresponding Urdu file path
        relative_path = md_file.relative_to(docs_dir)
        urdu_file_path = f"frontend/my-book/i18n/ur/docusaurus-plugin-content-docs/current/{relative_path.as_posix()}"

        # Add the Urdu button to the file
        success, message = add_urdu_button_to_file(md_file, urdu_file_path)

        if success:
            successful.append(str(md_file))
            print(f"[SUCCESS] {message}")
        else:
            failed.append(str(md_file))
            print(f"[ERROR] {message}")

    # Summary
    print("\n" + "="*60)
    print("URDU BUTTON ADDITION SUMMARY")
    print("="*60)
    print(f"Files successfully updated: {len(successful)}")
    print(f"Files with errors: {len(failed)}")
    print(f"Total files processed: {len(md_files)}")

    if successful:
        print("\nSuccessfully updated files:")
        for file in successful:
            print(f"  - {file}")

    if failed:
        print("\nFiles with errors:")
        for file in failed:
            print(f"  - {file}")

    # Create a log file
    log_data = {
        "summary": {
            "successful": len(successful),
            "errors": len(failed),
            "total": len(md_files)
        },
        "successful_files": successful,
        "error_files": failed
    }

    import json
    with open("add_urdu_button_log.json", "w", encoding="utf-8") as f:
        json.dump(log_data, f, indent=2, ensure_ascii=False)

    print(f"\nDetailed log saved to: add_urdu_button_log.json")

if __name__ == "__main__":
    main()