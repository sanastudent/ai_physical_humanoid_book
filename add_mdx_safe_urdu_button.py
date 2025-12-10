#!/usr/bin/env python3
"""
Script to add a 'Switch to Urdu' button at the start of each chapter in the Docusaurus book
using MDX-safe JSX syntax, ensuring no compilation errors.
"""
import os
import json
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import frontmatter  # type: ignore
import logging


def setup_logging():
    """Set up logging for the process."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('switch_to_urdu_log.json', mode='w'),  # Using 'w' to overwrite previous log
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)


def extract_markdown_files(docs_dir: str) -> List[str]:
    """Extract all .md and .mdx files from the docs directory."""
    markdown_files = []
    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                markdown_files.append(os.path.join(root, file))
    return markdown_files


def extract_frontmatter_and_content(file_path: str) -> Tuple[Dict, str]:
    """Extract frontmatter and content from a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        post = frontmatter.load(f)
        frontmatter_data = post.metadata
        content = post.content
    return frontmatter_data, content


def determine_urdu_path(original_path: str) -> str:
    """Determine the corresponding Urdu file path."""
    # Convert the original path to the Urdu translation path
    # Example: docs/intro.md -> i18n/ur/docusaurus-plugin-content-docs/current/intro.md
    relative_path = os.path.relpath(original_path, 'frontend/my-book/docs')
    urdu_path = f"frontend/my-book/i18n/ur/docusaurus-plugin-content-docs/current/{relative_path}"
    return urdu_path


def create_mdx_safe_button(urdu_path: str) -> str:
    """Create the MDX-safe JSX button with proper syntax."""
    button_jsx = f'''import React from 'react';

<div style={{{{marginBottom: '20px', padding: '10px', backgroundColor: '#f0f8ff', border: '1px solid #b3d9ff', borderRadius: '4px'}}}}>
  <button
    style={{{{backgroundColor: 'green', color: 'white', padding: '5px 10px', borderRadius: '5px', cursor: 'pointer'}}}}
    onClick={{{{() => {{
      const path = '{urdu_path}';
      fetch(path, {{method: 'HEAD'}}).then(res => {{
        if (res.ok) window.location.href = path;
        else alert('Urdu translation is not available yet.');
      }}).catch(error => {{
        alert('Urdu translation is not available yet.');
      }});
    }}}}}}
  >
    Urdu mein dekhein
  </button>
</div>

'''
    return button_jsx


def add_mdx_button_to_content(
    frontmatter_data: Dict,
    content: str,
    urdu_path: str
) -> str:
    """Add the MDX-safe switch to Urdu button to the content."""
    # Create the MDX-safe button
    button_jsx = create_mdx_safe_button(urdu_path)

    # Combine button with the original content
    updated_content = button_jsx + content

    return updated_content


def write_updated_file(
    original_file_path: str,
    frontmatter_data: Dict,
    updated_content: str
) -> bool:
    """Write the updated content with frontmatter back to the file."""
    try:
        # Create new frontmatter with updated content
        post = frontmatter.Post(updated_content, **frontmatter_data)

        # Write the file back by getting the string representation and writing it
        with open(original_file_path, 'w', encoding='utf-8') as f:
            content = frontmatter.dumps(post)
            f.write(content)

        return True
    except Exception as e:
        logging.error(f"Error writing updated file {original_file_path}: {e}")
        return False


def create_update_log(successful: List[str], skipped: List[str], errors: List[str]) -> None:
    """Create a log file with update results."""
    log_data = {
        "summary": {
            "successful": len(successful),
            "skipped": len(skipped),
            "errors": len(errors),
            "total": len(successful) + len(skipped) + len(errors)
        },
        "successful_updates": successful,
        "skipped_files": skipped,
        "error_files": errors
    }

    # Write to a separate log file to avoid conflicts with logging setup
    with open('switch_to_urdu_log.json', 'w', encoding='utf-8') as f:
        json.dump(log_data, f, ensure_ascii=False, indent=2)


def main():
    logger = setup_logging()
    logger.info("Starting to add MDX-safe 'Switch to Urdu' buttons to chapters")

    docs_dir = "frontend/my-book/docs"

    # Get all markdown files
    markdown_files = extract_markdown_files(docs_dir)
    logger.info(f"Found {len(markdown_files)} markdown files to update")

    successful_updates = []
    skipped_files = []
    error_files = []

    for i, file_path in enumerate(markdown_files, 1):
        logger.info(f"Processing file {i}/{len(markdown_files)}: {file_path}")

        try:
            # Extract frontmatter and content
            frontmatter_data, content = extract_frontmatter_and_content(file_path)

            # Determine the corresponding Urdu file path
            urdu_path = determine_urdu_path(file_path)

            # Add the MDX-safe switch button to the content
            updated_content = add_mdx_button_to_content(frontmatter_data, content, urdu_path)

            # Write the updated file back
            success = write_updated_file(file_path, frontmatter_data, updated_content)

            if success:
                successful_updates.append(file_path)
                logger.info(f"Successfully updated {file_path}")
            else:
                error_files.append(file_path)
                logger.error(f"Failed to update {file_path}")

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            error_files.append(file_path)

    # Create update log
    create_update_log(successful_updates, skipped_files, error_files)

    # Show summary
    print("\n" + "="*60)
    print("MDX-SAFE SWITCH TO URDU BUTTONS SUMMARY")
    print("="*60)
    print(f"Files successfully updated: {len(successful_updates)}")
    print(f"Files skipped: {len(skipped_files)}")
    print(f"Files with errors: {len(error_files)}")
    print(f"Total files processed: {len(markdown_files)}")
    print("="*60)

    if successful_updates:
        print("\nSuccessfully updated files:")
        for file in successful_updates[:10]:  # Show first 10
            print(f"  - {file}")
        if len(successful_updates) > 10:
            print(f"  ... and {len(successful_updates) - 10} more")

    if error_files:
        print("\nFiles with errors:")
        for file in error_files[:10]:  # Show first 10
            print(f"  - {file}")
        if len(error_files) > 10:
            print(f"  ... and {len(error_files) - 10} more")

    print(f"\nUpdate log saved to: switch_to_urdu_log.json")


if __name__ == "__main__":
    main()