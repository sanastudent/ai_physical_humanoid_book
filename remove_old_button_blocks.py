#!/usr/bin/env python3
"""
Script to clean all Docusaurus chapter files by removing the OLD HTML button
and script block, preserving only the MDX-safe JSX button.
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
            logging.FileHandler('remove_old_button.log'),
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


def remove_old_button_blocks(content: str) -> Tuple[str, bool]:
    """
    Remove old HTML button and script blocks while preserving MDX-safe JSX button.
    Returns cleaned content and a boolean indicating if changes were made.
    """
    original_content = content

    # Pattern to match old HTML button containers and script blocks
    # This matches the old button container with class "urdu-switch-button-container"
    old_button_pattern = r'<div class="urdu-switch-button-container"[^>]*>.*?</div>\s*<script>.*?</script>'

    # Alternative pattern for old button containers without the script
    old_button_pattern_alt = r'<div class="urdu-switch-button-container"[^>]*>.*?</div>'

    # Pattern for standalone script blocks with switchToUrdu function
    script_pattern = r'<script>\s*function switchToUrdu\(path\).*?</script>'

    # Remove the old button container and script combination
    content = re.sub(old_button_pattern, '', content, flags=re.DOTALL | re.IGNORECASE)

    # Remove any remaining old button containers
    content = re.sub(old_button_pattern_alt, '', content, flags=re.DOTALL | re.IGNORECASE)

    # Remove any remaining switchToUrdu script blocks
    content = re.sub(script_pattern, '', content, flags=re.DOTALL | re.IGNORECASE)

    # Clean up any extra newlines that might result from removal
    content = re.sub(r'\n\s*\n\s*\n', '\n\n', content)  # Replace multiple blank lines with single blank line

    # Remove any leading/trailing whitespace that might have been left
    content = content.strip()

    # Add a newline at the beginning if the MDX-safe button exists but there's no newline before it
    if 'import React from \'react\';' in content:
        # Ensure there's proper spacing before the React import if it's not at the very beginning
        content = re.sub(r'([^\n])\s*(import React from \'react\';)', r'\1\n\n\2', content)

    changes_made = (original_content != content)
    return content, changes_made


def write_cleaned_file(
    original_file_path: str,
    frontmatter_data: Dict,
    cleaned_content: str
) -> bool:
    """Write the cleaned content with frontmatter back to the file."""
    try:
        # Create new frontmatter with cleaned content
        post = frontmatter.Post(cleaned_content, **frontmatter_data)

        # Write the file back by getting the string representation and writing it
        with open(original_file_path, 'w', encoding='utf-8') as f:
            content = frontmatter.dumps(post)
            f.write(content)

        return True
    except Exception as e:
        logging.error(f"Error writing cleaned file {original_file_path}: {e}")
        return False


def create_cleaning_log(cleaned: List[str], unchanged: List[str], errors: List[str]) -> None:
    """Create a log file with cleaning results."""
    log_data = {
        "summary": {
            "cleaned": len(cleaned),
            "unchanged": len(unchanged),
            "errors": len(errors),
            "total": len(cleaned) + len(unchanged) + len(errors)
        },
        "cleaned_files": cleaned,
        "unchanged_files": unchanged,
        "error_files": errors
    }

    with open('remove_old_button_log.json', 'w', encoding='utf-8') as f:
        json.dump(log_data, f, ensure_ascii=False, indent=2)


def main():
    logger = setup_logging()
    logger.info("Starting to remove old HTML button and script blocks from chapters")

    docs_dir = "frontend/my-book/docs"

    # Get all markdown files
    markdown_files = extract_markdown_files(docs_dir)
    logger.info(f"Found {len(markdown_files)} markdown files to process")

    cleaned_files = []
    unchanged_files = []
    error_files = []

    for i, file_path in enumerate(markdown_files, 1):
        logger.info(f"Processing file {i}/{len(markdown_files)}: {file_path}")

        try:
            # Extract frontmatter and content
            frontmatter_data, content = extract_frontmatter_and_content(file_path)

            # Remove old button blocks
            cleaned_content, changes_made = remove_old_button_blocks(content)

            if changes_made:
                # Write the cleaned file back
                success = write_cleaned_file(file_path, frontmatter_data, cleaned_content)

                if success:
                    cleaned_files.append(file_path)
                    logger.info(f"Successfully cleaned {file_path}")
                else:
                    error_files.append(file_path)
                    logger.error(f"Failed to clean {file_path}")
            else:
                unchanged_files.append(file_path)
                logger.info(f"No changes needed for {file_path}")

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            error_files.append(file_path)

    # Create cleaning log
    create_cleaning_log(cleaned_files, unchanged_files, error_files)

    # Show summary
    print("\n" + "="*60)
    print("REMOVE OLD BUTTON BLOCKS SUMMARY")
    print("="*60)
    print(f"Files cleaned: {len(cleaned_files)}")
    print(f"Files unchanged: {len(unchanged_files)}")
    print(f"Files with errors: {len(error_files)}")
    print(f"Total files processed: {len(markdown_files)}")
    print("="*60)

    if cleaned_files:
        print("\nCleaned files:")
        for file in cleaned_files[:10]:  # Show first 10
            print(f"  - {file}")
        if len(cleaned_files) > 10:
            print(f"  ... and {len(cleaned_files) - 10} more")

    if unchanged_files:
        print(f"\nUnchanged files: {len(unchanged_files)} total")

    if error_files:
        print("\nFiles with errors:")
        for file in error_files[:10]:  # Show first 10
            print(f"  - {file}")
        if len(error_files) > 10:
            print(f"  ... and {len(error_files) - 10} more")

    print(f"\nCleaning log saved to: remove_old_button_log.json")


if __name__ == "__main__":
    main()