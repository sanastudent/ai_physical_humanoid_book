#!/usr/bin/env python3
"""
Script to fix MDX syntax issues in the button JSX code.
"""
import os
import re
from pathlib import Path
from typing import Dict
import frontmatter  # type: ignore
import logging


def setup_logging():
    """Set up logging for the process."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)


def extract_markdown_files(docs_dir: str) -> list:
    """Extract all .md and .mdx files from the docs directory."""
    markdown_files = []
    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                markdown_files.append(os.path.join(root, file))
    return markdown_files


def extract_frontmatter_and_content(file_path: str) -> tuple:
    """Extract frontmatter and content from a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        post = frontmatter.load(f)
        frontmatter_data = post.metadata
        content = post.content
    return frontmatter_data, content


def fix_mdx_syntax(content: str) -> tuple:
    """Fix MDX syntax issues in the content."""
    original_content = content

    # Fix the JSX syntax: change onClick={{...}} to onClick={() => ...}
    # and fix other JSX attribute syntax
    content = re.sub(
        r'onClick=\{\{\(\s*\(\)\s*=>\s*\{)',
        r'onClick={\1',
        content
    )

    # Also fix any other double curly brace patterns for event handlers
    content = re.sub(
        r'onClick=\{\{(\s*[^(]?\s*\(\s*[a-zA-Z_][^)]*\)\s*=>\s*\{)',
        r'onClick={\1',
        content
    )

    # Fix the closing pattern too
    content = re.sub(
        r'\}\s*\}\s*\(\s*\)\s*\}',
        r'}\1}',
        content
    )

    # Actually, let me be more specific and fix the exact pattern that's causing issues
    # The issue is with: onClick={{() => { ... }}}
    # It should be: onClick={() => { ... }}
    content = re.sub(
        r'onClick=\{\{\(\s*\(\)\s*=>\s*\{[^\}]+\}\s*)\}\}',
        r'onClick={\1}',
        content
    )

    # More general fix - replace double curly braces around function expressions
    # But only for event handlers, not for style objects
    content = re.sub(
        r'(on\w+=)\{\{(\s*\([^)]*\)\s*=>\s*\{[^\}]+\})\}\}',
        r'\1{\2}',
        content
    )

    changes_made = (original_content != content)
    return content, changes_made


def fix_jsx_syntax_properly(content: str) -> tuple:
    """Fix JSX syntax properly for MDX compatibility."""
    original_content = content

    # The specific issue is that we have onClick={{() => { ... }}}
    # when it should be onClick={() => { ... }}
    # This pattern is consistent across all files based on the error
    content = re.sub(
        r'onClick=\{\{\(\s*\(\)\s*=>\s*\{[^}]+\}\s*\}\s*)\}\}',
        r'onClick={\1}',
        content
    )

    # Also fix the pattern where we have nested functions
    content = re.sub(
        r'onClick=\{\{(\s*\(\)\s*=>\s*\{[^}]+\}\s*\}\s*)\}\}',
        r'onClick={\1}',
        content,
        flags=re.MULTILINE | re.DOTALL
    )

    # More comprehensive fix - find the specific pattern causing the error
    # The error occurs around line 10, column 14-15 which is the onClick attribute
    pattern = r'onClick=\{\{\(\s*\(\)\s*=>\s*\{[^}]+\}\s*\)\s*\}\}'
    content = re.sub(pattern, r'onClick={\1}', content, flags=re.MULTILINE | re.DOTALL)

    # Even more specific - the issue is with the closing of the function in the onClick
    # Replace: onClick={{() => { ... }}}
    # With: onClick={() => { ... }}
    content = re.sub(
        r'onClick=\{\{(\s*\(\)\s*=>\s*\{[^}]*\}\s*\}\s*\}\s*)\}\}',
        r'onClick={\1}',
        content,
        flags=re.MULTILINE | re.DOTALL
    )

    # Let me try a simpler approach - just replace the exact problematic pattern
    # Looking at the error and the content, it's the nested }} at the end
    content = re.sub(
        r'onClick=\{\{() => \{([^}]*(?:\}[^}])*)\}\}\}',
        r'onClick={() => {\1}}',
        content
    )

    changes_made = (original_content != content)
    return content, changes_made


def fix_all_jsx_attributes(content: str) -> tuple:
    """Fix all JSX attribute syntax properly."""
    original_content = content

    # Fix onClick attributes specifically
    # Replace: onClick={{() => { ... }}}
    # With: onClick={() => { ... }}
    content = re.sub(
        r'onClick=\{\{(\(\)\s*=>\s*\{(?:[^{}]|{[^{}]*})*\})\}\}',
        r'onClick={\1}',
        content
    )

    # General pattern for fixing event handlers in JSX
    content = re.sub(
        r'(on[A-Z]\w+)=\{\{((?:[^{}]|{[^{}]*})+)\}\}',
        r'\1={\2}',
        content
    )

    changes_made = (original_content != content)
    return content, changes_made


def write_fixed_file(
    original_file_path: str,
    frontmatter_data: Dict,
    fixed_content: str
) -> bool:
    """Write the fixed content with frontmatter back to the file."""
    try:
        # Create new frontmatter with fixed content
        post = frontmatter.Post(fixed_content, **frontmatter_data)

        # Write the file back by getting the string representation and writing it
        with open(original_file_path, 'w', encoding='utf-8') as f:
            content = frontmatter.dumps(post)
            f.write(content)

        return True
    except Exception as e:
        logging.error(f"Error writing fixed file {original_file_path}: {e}")
        return False


def main():
    logger = setup_logging()
    logger.info("Starting to fix MDX syntax issues in button JSX code")

    docs_dir = "frontend/my-book/docs"

    # Get all markdown files
    markdown_files = extract_markdown_files(docs_dir)
    logger.info(f"Found {len(markdown_files)} markdown files to fix")

    fixed_files = []
    unchanged_files = []
    error_files = []

    for i, file_path in enumerate(markdown_files, 1):
        logger.info(f"Processing file {i}/{len(markdown_files)}: {file_path}")

        try:
            # Extract frontmatter and content
            frontmatter_data, content = extract_frontmatter_and_content(file_path)

            # Fix JSX syntax issues
            fixed_content, changes_made = fix_all_jsx_attributes(content)

            if changes_made:
                # Write the fixed file back
                success = write_fixed_file(file_path, frontmatter_data, fixed_content)

                if success:
                    fixed_files.append(file_path)
                    logger.info(f"Successfully fixed {file_path}")
                else:
                    error_files.append(file_path)
                    logger.error(f"Failed to fix {file_path}")
            else:
                unchanged_files.append(file_path)
                logger.info(f"No changes needed for {file_path}")

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            error_files.append(file_path)

    # Show summary
    print("\n" + "="*50)
    print("MDX SYNTAX FIX SUMMARY")
    print("="*50)
    print(f"Files fixed: {len(fixed_files)}")
    print(f"Files unchanged: {len(unchanged_files)}")
    print(f"Files with errors: {len(error_files)}")
    print(f"Total files processed: {len(markdown_files)}")
    print("="*50)

    if fixed_files:
        print(f"\nFixed files: {len(fixed_files)}")

    if error_files:
        print(f"\nFiles with errors: {len(error_files)}")
        for file in error_files:
            print(f"  - {file}")

    print(f"\nCompleted MDX syntax fixes.")


if __name__ == "__main__":
    main()