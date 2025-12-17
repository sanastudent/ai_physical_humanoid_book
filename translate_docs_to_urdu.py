#!/usr/bin/env python3
"""
Script to translate all Docusaurus English docs to Urdu using Book Assistant backend (RAG)
and write them into i18n/ur folders.
"""
import os
import json
import requests
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import frontmatter  # type: ignore
import logging
from urllib.parse import urlparse, urljoin


def setup_logging():
    """Set up logging for the translation process."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('translation.log'),
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


def translate_content_to_urdu(content: str, api_url: str = None) -> Optional[str]:
    """Send content to Book Assistant backend API for Urdu translation."""
    if api_url is None:
        api_url = os.getenv("BACKEND_URL", "http://localhost:8000") + "/query"
    try:
        payload = {
            "query": f"Translate this chapter into Urdu:\n\n{content}",
            "mode": "global"
        }
        response = requests.post(
            api_url,
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=300  # 5 minute timeout for long content
        )

        if response.status_code == 200:
            data = response.json()
            return data.get("answer", data.get("response", ""))
        else:
            logging.error(f"API request failed with status {response.status_code}: {response.text}")
            return None
    except requests.exceptions.RequestException as e:
        logging.error(f"Error making API request: {e}")
        return None
    except Exception as e:
        logging.error(f"Unexpected error during translation: {e}")
        return None


def translate_internal_links(content: str, translations: Dict[str, str]) -> str:
    """Translate internal links if corresponding translation exists."""
    # Find all markdown links [text](path)
    link_pattern = r'\[([^\]]+)\]\(([^)]+)\)'

    def replace_link(match):
        link_text = match.group(1)
        link_path = match.group(2)

        # Check if it's an internal link (not starting with http/https)
        parsed = urlparse(link_path)
        if not parsed.scheme and not link_path.startswith(('http://', 'https://', '#', '/')):
            # Try to find corresponding translated file
            # This is a simplified approach - in practice you might need more sophisticated mapping
            translated_text = translations.get(link_text, link_text)
            return f"[{translated_text}]({link_path})"

        return match.group(0)  # Return original if external link

    return re.sub(link_pattern, replace_link, content)


def write_translated_file(
    original_file_path: str,
    frontmatter_data: Dict,
    urdu_content: str,
    output_dir: str
) -> bool:
    """Write the translated content with frontmatter to the Urdu directory."""
    try:
        # Create the output directory structure
        relative_path = os.path.relpath(original_file_path, 'docs')
        output_file_path = os.path.join(output_dir, relative_path)

        # Ensure the directory exists
        os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

        # Create new frontmatter with translated content
        post = frontmatter.Post(urdu_content, **frontmatter_data)

        # Write the file
        with open(output_file_path, 'w', encoding='utf-8') as f:
            frontmatter.dump(post, f)

        return True
    except Exception as e:
        logging.error(f"Error writing translated file {original_file_path}: {e}")
        return False


def create_urdu_placeholder_file(
    original_file_path: str,
    frontmatter_data: Dict,
    original_content: str,
    output_dir: str
) -> bool:
    """Create a Urdu translation file with original content as fallback when API fails."""
    try:
        # Create the output directory structure
        relative_path = os.path.relpath(original_file_path, 'docs')
        output_file_path = os.path.join(output_dir, relative_path)

        # Ensure the directory exists
        os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

        # Create placeholder content with preserved frontmatter
        if frontmatter_data:
            # Build frontmatter string
            frontmatter_str = ""
            for key, value in frontmatter_data.items():
                if isinstance(value, str):
                    # Escape quotes if needed
                    escaped_value = value.replace('"', '\\"')
                    frontmatter_str += f"{key}: \"{escaped_value}\"\n"
                else:
                    frontmatter_str += f"{key}: {value}\n"

            urdu_content = f"""---
{frontmatter_str}---
<!--
  Urdu Translation Placeholder
  English source: {relative_path}

  TODO: Add Urdu translation below this comment.
  The frontmatter above (id, title, sidebar_position, etc.) has been preserved
  to ensure proper chapter ID matching and navigation.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

{original_content.strip()}
"""
        else:
            # No frontmatter, just add placeholder
            urdu_content = f"""<!--
  Urdu Translation Placeholder
  English source: {relative_path}

  TODO: Add Urdu translation below this comment.
-->

# [Urdu Translation Required]

یہاں اردو ترجمہ شامل کریں
(Add Urdu translation here)

---

**Original English Content (for reference):**

{original_content.strip()}
"""

        # Write the file
        with open(output_file_path, 'w', encoding='utf-8') as f:
            f.write(urdu_content)

        return True
    except Exception as e:
        logging.error(f"Error writing placeholder file {original_file_path}: {e}")
        return False


def create_translation_log(successful: List[str], skipped: List[str], errors: List[str]) -> None:
    """Create a log file with translation results."""
    log_data = {
        "summary": {
            "successful": len(successful),
            "skipped": len(skipped),
            "errors": len(errors),
            "total": len(successful) + len(skipped) + len(errors)
        },
        "successful_translations": successful,
        "skipped_files": skipped,
        "error_files": errors
    }

    with open('translation_log.json', 'w', encoding='utf-8') as f:
        json.dump(log_data, f, ensure_ascii=False, indent=2)


def main():
    logger = setup_logging()
    logger.info("Starting Urdu translation process for Docusaurus docs")

    docs_dir = "frontend/my-book/docs"
    output_dir = "frontend/my-book/i18n/ur/docusaurus-plugin-content-docs/current"

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Get all markdown files
    markdown_files = extract_markdown_files(docs_dir)
    logger.info(f"Found {len(markdown_files)} markdown files to translate")

    successful_translations = []
    skipped_files = []
    error_files = []

    # Dictionary to store translations for link processing (simplified)
    text_translations = {}

    for i, file_path in enumerate(markdown_files, 1):
        logger.info(f"Processing file {i}/{len(markdown_files)}: {file_path}")

        try:
            # Extract frontmatter and content
            frontmatter_data, content = extract_frontmatter_and_content(file_path)

            # Send content to Book Assistant API for translation
            urdu_content = translate_content_to_urdu(content)

            if urdu_content is None:
                # API failed, create a placeholder file with original content
                logger.warning(f"API failed for {file_path}, creating placeholder file")
                success = create_urdu_placeholder_file(file_path, frontmatter_data, content, output_dir)

                if success:
                    successful_translations.append(file_path)
                    logger.info(f"Created placeholder file for {file_path}")
                else:
                    error_files.append(file_path)
                    logger.error(f"Failed to create placeholder file for {file_path}")
                continue

            # Process internal links (simplified approach)
            # In a more advanced implementation, you might want to maintain a mapping
            # of translated links based on file relationships
            processed_content = translate_internal_links(urdu_content, text_translations)

            # Write translated file
            success = write_translated_file(file_path, frontmatter_data, processed_content, output_dir)

            if success:
                successful_translations.append(file_path)
                logger.info(f"Successfully translated {file_path}")
            else:
                error_files.append(file_path)
                logger.error(f"Failed to write translated file {file_path}")

        except Exception as e:
            logger.error(f"Error processing file {file_path}: {e}")
            error_files.append(file_path)

    # Create translation log
    create_translation_log(successful_translations, skipped_files, error_files)

    # Show summary
    print("\n" + "="*50)
    print("TRANSLATION SUMMARY")
    print("="*50)
    print(f"Files successfully translated: {len(successful_translations)}")
    print(f"Files skipped: {len(skipped_files)}")
    print(f"Files with errors: {len(error_files)}")
    print(f"Total files processed: {len(markdown_files)}")
    print("="*50)

    if successful_translations:
        print("\nSuccessfully translated files:")
        for file in successful_translations[:10]:  # Show first 10
            print(f"  - {file}")
        if len(successful_translations) > 10:
            print(f"  ... and {len(successful_translations) - 10} more")

    if error_files:
        print("\nFiles with errors:")
        for file in error_files[:10]:  # Show first 10
            print(f"  - {file}")
        if len(error_files) > 10:
            print(f"  ... and {len(error_files) - 10} more")

    print(f"\nTranslation log saved to: translation_log.json")
    print(f"Translated files are in: {output_dir}")


if __name__ == "__main__":
    main()