#!/usr/bin/env python3
"""
Script to verify that all docs compile cleanly after removing old HTML button blocks
and ensure no invalid HTML/JS remains.
"""
import os
import re
from pathlib import Path
from typing import List, Dict
import subprocess
import json


def search_forbidden_patterns(docs_dir: str) -> Dict[str, List[str]]:
    """Search all files in /docs for forbidden patterns."""
    forbidden_patterns = {
        'script_tags': r'<script[^>]*>.*?</script>',
        'onclick_attributes': r'onclick\s*=',  # Matches onclick= (HTML attribute)
        'html_button_style': r'<button\s+[^>]*style\s*=',  # Matches <button with style attribute
        'urdu_switch_container': r'urdu-switch-button-container'
    }

    issues_found = {}

    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(('.md', '.mdx')):
                file_path = os.path.join(root, file)

                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Check each pattern but exclude MDX-safe JSX patterns
                for pattern_name, pattern in forbidden_patterns.items():
                    matches = re.findall(pattern, content, re.IGNORECASE | re.DOTALL)

                    # Filter out matches that are part of MDX-safe JSX syntax
                    if matches:
                        # For onclick, exclude cases that are part of MDX/JSX syntax
                        if pattern_name == 'onclick_attributes':
                            # Find actual problematic onclick attributes (HTML, not JSX)
                            html_onclick_pattern = r'onclick\s*=\s*["\'][^"\']*["\']'
                            html_matches = re.findall(html_onclick_pattern, content, re.IGNORECASE)
                            if html_matches:
                                if pattern_name not in issues_found:
                                    issues_found[pattern_name] = []
                                issues_found[pattern_name].append(file_path)

                        elif pattern_name == 'html_button_style':
                            # Check if the button pattern is part of HTML (not JSX)
                            # Look for <button with HTML-style attributes (not JSX-style)
                            problematic_button_pattern = r'<button\s+[^>]*style\s*=\s*["\'][^"\']*["\'][^>]*>'
                            html_button_matches = re.findall(problematic_button_pattern, content, re.IGNORECASE)
                            if html_button_matches:
                                if pattern_name not in issues_found:
                                    issues_found[pattern_name] = []
                                issues_found[pattern_name].append(file_path)

                        else:
                            # For script_tags and urdu_switch_container, add all matches
                            if pattern_name not in issues_found:
                                issues_found[pattern_name] = []
                            issues_found[pattern_name].append(file_path)

    return issues_found


def run_build() -> Dict[str, str]:
    """Run npm run build and capture output."""
    try:
        # Change to the frontend/my-book directory
        original_dir = os.getcwd()
        os.chdir('frontend/my-book')

        # Run the build command
        result = subprocess.run(
            ['npm', 'run', 'build'],
            capture_output=True,
            text=True,
            timeout=300  # 5 minute timeout
        )

        # Change back to original directory
        os.chdir(original_dir)

        return {
            'return_code': str(result.returncode),
            'stdout': result.stdout,
            'stderr': result.stderr,
            'success': result.returncode == 0
        }
    except subprocess.TimeoutExpired:
        os.chdir(original_dir)
        return {
            'return_code': 'timeout',
            'stdout': '',
            'stderr': 'Build timed out after 5 minutes',
            'success': False
        }
    except FileNotFoundError:
        os.chdir(original_dir)
        return {
            'return_code': 'error',
            'stdout': '',
            'stderr': 'npm command not found. Please ensure Node.js and npm are installed.',
            'success': False
        }
    except Exception as e:
        os.chdir(original_dir)
        return {
            'return_code': 'error',
            'stdout': '',
            'stderr': f'Error running build: {str(e)}',
            'success': False
        }


def generate_validation_report(issues_found: Dict[str, List[str]], build_result: Dict[str, str]) -> str:
    """Generate a full validation report."""
    report = []
    report.append("=" * 60)
    report.append("MDX CLEAN BUILD VERIFICATION REPORT")
    report.append("=" * 60)

    # Report any issues found
    if issues_found:
        report.append("\n[X] ISSUES FOUND:")
        report.append("-" * 30)
        for pattern_name, files in issues_found.items():
            pattern_desc = {
                'script_tags': '<script> tags',
                'onclick_attributes': 'onclick= attributes',
                'html_button_style': '<button style= patterns',
                'urdu_switch_container': 'urdu-switch-button-container classes'
            }
            report.append(f"\n{pattern_desc.get(pattern_name, pattern_name)} found in:")
            for file in set(files):  # Use set to avoid duplicates
                report.append(f"  - {file}")
    else:
        report.append("\n[OK] NO FORBIDDEN PATTERNS FOUND")
        report.append("-" * 30)
        report.append("All files are clean of script tags, onclick attributes,")
        report.append("HTML button patterns, and old container classes.")

    # Report build results
    report.append(f"\nBUILD PROCESS RESULT:")
    report.append("-" * 30)
    if build_result['success']:
        report.append("[OK] BUILD SUCCESSFUL")
        report.append("No MDX compilation errors detected.")
    else:
        report.append("[ERROR] BUILD FAILED")
        report.append(f"Return code: {build_result['return_code']}")
        if build_result['stderr']:
            report.append(f"Error output:\n{build_result['stderr']}")
        if build_result['stdout']:
            report.append(f"Standard output:\n{build_result['stdout']}")

    # Safety check
    report.append(f"\nDEPLOYMENT SAFETY CHECK:")
    report.append("-" * 30)
    if not issues_found and build_result['success']:
        report.append("[OK] SAFE TO DEPLOY")
        report.append("All checks passed - no forbidden patterns and build successful.")
    elif not issues_found and not build_result['success']:
        report.append("[WARN] NOT SAFE TO DEPLOY - BUILD FAILED")
        report.append("Forbidden patterns removed but build failed.")
    elif issues_found and build_result['success']:
        report.append("[WARN] NOT SAFE TO DEPLOY - ISSUES FOUND")
        report.append("Build successful but forbidden patterns still present.")
    else:
        report.append("[ERROR] NOT SAFE TO DEPLOY - ISSUES AND BUILD FAILED")
        report.append("Both forbidden patterns and build failures detected.")

    report.append("=" * 60)

    return "\n".join(report)


def main():
    print("Starting MDX clean build verification...")

    # Step 1 & 2: Search for forbidden patterns
    print("1. Searching for forbidden patterns...")
    docs_dir = "frontend/my-book/docs"
    issues_found = search_forbidden_patterns(docs_dir)

    if issues_found:
        print(f"   [X] Found issues in {sum(len(files) for files in issues_found.values())} files")
    else:
        print("   [OK] No forbidden patterns found")

    # Step 3 & 4: Run build and capture errors
    print("2. Running build process...")
    build_result = run_build()

    if build_result['success']:
        print("   [OK] Build completed successfully")
    else:
        print(f"   [ERROR] Build failed with return code: {build_result['return_code']}")

    # Step 5: Generate validation report
    print("3. Generating validation report...")
    report = generate_validation_report(issues_found, build_result)

    # Print the report
    print(report)

    # Step 6: Provide confirmation and safety check
    print("\nSUMMARY:")
    print("-" * 20)
    if not issues_found and build_result['success']:
        print("[OK] CONFIRMATION: All docs are clean and build successfully!")
        print("[OK] SAFE TO DEPLOY: Yes")
    elif issues_found and not build_result['success']:
        print("[ERROR] CONFIRMATION: Issues found AND build failed!")
        print("[ERROR] SAFE TO DEPLOY: No")
    elif issues_found:
        print("[WARN] CONFIRMATION: Issues found but build succeeded!")
        print("[ERROR] SAFE TO DEPLOY: No (fix issues first)")
    else:
        print("[WARN] CONFIRMATION: No issues found but build failed!")
        print("[ERROR] SAFE TO DEPLOY: No (fix build errors first)")

    # Save report to file
    with open('mdx_verification_report.txt', 'w', encoding='utf-8') as f:
        f.write(report)

    print("\nFull report saved to: mdx_verification_report.txt")


if __name__ == "__main__":
    main()