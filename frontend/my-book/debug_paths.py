from pathlib import Path
import os

docs_dir = Path('docs')
urdu_dir = Path('i18n/ur/docusaurus-plugin-content-docs/current')

print('English docs:')
en_paths = []
for md_file in docs_dir.rglob('*.md'):
    if 'node_modules' not in str(md_file):
        rel_path = os.path.relpath(str(md_file), 'docs').replace('\\', '/')
        en_paths.append(rel_path)
        if len(en_paths) <= 5:
            print(f'  {rel_path}')

print(f'\nTotal: {len(en_paths)}')

print('\nUrdu docs:')
ur_paths = []
for md_file in urdu_dir.rglob('*.md'):
    rel_path = os.path.relpath(str(md_file), 'i18n/ur/docusaurus-plugin-content-docs/current').replace('\\', '/')
    ur_paths.append(rel_path)
    if len(ur_paths) <= 5:
        print(f'  {rel_path}')

print(f'\nTotal: {len(ur_paths)}')

# Check if any match
print('\nChecking matches:')
matches = set(en_paths).intersection(set(ur_paths))
print(f'Matches: {len(matches)}')
if matches:
    for m in list(matches)[:5]:
        print(f'  {m}')
