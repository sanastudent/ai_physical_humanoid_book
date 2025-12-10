# Docusaurus i18n Quick Reference

## Quick Start Commands

### Development
```bash
# English version
npm start

# Urdu version
npm run start:ur
```

### Build
```bash
# Build both locales
npm run build

# Serve production build
npm run serve          # English
npm run serve:ur       # Urdu
```

### Translation Management
```bash
# Generate new translation scaffolding
npx docusaurus write-translations --locale ur

# Clear cache
npm run clear
```

## File Locations

### Configuration
- Main config: `docusaurus.config.ts`
- Package scripts: `package.json`
- Sidebar: `sidebars.ts`

### English Content
```
docs/
├── chapters/           # Book chapters
├── tutorial-basics/    # Basic tutorials
├── tutorial-extras/    # Extra tutorials
├── intro.md
├── introduction.md
├── summary.md
├── glossary.md
└── references.md
```

### Urdu Translations
```
i18n/ur/
├── code.json                              # UI strings
├── docusaurus-theme-classic/
│   ├── navbar.json                        # Navigation
│   └── footer.json                        # Footer
└── docusaurus-plugin-content-docs/current/
    ├── chapters/                          # Translated chapters
    ├── tutorial-basics/
    ├── tutorial-extras/
    └── [all .md files mirror English structure]
```

## Translation Workflow

### 1. Find the File
- English: `docs/[path]/file.md`
- Urdu: `i18n/ur/docusaurus-plugin-content-docs/current/[path]/file.md`

### 2. Translate Content
- **Keep frontmatter unchanged** (between `---` markers)
- Replace placeholder with Urdu content
- Reference English content at bottom of file

### 3. Test
```bash
npm run start:ur
```

### 4. Verify Build
```bash
npm run build
```

## Important Rules

### DO ✅
- Preserve all frontmatter fields
- Keep file names identical to English
- Maintain folder structure
- Use UTF-8 encoding
- Test both locales after changes

### DON'T ❌
- Change file names
- Modify `id:`, `slug:`, or `sidebar_position:` fields
- Delete frontmatter
- Use relative image paths (use absolute paths)
- Skip build testing

## Common Tasks

### Add New Document

**English:**
1. Create `docs/new-file.md`
2. Add frontmatter
3. Update `sidebars.ts` if needed

**Urdu:**
1. Copy structure to `i18n/ur/.../new-file.md`
2. Preserve frontmatter
3. Translate content

### Update Existing Document

**English:**
1. Edit `docs/[path]/file.md`

**Urdu:**
1. Edit `i18n/ur/.../[path]/file.md`
2. Keep frontmatter in sync

### Add UI Translation

Edit `i18n/ur/code.json`:
```json
{
  "theme.some.key": {
    "message": "اردو ترجمہ",
    "description": "Description in English"
  }
}
```

### Add Locale Switcher

In `docusaurus.config.ts`:
```typescript
navbar: {
  items: [
    {
      type: 'localeDropdown',
      position: 'right',
    },
  ],
}
```

## Troubleshooting

### Build Fails
```bash
# Clear cache
npm run clear

# Rebuild
npm run build
```

### Missing Translations
```bash
# Regenerate scaffolding
npx docusaurus write-translations --locale ur
```

### Broken Links
- Check frontmatter matches between locales
- Verify file paths are identical
- Ensure no typos in IDs or slugs

### Image Issues
- Use absolute paths: `/img/image.png`
- Or reference from static folder
- Avoid relative paths in translation files

## File Structure Validation

### Check Structure Match
```bash
# English files
find docs -name "*.md" | wc -l

# Urdu files
find i18n/ur/docusaurus-plugin-content-docs/current -name "*.md" | wc -l
```

Should show same count (28 files currently).

### Verify IDs
```bash
python validate_ids.py
```

## URLs

### English
- Homepage: `http://localhost:3000/book/`
- Docs: `http://localhost:3000/book/docs/intro`

### Urdu
- Homepage: `http://localhost:3000/book/ur/`
- Docs: `http://localhost:3000/book/ur/docs/intro`

## Resources

- [Docusaurus i18n Guide](https://docusaurus.io/docs/i18n/introduction)
- Summary: `URDU_I18N_SUMMARY.md`
- Validation: `id_validation_report.json`
- Translation list: `urdu_translation_summary.json`

---

**Last Updated:** 2025-12-09
**Locales:** en (English), ur (Urdu)
**Total Docs:** 28 files per locale
