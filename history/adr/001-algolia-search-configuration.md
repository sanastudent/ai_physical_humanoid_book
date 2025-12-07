# ADR 001: Algolia Search Configuration for Docusaurus

## Status
Accepted

## Date
2025-12-06

## Context
During the implementation of the AI-Driven Book + RAG Chatbot project, we encountered an issue with the Docusaurus build process. The default Docusaurus configuration included Algolia search integration, which requires valid API credentials (appId and apiKey). During the build process, if these credentials are not provided via environment variables, the build fails with a validation error.

The project requires a successful Docusaurus build for deployment to GitHub Pages as part of the frontend for the AI book content. The immediate need was to verify that the documentation site could build successfully before proceeding with full deployment.

## Decision
We have decided to temporarily disable Algolia search integration in the Docusaurus configuration by commenting out the algolia configuration block in `docusaurus.config.ts`. This allows the build process to complete successfully without requiring search functionality during the initial deployment phase.

The specific change involves commenting out the algolia configuration section:

```typescript
// algolia: {
//   // Optional: configure Algolia search
//   appId: process.env.ALGOLIA_APP_ID,
//   apiKey: process.env.ALGOLIA_API_KEY,
//   indexName: 'book',
//   contextualSearch: true,
// },
```

## Alternatives Considered

### Alternative 1: Provide Temporary/Placeholder Credentials
- **Pros**: Maintains search functionality during development
- **Cons**: Introduces security risks with placeholder credentials; still requires valid Algolia account setup

### Alternative 2: Use Docusaurus Built-in Search (Local Search Plugin)
- **Pros**: No external dependencies; works out of the box
- **Cons**: Less powerful than Algolia; may not meet long-term search requirements

### Alternative 3: Conditional Configuration Based on Environment
- **Pros**: Maintains Algolia in production while allowing builds in development
- **Cons**: More complex configuration logic; potential for inconsistent behavior across environments

## Consequences

### Positive
- Docusaurus build process completes successfully
- Enables immediate deployment to GitHub Pages
- Allows focus on core RAG functionality without search blocking the release
- Maintains flexibility to add search later

### Negative
- Production site will lack search functionality initially
- May impact user experience for large documentation sets
- Requires additional work to implement search later
- May need to re-evaluate search requirements and alternatives

### Neutral
- The decision is reversible without breaking changes to other components
- Search functionality can be added incrementally without affecting core book/RAG features

## References
- Docusaurus documentation on search configuration
- Project requirement for GitHub Pages deployment (FR-006)
- Frontend architecture decisions in plan.md