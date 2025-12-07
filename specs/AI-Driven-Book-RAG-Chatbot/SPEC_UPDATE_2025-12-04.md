# Specification Update Log

**Date**: 2025-12-04
**Updated By**: Claude Code
**Version**: 1.1.0
**Previous Version**: 1.0.0

## Summary of Changes

Replaced ChatKit SDK with OpenAI Agents SDK for RAG chatbot implementation.

## Detailed Changes

### Section 2: RAG Chatbot Specification

**Removed**:
- ChatKit (SDK: @openai/chatkit@latest)
- ChatKit-specific streaming implementation

**Added**:
- Custom React-based ChatUI component
- OpenAI Agents SDK (Python) integration
- Configurable streaming/non-streaming responses

### Section 3: Backend Specification (FastAPI)

**Added**:
- AI Integration subsection:
  - OpenAI Agents SDK (Python) for RAG responses
  - Model: gpt-4o-mini or compatible
  - Qdrant context retrieval before agent call
  - Agent receives context chunks and user query

**Updated Endpoints**:
- `/query` → now uses OpenAI Agents SDK
- `/select` → now uses OpenAI Agents SDK

### Section 4: LLM Integration

**Restructured**:
- Split into two categories:
  1. **Book Generation**: Claude Code Router → Claude (unchanged)
  2. **RAG Chatbot**: OpenAI Agents SDK (new)

**Updated Configuration**:
```bash
# Old
export CLAUDE_API_KEY="sk-XXXX-XXXX-XXXX"

# New
export ANTHROPIC_API_KEY="sk-XXXX-XXXX-XXXX"
export OPENAI_API_KEY="sk-XXXX-XXXX-XXXX"
```

### Functional Requirements Updates

**FR-011** (Changed):
- Old: "The chatbot MUST integrate ChatKit for the frontend UI."
- New: "The chatbot MUST use a custom React-based ChatUI component for the frontend."

**FR-013** (Changed):
- Old: "The chatbot MUST stream answers to the frontend."
- New: "The chatbot MUST support streaming or non-streaming answers to the frontend."

**FR-016** (Clarified):
- Old: "The system MUST use Claude (paid API) for LLM integration via Claude Code Router."
- New: "The system MUST use Claude (paid API) for book generation via Claude Code Router."

**FR-020** (Added):
- New: "The RAG chatbot MUST use OpenAI Agents SDK (Python) for generating responses with context from Qdrant."

### User Story 2 Acceptance Scenario Update

**Changed**:
- Old: "...streams an answer..."
- New: "...shows an answer (streaming or non-streaming)..."

## Architecture Impact

### Frontend
- No dependency on @openai/chatkit
- Custom ChatUI.jsx uses fetch API for backend communication
- Simpler implementation with standard React patterns

### Backend
- New dependency: OpenAI Agents SDK (Python)
- Integration with Qdrant remains unchanged
- RAG flow: Retrieve context from Qdrant → Pass to OpenAI Agent → Return response

### Dependencies

**Removed**:
- @openai/chatkit@latest (frontend)

**Added**:
- OpenAI Agents SDK (backend)
- openai>=1.10.0 (already in requirements.txt)

## Backward Compatibility

This is a **breaking change** for implementations that relied on ChatKit:

### Migration Required:
1. Remove ChatKit imports from frontend
2. Update backend to use OpenAI Agents SDK
3. Update environment variables (ANTHROPIC_API_KEY, OPENAI_API_KEY)
4. Test custom ChatUI component

### No Migration Needed:
- Qdrant schema (unchanged)
- Embedding pipeline (unchanged)
- Book structure and content (unchanged)
- Deployment targets (unchanged)

## Benefits of This Change

1. **Simpler Architecture**: No external chat SDK dependency
2. **More Control**: Custom UI allows full customization
3. **Better Integration**: OpenAI Agents SDK purpose-built for this use case
4. **Flexibility**: Easy to switch models (gpt-4o-mini, gpt-4, etc.)
5. **Cost Efficiency**: Direct control over API usage

## Implementation Notes

### OpenAI Agents SDK Usage

```python
from openai import OpenAI

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Retrieve context from Qdrant
context_chunks = qdrant_manager.search(query_vector, limit=5)

# Format context for agent
context_str = "\n\n".join([chunk.text for chunk in context_chunks])

# Call OpenAI agent with context
response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": f"Context:\n{context_str}"},
        {"role": "user", "content": query}
    ]
)
```

### Frontend Fetch Pattern

```javascript
const response = await fetch(`${backendUrl}/query`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ query, mode: 'global' })
});

const data = await response.json();
// Display answer and citations
```

## Testing Considerations

1. Verify OpenAI API key is configured
2. Test both /query and /select endpoints
3. Validate citation extraction
4. Check error handling for API failures
5. Performance test with various query complexities

## Documentation Updates Needed

- [x] spec.md - Updated
- [ ] README.md - Needs update to remove ChatKit mentions
- [ ] IMPLEMENTATION_GUIDE.md - Needs update for OpenAI SDK setup
- [ ] Backend requirements.txt - May need OpenAI Agents SDK package

## Rollout Plan

1. Update specification (complete)
2. Update backend implementation
3. Update frontend ChatUI
4. Update documentation
5. Test integration
6. Deploy to staging
7. Deploy to production

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-03 | Initial specification with ChatKit |
| 1.1.0 | 2025-12-04 | Replaced ChatKit with OpenAI Agents SDK |

## Approval

- **Specification Updated**: Yes
- **Constitution Updated**: Not required (no ChatKit-specific principles)
- **Tasks Updated**: Required (pending)
- **Plan Updated**: Required (pending)

---

**Note**: This change improves the architecture by removing an unnecessary dependency and using a more appropriate tool (OpenAI Agents SDK) for RAG implementation.
