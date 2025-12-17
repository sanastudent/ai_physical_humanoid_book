# Selected Text QA - OpenAI Implementation

## Overview

The `/query_selected` endpoint has been updated to use **OpenAI GPT-4** instead of Anthropic Claude. This endpoint allows users to ask questions about selected text from the book without performing vector search.

## Endpoint Details

**URL:** `POST /query_selected`

**Request Body:**
```json
{
  "selected_text": "string",
  "question": "string"
}
```

**Response:**
```json
{
  "answer": "string",
  "citation": "string or null",
  "error": "string or null"
}
```

## Setup Instructions

### 1. Install OpenAI Python Package

```bash
cd backend
pip install openai
```

### 2. Configure Environment Variables

Add your OpenAI API key to `.env` file:

```bash
# backend/.env
OPENAI_API_KEY=sk-your-openai-api-key-here
```

### 3. Start the Backend Server

```bash
cd backend
python -m src.main
```

The server will start on `http://localhost:8000`

## Example Request

### Using cURL

```bash
curl -X POST http://localhost:8000/query_selected \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "ROS 2 (Robot Operating System 2) is the next generation of the most widely used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is built from the ground up for production systems, offering real-time capabilities, improved security, and multi-robot support.",
    "question": "What are the key improvements of ROS 2 over ROS 1?"
  }'
```

### Using Python requests

```python
import requests

url = "http://localhost:8000/query_selected"
payload = {
    "selected_text": "ROS 2 is built for production systems with real-time capabilities.",
    "question": "What is ROS 2 designed for?"
}

response = requests.post(url, json=payload)
print(response.json())
```

### Using JavaScript (Frontend)

```javascript
const response = await fetch('http://localhost:8000/query_selected', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    selected_text: "ROS 2 is a robotics middleware framework.",
    question: "What is ROS 2?"
  })
});

const data = await response.json();
console.log(data.answer);
```

## Example Responses

### Success Response

```json
{
  "answer": "Based on the selected text, the key improvements of ROS 2 over ROS 1 include:\n\n1. **Real-time capabilities** - ROS 2 is built from the ground up for production systems with real-time support\n2. **Improved security** - Enhanced security features compared to ROS 1\n3. **Multi-robot support** - Native support for coordinating multiple robots\n\nThese improvements make ROS 2 suitable for production environments, unlike its predecessor.",
  "citation": "ROS 2 (Robot Operating System 2) is the next generation of the most widely used robotics middleware framework. Unlike its predecessor ROS 1, ROS 2 is built from the ground up for production...",
  "error": null
}
```

### Empty Selected Text

```json
{
  "answer": "Please select some text first.",
  "citation": null,
  "error": null
}
```

### Empty Question

```json
{
  "answer": "Please provide a question.",
  "citation": null,
  "error": null
}
```

### API Key Not Configured

```json
{
  "answer": "Server configuration error: OpenAI API key not found.",
  "citation": null,
  "error": "OPENAI_API_KEY not configured"
}
```

### API Error (After Retries)

```json
{
  "answer": "I encountered an error processing your question. Please try again later.",
  "citation": null,
  "error": "Failed after 3 attempts: Rate limit exceeded"
}
```

## Features

### ✅ OpenAI GPT-4 Integration
- Uses `gpt-4` model for high-quality answers
- Can be changed to `gpt-3.5-turbo` for faster/cheaper responses

### ✅ Context-Only Answers
- Uses **ONLY** the selected text as context
- Does **NOT** perform vector search in Qdrant
- Does **NOT** retrieve additional book content

### ✅ Automatic Retry Logic
- 3 total attempts (initial + 2 retries)
- Exponential backoff: 2s, 4s
- Handles rate limits and temporary failures

### ✅ Error Handling
- Empty selected text validation
- Empty question validation
- Missing API key detection
- OpenAI API error handling
- Detailed error logging

### ✅ Response Format
- Consistent JSON structure
- Answer field with generated response
- Citation field with truncated selected text
- Error field for troubleshooting

## Model Configuration

### Using GPT-4 (Current)
```python
model="gpt-4"
```
- Best quality answers
- Higher cost per request
- Slower response time

### Using GPT-3.5 Turbo (Alternative)
```python
model="gpt-3.5-turbo"
```
- Good quality answers
- Lower cost per request
- Faster response time

To switch models, edit `backend/src/main.py` line 410:
```python
response = openai.ChatCompletion.create(
    model="gpt-3.5-turbo",  # Change here
    messages=[...],
    max_tokens=1024,
    temperature=0.7
)
```

## Testing

Run the test script to verify the endpoint:

```bash
cd backend
python test_query_selected.py
```

Expected output:
```
================================================================================
TESTING /query_selected ENDPOINT
================================================================================
Target: http://localhost:8000/query_selected

Make sure the backend server is running on port 8000
================================================================================

================================================================================
TEST 1: Valid Request
================================================================================
Status Code: 200
Response: {
  "answer": "...",
  "citation": "...",
  "error": null
}
✓ TEST PASSED: Valid answer returned

[Additional tests...]
```

## Troubleshooting

### Error: "Server configuration error: OpenAI API key not found"
**Solution:** Set `OPENAI_API_KEY` in your `.env` file

### Error: "Rate limit exceeded"
**Solution:**
- Wait a few minutes and try again
- Upgrade your OpenAI API plan
- Switch to `gpt-3.5-turbo` model

### Error: "Connection refused"
**Solution:** Make sure the backend server is running on port 8000

### Empty or Invalid Responses
**Solution:**
- Check that `OPENAI_API_KEY` is valid
- Check OpenAI API status at https://status.openai.com
- Review server logs for detailed error messages

## API Cost Estimation

### GPT-4 Pricing (as of 2024)
- Input: $0.03 per 1K tokens
- Output: $0.06 per 1K tokens

**Typical Request:**
- Selected text: ~200 tokens
- Question: ~20 tokens
- System message: ~50 tokens
- **Total input:** ~270 tokens = $0.0081

- Answer: ~150 tokens
- **Total output:** ~150 tokens = $0.009

**Cost per request:** ~$0.017 (1.7 cents)

### GPT-3.5 Turbo Pricing
- Input: $0.0015 per 1K tokens
- Output: $0.002 per 1K tokens

**Cost per request:** ~$0.0007 (0.07 cents)

**Savings:** ~96% cheaper than GPT-4

## Integration with Docusaurus Frontend

The frontend should call this endpoint when users select text and ask questions:

```typescript
// frontend/my-book/src/components/ChatUI/index.tsx

async function handleSelectedTextQuery(selectedText: string, question: string) {
  const response = await fetch('http://localhost:8000/query_selected', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      selected_text: selectedText,
      question: question
    })
  });

  const data = await response.json();

  if (data.error) {
    console.error('Error:', data.error);
    return "Sorry, I encountered an error processing your question.";
  }

  return data.answer;
}
```

## Summary

✅ **Fixed:** Endpoint now uses OpenAI GPT-4 instead of Anthropic Claude
✅ **Configured:** Uses `OPENAI_API_KEY` from `.env`
✅ **Maintained:** All error handling and retry logic
✅ **Maintained:** Selected-text-only context (no vector search)
✅ **Ready:** Can be tested immediately with your OpenAI API key
