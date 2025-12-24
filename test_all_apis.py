"""Test all available APIs in priority order"""
import os
from dotenv import load_dotenv

load_dotenv()

def test_openai():
    """Test OpenAI API with gpt-4o-mini"""
    try:
        from openai import OpenAI
        api_key = os.getenv("OPENAI_KEY")

        if not api_key:
            return False, "No API key found"

        if not api_key.startswith("sk-"):
            return False, f"Invalid key format (starts with: {api_key[:10]})"

        print(f"Testing OpenAI with key: {api_key[:20]}...")
        client = OpenAI(api_key=api_key)

        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[{"role": "user", "content": "Say 'OpenAI works!'"}],
            max_tokens=10
        )

        result = response.choices[0].message.content
        return True, f"SUCCESS: {result}"
    except Exception as e:
        return False, f"ERROR: {str(e)}"

def test_grok():
    """Test Grok API"""
    try:
        import requests
        api_key = os.getenv("GROK_KEY")

        if not api_key:
            return False, "No API key found"

        if not api_key.startswith("gsk_"):
            return False, f"Invalid key format (starts with: {api_key[:10]})"

        print(f"Testing Grok with key: {api_key[:20]}...")

        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }
        data = {
            "model": "grok-beta",
            "messages": [{"role": "user", "content": "Say 'Grok works!'"}],
            "max_tokens": 10
        }

        response = requests.post(
            "https://api.x.ai/v1/chat/completions",
            headers=headers,
            json=data,
            timeout=30
        )

        if response.status_code == 200:
            result = response.json()["choices"][0]["message"]["content"]
            return True, f"SUCCESS: {result}"
        else:
            return False, f"ERROR {response.status_code}: {response.text}"
    except Exception as e:
        return False, f"ERROR: {str(e)}"

def test_gemini():
    """Test Google Gemini API"""
    try:
        import google.generativeai as genai
        api_key = os.getenv("GOOGLE_API_KEY")

        if not api_key:
            return False, "No API key found"

        if not api_key.startswith("AI"):
            return False, f"Invalid key format (starts with: {api_key[:10]})"

        print(f"Testing Gemini with key: {api_key[:20]}...")
        genai.configure(api_key=api_key)

        model = genai.GenerativeModel('gemini-1.5-flash')
        response = model.generate_content(
            "Say 'Gemini works!'",
            generation_config=genai.GenerationConfig(max_output_tokens=10)
        )

        return True, f"SUCCESS: {response.text}"
    except Exception as e:
        return False, f"ERROR: {str(e)}"

if __name__ == "__main__":
    print("=" * 60)
    print("TESTING ALL APIs IN PRIORITY ORDER")
    print("=" * 60)

    print("\n1. Testing OpenAI (gpt-4o-mini) - PRIORITY 1")
    print("-" * 60)
    success, message = test_openai()
    print(f"   Result: {message}")
    openai_works = success

    print("\n2. Testing Grok API - PRIORITY 2")
    print("-" * 60)
    success, message = test_grok()
    print(f"   Result: {message}")
    grok_works = success

    print("\n3. Testing Google Gemini - PRIORITY 3")
    print("-" * 60)
    success, message = test_gemini()
    print(f"   Result: {message}")
    gemini_works = success

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"OpenAI (Priority 1):  {'✓ WORKING' if openai_works else '✗ FAILED'}")
    print(f"Grok (Priority 2):    {'✓ WORKING' if grok_works else '✗ FAILED'}")
    print(f"Gemini (Priority 3):  {'✓ WORKING' if gemini_works else '✗ FAILED'}")

    if openai_works:
        print("\n✓ RECOMMENDED: Use OpenAI (gpt-4o-mini)")
    elif grok_works:
        print("\n✓ RECOMMENDED: Use Grok (OpenAI failed)")
    elif gemini_works:
        print("\n✓ RECOMMENDED: Use Gemini (OpenAI and Grok failed)")
    else:
        print("\n✗ WARNING: All APIs failed! Will use local mode")
