"""Test which Gemini model works"""
import os
from dotenv import load_dotenv
import google.generativeai as genai

load_dotenv()

api_key = os.getenv("GOOGLE_API_KEY")
genai.configure(api_key=api_key)

print("Available Gemini models:")
print("=" * 60)

for model in genai.list_models():
    if 'generateContent' in model.supported_generation_methods:
        print(f"  - {model.name}")

print("\nTesting gemini-pro...")
try:
    model = genai.GenerativeModel('gemini-pro')
    response = model.generate_content("Say 'Gemini works!'")
    print(f"✓ gemini-pro WORKS: {response.text}")
except Exception as e:
    print(f"✗ gemini-pro failed: {e}")

print("\nTesting gemini-1.5-pro...")
try:
    model = genai.GenerativeModel('gemini-1.5-pro')
    response = model.generate_content("Say 'Gemini works!'")
    print(f"✓ gemini-1.5-pro WORKS: {response.text}")
except Exception as e:
    print(f"✗ gemini-1.5-pro failed: {e}")
