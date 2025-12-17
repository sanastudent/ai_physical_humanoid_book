"""Test RAG end-to-end functionality after Gemini API key update"""
import requests
import json
import os

def test_rag_query():
    print("=" * 70)
    print("Testing RAG Query: 'What is ROS?'")
    print("=" * 70)

    try:
        # Send query
        BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
        response = requests.post(
            f"{BACKEND_URL}/query",
            json={"query": "What is ROS?"},
            timeout=60
        )

        print(f"\nStatus Code: {response.status_code}")

        data = response.json()

        # Display answer
        answer = data.get("answer", "")
        print(f"\nAnswer ({len(answer)} chars):")
        print("-" * 70)
        print(answer)
        print("-" * 70)

        # Display citations
        citations = data.get("citations", [])
        print(f"\nCitations ({len(citations)}):")
        if citations:
            for citation in citations:
                print(f"  - {citation}")
        else:
            print("  No citations found")

        # Display sources
        sources = data.get("sources", [])
        print(f"\nSources Retrieved: {len(sources)}")
        if sources:
            print("Top 3 Sources:")
            for i, source in enumerate(sources[:3], 1):
                print(f"  {i}. Chapter: {source.get('chapter')}")
                print(f"     Score: {source.get('score'):.4f}")
                print(f"     Text preview: {source.get('text', '')[:100]}...")

        # Validation checks
        print("\n" + "=" * 70)
        print("RAG Validation Checks")
        print("=" * 70)

        has_context = "No relevant context" not in answer
        answer_generated = len(answer) > 100
        no_quota_error = "429" not in answer and "quota" not in answer.lower()
        no_api_error = "check that the Google API" not in answer

        print(f"  Has Context: {'PASS' if has_context else 'FAIL'}")
        print(f"  Answer Generated (>100 chars): {'PASS' if answer_generated else 'FAIL'}")
        print(f"  No Quota Error: {'PASS' if no_quota_error else 'FAIL'}")
        print(f"  No API Configuration Error: {'PASS' if no_api_error else 'FAIL'}")
        print(f"  Sources Retrieved: {'PASS' if len(sources) > 0 else 'FAIL'}")
        print(f"  Citations Present: {'PASS' if len(citations) > 0 else 'WARN (optional)'}")

        # Overall status
        all_passed = has_context and answer_generated and no_quota_error and no_api_error and len(sources) > 0

        print("\n" + "=" * 70)
        if all_passed:
            print("RESULT: RAG IS FULLY OPERATIONAL")
        else:
            print("RESULT: RAG HAS ISSUES - SEE FAILURES ABOVE")
        print("=" * 70)

        return all_passed

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    import sys
    success = test_rag_query()
    sys.exit(0 if success else 1)
