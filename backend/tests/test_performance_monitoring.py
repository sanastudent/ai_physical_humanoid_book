"""
Performance monitoring tests
Verify backend endpoints respond within 2 seconds (p95 latency)
Implements SC-002: Verify backend endpoints respond within 2 seconds (p95 latency)
"""
import asyncio
import time
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.main import app
from src.middleware.performance import PerformanceMiddleware, get_performance_stats, is_within_performance_threshold
from fastapi.testclient import TestClient


def test_performance_middleware_tracking():
    """
    Test that performance middleware tracks response times correctly
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Make a few test requests
    for i in range(5):
        response = client.get("/health")
        assert response.status_code == 200

    # Check that stats are being tracked
    stats = get_performance_stats()
    assert stats["total_requests"] >= 5
    assert stats["tracked_responses"] >= 5
    assert "response_times" in stats
    assert len(stats["response_times"]) >= 5


def test_performance_threshold_validation():
    """
    Test that the performance threshold validation works correctly
    """
    # Initially should be within threshold (no requests or all fast requests)
    assert is_within_performance_threshold() == True

    # Add some slow requests to test the validation
    PerformanceMiddleware.request_stats.clear()

    # Simulate some fast requests
    for i in range(8):
        PerformanceMiddleware.record_response("/test", 0.1, 200)  # 100ms responses

    # Add one slow request
    PerformanceMiddleware.record_response("/test", 2.5, 200)  # 2.5s response

    # Should still be within threshold (9 requests, 1 slow = 11% slow, < 25% threshold)
    assert is_within_performance_threshold() == True

    # Add more slow requests to exceed threshold
    for i in range(3):
        PerformanceMiddleware.record_response("/test", 2.5, 200)  # 2.5s responses

    # Now should exceed threshold (12 requests, 4 slow = 33% slow, > 25% threshold)
    assert is_within_performance_threshold() == False


def test_endpoint_response_time_limits():
    """
    Test that endpoints respond within the 2-second requirement
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Test health endpoint
    start_time = time.time()
    response = client.get("/health")
    end_time = time.time()
    response_time = end_time - start_time

    assert response.status_code == 200
    assert response_time <= 2.0, f"Health endpoint took {response_time:.3f}s, exceeds 2s limit"

    # Test performance metrics endpoint
    start_time = time.time()
    response = client.get("/performance")
    end_time = time.time()
    response_time = end_time - start_time

    assert response.status_code == 200
    assert response_time <= 2.0, f"Performance endpoint took {response_time:.3f}s, exceeds 2s limit"


def test_performance_metrics_endpoint():
    """
    Test the performance metrics endpoint functionality
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Make some requests to generate stats
    for i in range(3):
        client.get("/health")
        client.get("/performance")

    response = client.get("/performance")
    assert response.status_code == 200

    data = response.json()
    assert "metrics" in data
    assert "threshold_met" in data
    assert "requirement" in data

    metrics = data["metrics"]
    assert metrics["total_requests"] >= 6  # 3 health + 3 performance requests
    assert metrics["tracked_responses"] >= 6
    assert "p95_latency" in metrics
    assert "average_response_time" in metrics
    assert "slow_requests_count" in metrics

    # The p95 latency should be reasonable (under 2 seconds for our simple endpoints)
    assert metrics["p95_latency"] <= 2.0


def test_slow_requests_endpoint():
    """
    Test the slow requests tracking endpoint
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Add some slow requests manually to test tracking
    PerformanceMiddleware.record_response("/slow-test", 2.5, 200)  # Slow request
    PerformanceMiddleware.record_response("/fast-test", 0.1, 200)  # Fast request

    response = client.get("/performance/slow-requests")
    assert response.status_code == 200

    data = response.json()
    assert "slow_requests" in data
    assert "count" in data
    assert data["count"] >= 1  # Should have at least the slow request we added


def test_embed_endpoint_performance():
    """
    Test embed endpoint performance with mocked dependencies
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Mock the embedding and Qdrant operations to avoid actual API calls
    with patch('src.embed.EmbeddingGenerator.generate_embedding') as mock_embed, \
         patch('src.qdrant_manager.QdrantManager.insert_embeddings') as mock_insert:

        mock_embed.return_value = [0.1] * 1536  # Mock embedding vector
        mock_insert.return_value = None

        # Test embed endpoint with minimal content
        embed_request = {
            "content": "This is a test chapter content for performance testing.",
            "chapter": "Test Chapter"
        }

        start_time = time.time()
        response = client.post("/embed", json=embed_request)
        end_time = time.time()
        response_time = end_time - start_time

        assert response.status_code == 200
        assert response_time <= 2.0, f"Embed endpoint took {response_time:.3f}s, exceeds 2s limit"


def test_query_endpoint_performance():
    """
    Test query endpoint performance with mocked dependencies
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Mock the RAG engine operations to avoid actual API calls
    with patch('src.rag.RAGEngine.query_global') as mock_query:

        mock_query.return_value = {
            "answer": "This is a test answer for performance validation.",
            "citations": [],
            "sources": []
        }

        # Test query endpoint
        query_request = {
            "query": "What is this book about?",
            "mode": "global"
        }

        start_time = time.time()
        response = client.post("/query", json=query_request)
        end_time = time.time()
        response_time = end_time - start_time

        assert response.status_code == 200
        assert response_time <= 2.0, f"Query endpoint took {response_time:.3f}s, exceeds 2s limit"


def test_select_endpoint_performance():
    """
    Test select endpoint performance with mocked dependencies
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Mock the RAG engine operations to avoid actual API calls
    with patch('src.rag.RAGEngine.query_selected') as mock_query:

        mock_query.return_value = {
            "answer": "This is a test answer for selected text.",
            "citations": [],
            "sources": []
        }

        # Test select endpoint
        query_request = {
            "query": "What does this mean?",
            "context": "This is the selected text for testing.",
            "mode": "selected"
        }

        start_time = time.time()
        response = client.post("/select", json=query_request)
        end_time = time.time()
        response_time = end_time - start_time

        assert response.status_code == 200
        assert response_time <= 2.0, f"Select endpoint took {response_time:.3f}s, exceeds 2s limit"


def test_performance_under_load_simulation():
    """
    Simulate performance under load by making multiple requests
    """
    client = TestClient(app)

    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    # Make multiple requests to test performance consistency
    request_times = []
    for i in range(10):
        start_time = time.time()
        response = client.get("/health")
        end_time = time.time()
        response_time = end_time - start_time
        request_times.append(response_time)
        assert response.status_code == 200
        assert response_time <= 2.0, f"Request {i+1} took {response_time:.3f}s, exceeds 2s limit"

    # Calculate p95 latency
    sorted_times = sorted(request_times)
    p95_index = int(0.95 * len(sorted_times))
    p95_latency = sorted_times[min(p95_index, len(sorted_times) - 1)]

    assert p95_latency <= 2.0, f"P95 latency {p95_latency:.3f}s exceeds 2s limit"


def test_performance_threshold_compliance():
    """
    Final comprehensive test to ensure performance threshold compliance
    """
    # Clear existing stats
    PerformanceMiddleware.request_stats.clear()

    client = TestClient(app)

    # Make a series of requests to various endpoints
    endpoints_and_requests = [
        ("/health", {}),
        ("/performance", {}),
        ("/performance/slow-requests", {}),
    ]

    for endpoint, request_data in endpoints_and_requests:
        if endpoint == "/health" or endpoint == "/performance" or endpoint == "/performance/slow-requests":
            response = client.get(endpoint)
        else:
            response = client.post(endpoint, json=request_data)

        assert response.status_code == 200

    # Verify performance metrics
    stats = get_performance_stats()
    assert stats["total_requests"] >= len(endpoints_and_requests)

    # Verify p95 latency is under 2 seconds
    assert stats["p95_latency"] <= 2.0, f"P95 latency {stats['p95_latency']:.3f}s exceeds 2s limit"

    # Verify threshold compliance
    assert is_within_performance_threshold() == True, "Performance threshold not met"


if __name__ == "__main__":
    test_performance_middleware_tracking()
    test_performance_threshold_validation()
    test_endpoint_response_time_limits()
    test_performance_metrics_endpoint()
    test_slow_requests_endpoint()
    test_embed_endpoint_performance()
    test_query_endpoint_performance()
    test_select_endpoint_performance()
    test_performance_under_load_simulation()
    test_performance_threshold_compliance()
    print("✅ All performance monitoring tests passed!")
    print("🎯 Backend endpoints respond within 2 seconds (p95 latency) requirement verified")