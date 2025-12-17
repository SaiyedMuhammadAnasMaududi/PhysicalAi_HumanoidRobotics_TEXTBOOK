#!/usr/bin/env python3
"""
CLI Validation Script for Retrieval & Query Validation

This script provides a command-line interface for testing the semantic retrieval
system. It accepts natural-language queries and displays ordered results with
similarity scores and metadata.

Usage:
    python3 backend/scripts/test_retrieval.py "What is ROS2?"
    python3 backend/scripts/test_retrieval.py "robotics sensors" --top_k 10
    python3 backend/scripts/test_retrieval.py "control systems" --threshold 0.7
    python3 backend/scripts/test_retrieval.py "ROS2" --verbose
    python3 backend/scripts/test_retrieval.py "navigation" --format json
"""

import sys
import os
import argparse
import json
from pathlib import Path
from datetime import datetime

# Add backend/src to Python path
backend_src = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(backend_src))

from retrieval import RetrievalPipeline, RetrievalResponse


def format_table_output(response: RetrievalResponse) -> str:
    """Format results as a readable table"""
    lines = []
    lines.append("=" * 100)
    lines.append(f"QUERY: {response.query_text}")
    lines.append(f"RESULTS: {response.total_results} found in {response.execution_time_ms:.0f}ms")
    lines.append("=" * 100)

    if response.total_results == 0:
        lines.append("\nNo results found. Try adjusting your query or lowering the similarity threshold.")
        return "\n".join(lines)

    for result in response.results:
        lines.append(f"\n[{result.rank}] Score: {result.similarity_score:.4f}")
        lines.append(f"    File: {result.source_file}")
        lines.append(f"    Section: {result.section_title}")
        lines.append(f"    Chunk: {result.chunk_sequence}/{result.total_chunks}")

        # Content preview (first 200 characters)
        content_preview = result.content[:200].replace('\n', ' ')
        if len(result.content) > 200:
            content_preview += "..."
        lines.append(f"    Preview: {content_preview}")
        lines.append("-" * 100)

    return "\n".join(lines)


def format_verbose_output(response: RetrievalResponse) -> str:
    """Format results with full metadata for debugging"""
    lines = []
    lines.append("=" * 100)
    lines.append(f"QUERY: {response.query_text}")
    lines.append(f"TIMESTAMP: {response.timestamp.isoformat()}")
    lines.append(f"EXECUTION TIME: {response.execution_time_ms:.2f}ms")
    lines.append(f"TOTAL RESULTS: {response.total_results}")
    lines.append("\nPARAMETERS:")
    for key, value in response.parameters.items():
        lines.append(f"  {key}: {value}")
    lines.append("=" * 100)

    if response.total_results == 0:
        lines.append("\nNo results found.")
        return "\n".join(lines)

    for result in response.results:
        lines.append(f"\n{'=' * 100}")
        lines.append(f"RANK {result.rank} | SIMILARITY: {result.similarity_score:.6f}")
        lines.append(f"{'=' * 100}")
        lines.append(f"Chunk ID: {result.chunk_id}")
        lines.append(f"Source File: {result.source_file}")
        lines.append(f"Section Title: {result.section_title}")
        lines.append(f"Chunk Position: {result.chunk_sequence}/{result.total_chunks}")
        lines.append(f"Content Hash: {result.content_hash}")
        lines.append(f"Processing Timestamp: {result.processing_timestamp.isoformat()}")
        lines.append(f"Token Count: {result.token_count}")
        lines.append("\nFULL CONTENT:")
        lines.append("-" * 100)
        lines.append(result.content)
        lines.append("-" * 100)

    return "\n".join(lines)


def format_json_output(response: RetrievalResponse) -> str:
    """Format results as JSON for programmatic use"""
    data = {
        "query_text": response.query_text,
        "timestamp": response.timestamp.isoformat(),
        "execution_time_ms": response.execution_time_ms,
        "total_results": response.total_results,
        "parameters": response.parameters,
        "results": [
            {
                "rank": r.rank,
                "similarity_score": r.similarity_score,
                "chunk_id": r.chunk_id,
                "source_file": r.source_file,
                "section_title": r.section_title,
                "content": r.content,
                "content_hash": r.content_hash,
                "chunk_sequence": r.chunk_sequence,
                "total_chunks": r.total_chunks,
                "processing_timestamp": r.processing_timestamp.isoformat(),
                "token_count": r.token_count
            }
            for r in response.results
        ]
    }
    return json.dumps(data, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Test semantic retrieval for RAG chatbot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s "What is ROS2?"
  %(prog)s "robotics sensors" --top_k 10
  %(prog)s "control systems" --threshold 0.7 --verbose
  %(prog)s "navigation" --format json > results.json
        """
    )

    parser.add_argument(
        "query",
        type=str,
        help="Natural-language query string"
    )

    parser.add_argument(
        "--top_k",
        type=int,
        default=5,
        help="Maximum number of results to return (default: 5)"
    )

    parser.add_argument(
        "--threshold",
        type=float,
        default=0.0,
        help="Minimum similarity score threshold 0.0-1.0 (default: 0.0)"
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Display full metadata for each result"
    )

    parser.add_argument(
        "--format",
        choices=["table", "json"],
        default="table",
        help="Output format: table or json (default: table)"
    )

    args = parser.parse_args()

    # Validate parameters
    if args.top_k < 1:
        print("Error: --top_k must be at least 1", file=sys.stderr)
        sys.exit(1)

    if not 0.0 <= args.threshold <= 1.0:
        print("Error: --threshold must be between 0.0 and 1.0", file=sys.stderr)
        sys.exit(1)

    try:
        # Initialize retrieval pipeline (reads from environment variables)
        pipeline = RetrievalPipeline()

        # Execute query
        response = pipeline.query(
            query_text=args.query,
            top_k=args.top_k,
            similarity_threshold=args.threshold
        )

        # Format and display output
        if args.format == "json":
            print(format_json_output(response))
        elif args.verbose:
            print(format_verbose_output(response))
        else:
            print(format_table_output(response))

        # Exit with success
        sys.exit(0)

    except ValueError as e:
        print(f"Validation Error: {e}", file=sys.stderr)
        sys.exit(1)

    except ConnectionError as e:
        print(f"Connection Error: {e}", file=sys.stderr)
        print("\nPlease check:", file=sys.stderr)
        print("  - Qdrant URL and API key are correct in backend/.env", file=sys.stderr)
        print("  - Qdrant instance is running and accessible", file=sys.stderr)
        sys.exit(2)

    except Exception as e:
        print(f"Unexpected Error: {e}", file=sys.stderr)
        print("\nPlease check:", file=sys.stderr)
        print("  - All environment variables are set in backend/.env", file=sys.stderr)
        print("  - Cohere API key is valid", file=sys.stderr)
        print(f"  - Collection '{os.getenv('QDRANT_COLLECTION', 'physicalai')}' exists in Qdrant", file=sys.stderr)
        sys.exit(3)


if __name__ == "__main__":
    main()
