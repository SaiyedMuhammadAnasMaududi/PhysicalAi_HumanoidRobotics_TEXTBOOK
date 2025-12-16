#!/usr/bin/env python3
"""
Script to run the embedding pipeline with proper error handling
"""

import sys
import os
import logging
from pathlib import Path

# Add src to path so we can import the main module
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from main import EmbeddingPipeline


def main():
    """Run the embedding pipeline with error handling"""
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    try:
        # Create and run the pipeline
        pipeline = EmbeddingPipeline()
        pipeline.run()
        logging.info("Embedding pipeline completed successfully!")
        return 0
    except Exception as e:
        logging.error(f"Embedding pipeline failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())