# Quickstart: Retrieval & Query Validation

**Feature**: 008-retrieval-query-validation
**Prerequisites**: Spec 1 (007-book-embedding-pipeline) completed
**Est. Time**: 5 minutes

## Prerequisites

Before starting, ensure you have:

1. ✅ **Spec 1 Completed**: The embedding pipeline has run successfully
2. ✅ **Qdrant Collection**: 'physicalai' collection populated with 550 embedded chunks
3. ✅ **Environment Configured**: `.env` file with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
4. ✅ **Dependencies Installed**: From Spec 1 - cohere, qdrant-client, python-dotenv

**Verify Prerequisites**:
```bash
# Check Qdrant collection
python3 -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
load_dotenv('backend/.env')
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
count = client.count(collection_name='physicalai')
print(f'Qdrant collection contains {count.count} vectors')
assert count.count == 550, 'Expected 550 vectors'
print('✅ Prerequisites met!')
"
```

---

## 5-Minute Quickstart

### Step 1: Run Your First Query

```bash
cd backend
python3 scripts/test_retrieval.py "What is ROS2?"
```

**Expected Output**:
```
Query: "What is ROS2?"
Parameters: top_k=5, similarity_threshold=0.0
Execution time: 1.2s

Results:
┌──────┬──────────┬────────────────────────────────────────┬─────────────────────────────────────────┐
│ Rank │ Score    │ Source File                            │ Section                                 │
├──────┼──────────┼────────────────────────────────────────┼─────────────────────────────────────────┤
│ 1    │ 0.87     │ module-01-ros2/01-introduction.md      │ What is ROS2?                           │
│ 2    │ 0.82     │ module-01-ros2/01-introduction.md      │ Key Features of ROS2                    │
│ 3    │ 0.76     │ module-01-ros2/02-ros2-installation... │ Getting Started with ROS2               │
│ 4    │ 0.71     │ module-01-ros2/03-nodes-topics...      │ ROS2 Architecture                       │
│ 5    │ 0.65     │ module-01-ros2/04-publishers...        │ Building Your First ROS2 Node           │
└──────┴──────────┴────────────────────────────────────────┴─────────────────────────────────────────┘

✅ All results are from module-01 (ROS2) - semantically correct!
```

---

### Step 2: Adjust Retrieval Parameters

**Try Different `top_k` Values**:
```bash
# Get top 3 results
python3 scripts/test_retrieval.py "Gazebo simulation" --top_k 3

# Get top 10 results
python3 scripts/test_retrieval.py "robotics perception" --top_k 10
```

**Try Similarity Thresholds**:
```bash
# Only highly relevant results (score >= 0.7)
python3 scripts/test_retrieval.py "computer vision" --threshold 0.7

# Moderate relevance (score >= 0.5)
python3 scripts/test_retrieval.py "behavior trees" --threshold 0.5

# All results (no filtering)
python3 scripts/test_retrieval.py "humanoid robots" --threshold 0.0
```

---

### Step 3: Test Module-Specific Queries

Run queries for each book module to validate retrieval coverage:

```bash
# Module 1: ROS2
python3 scripts/test_retrieval.py "How do I install ROS2?"

# Module 2: Simulation
python3 scripts/test_retrieval.py "Gazebo URDF models"

# Module 3: Perception
python3 scripts/test_retrieval.py "Computer vision for robots"

# Module 4: Control
python3 scripts/test_retrieval.py "Task planning with behavior trees"

# Module 5: Capstone
python3 scripts/test_retrieval.py "Integration and deployment"
```

**Expected**: Each query should return chunks primarily from the corresponding module.

---

### Step 4: View Detailed Results

For full content and metadata:

```bash
# JSON output for programmatic use
python3 scripts/test_retrieval.py "ROS2 nodes" --format json > results.json

# Verbose mode with full content
python3 scripts/test_retrieval.py "sensor simulation" --verbose
```

**Verbose Output Example**:
```
Result #1
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Similarity Score: 0.89
Rank: 1
Source File: ../docs/module-02-simulation/04-sensor-simulation.md
Section: Simulating Sensors in Gazebo
Chunk: 0 of 12
Content:
    Sensor simulation is a critical component of robotics development.
    Gazebo provides built-in support for simulating cameras, LiDAR,
    IMU sensors, and more. This section covers how to add and configure
    virtual sensors in your robot model...

    [Content continues for ~512 tokens]

Processing Time: 2025-12-16 10:30:00
Content Hash: a1b2c3d4...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

## Common Queries for Testing

### Recommended Test Queries

| Query | Expected Module | Purpose |
|-------|-----------------|---------|
| "What is ROS2?" | module-01-ros2 | Basic concept retrieval |
| "Install Gazebo simulator" | module-02-simulation | Setup instructions |
| "Computer vision with OpenCV" | module-03-perception | Technology-specific |
| "Behavior tree execution" | module-04-control | Advanced concepts |
| "Deploy humanoid robot" | module-05-capstone | Integration topics |
| "Connect sensors to ROS2" | module-01 + module-03 | Cross-module queries |

### Edge Case Queries

```bash
# Empty query (should error)
python3 scripts/test_retrieval.py ""

# Special characters
python3 scripts/test_retrieval.py "What is C++?"

# Very long query
python3 scripts/test_retrieval.py "$(printf 'robotics %.0s' {1..200})"

# Multilingual (if testing multilingual support)
python3 scripts/test_retrieval.py "¿Qué es ROS2?"
```

---

## Configuration

### Environment Variables (`.env`)

```bash
# Required (from Spec 1)
COHERE_API_KEY=<your-cohere-api-key>
QDRANT_URL=<your-qdrant-url>
QDRANT_API_KEY=<your-qdrant-api-key>
QDRANT_COLLECTION=physicalai
EMBEDDING_MODEL=embed-multilingual-v3.0

# Optional defaults
DEFAULT_TOP_K=5
DEFAULT_SIMILARITY_THRESHOLD=0.0
```

### Script Parameters

```bash
python3 scripts/test_retrieval.py [OPTIONS] "query text"

Options:
  --top_k INTEGER          Number of results (default: 5)
  --threshold FLOAT        Minimum similarity score 0.0-1.0 (default: 0.0)
  --format [table|json]    Output format (default: table)
  --verbose                Show full content and metadata
  --help                   Show help message
```

---

## Troubleshooting

### Issue: "No results found"

**Possible Causes**:
1. Similarity threshold too high
2. Query topic not in book content
3. Qdrant collection empty

**Solutions**:
```bash
# Try with threshold=0.0 (no filtering)
python3 scripts/test_retrieval.py "your query" --threshold 0.0

# Verify collection has data
python3 -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
load_dotenv('backend/.env')
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
print(f'Vectors: {client.count(collection_name=\"physicalai\").count}')
"
```

---

### Issue: "ConnectionError: Cannot reach Qdrant"

**Solutions**:
1. Check Qdrant URL in `.env` is correct
2. Verify Qdrant instance is running
3. Check network/firewall settings

```bash
# Test Qdrant connection
curl -X GET "$QDRANT_URL/collections/physicalai" \
  -H "api-key: $QDRANT_API_KEY"
```

---

### Issue: "Cohere API error"

**Solutions**:
1. Verify API key is valid
2. Check rate limits
3. Ensure internet connection

```bash
# Test Cohere API
python3 -c "
import cohere
import os
from dotenv import load_dotenv
load_dotenv('backend/.env')
client = cohere.Client(os.getenv('COHERE_API_KEY'))
result = client.embed(texts=['test'], model='embed-multilingual-v3.0', input_type='search_query')
print(f'✅ Cohere API working - embedding dimension: {len(result.embeddings[0])}')
"
```

---

## Performance Expectations

**Typical Query Times**:
- Query embedding generation: 200-500ms
- Qdrant search: 50-200ms
- Result formatting: <10ms
- **Total**: <2 seconds for 95% of queries

**If queries are slower**:
- Check network latency to Cohere/Qdrant
- Verify Qdrant instance has adequate resources
- Consider reducing `top_k` for faster searches

---

## Next Steps

After validating retrieval works correctly:

1. ✅ **Document Optimal Thresholds**: Based on your testing, update this guide with recommended threshold values for different query types

2. ✅ **Run Full Validation Checklist**: Complete `validation-checklist.md`

3. → **Ready for Spec 3**: Agent Integration with LiteLLM

---

## Advanced Usage

### Programmatic API Usage

```python
from backend.src.retrieval import RetrievalPipeline
import os
from dotenv import load_dotenv

load_dotenv('backend/.env')

# Initialize pipeline
pipeline = RetrievalPipeline(
    cohere_api_key=os.getenv('COHERE_API_KEY'),
    qdrant_url=os.getenv('QDRANT_URL'),
    qdrant_api_key=os.getenv('QDRANT_API_KEY'),
    collection_name='physicalai'
)

# Execute query
response = pipeline.query(
    query_text="What is ROS2?",
    top_k=5,
    similarity_threshold=0.7
)

# Access results
for result in response.results:
    print(f"Score: {result.similarity_score:.2f}")
    print(f"File: {result.source_file}")
    print(f"Content: {result.content[:100]}...")
    print()
```

### Batch Queries

```python
queries = [
    "What is ROS2?",
    "Gazebo simulation",
    "Computer vision",
    "Behavior trees",
    "Capstone project"
]

for query in queries:
    response = pipeline.query(query, top_k=3)
    print(f"{query}: {len(response.results)} results")
```

---

## Tips & Best Practices

1. **Start with `threshold=0.0`**: See all results, then raise threshold to filter
2. **Use `top_k=10`** for exploratory queries, `top_k=3` for precise answers
3. **Check source files**: Verify results come from expected modules
4. **Validate consistency**: Same query should return same results every time
5. **Test edge cases**: Empty queries, special chars, very long queries

---

## Resources

- **Spec Document**: `specs/008-retrieval-query-validation/spec.md`
- **Plan Document**: `specs/008-retrieval-query-validation/plan.md`
- **Data Model**: `specs/008-retrieval-query-validation/data-model.md`
- **Spec 1 (Embedding Pipeline)**: `specs/007-book-embedding-pipeline/`
- **Cohere Docs**: https://docs.cohere.com
- **Qdrant Docs**: https://qdrant.tech/documentation
