# Code Example Validation Workflow

**Purpose**: Ensure all code examples in the textbook are tested, functional, and reproducible before publication.

**Last Updated**: 2025-12-05

---

## Requirements

All code examples MUST:
- ✅ Run without errors in specified environment
- ✅ Include inline comments explaining key concepts
- ✅ Be reproducible by students with documented setup
- ✅ Follow language best practices and style guides
- ✅ Include expected output or behavior description

---

## Supported Languages and Environments

### Python 3.8+
- **Use Cases**: ROS 2 nodes, perception algorithms, planning logic
- **Testing Environment**: Python 3.8+ with virtual environment
- **Style Guide**: PEP 8
- **Required**: Type hints encouraged, docstrings for functions

### JavaScript/TypeScript (ES6+)
- **Use Cases**: MCP server examples, agent workflows, web-based tools
- **Testing Environment**: Node.js 18+
- **Style Guide**: Airbnb JavaScript Style Guide
- **Required**: Modern ES6+ syntax, clear variable names

### ROS 2 (Humble/Iron)
- **Use Cases**: Robot control nodes, simulation integration
- **Testing Environment**: ROS 2 Humble or Iron on Ubuntu 22.04 (or Docker)
- **Required**: Package dependencies documented, launch files tested

### YAML/XML
- **Use Cases**: URDF models, configuration files, launch files
- **Testing Environment**: Validated against schema
- **Required**: Proper indentation, comments for complex structures

---

## Validation Workflow

### Step 1: Code Example Creation

When writing code for a chapter:

1. **Create isolated example**: Code should be self-contained when possible
2. **Add inline comments**: Explain key concepts for educational purposes
3. **Document dependencies**: List all required packages/libraries
4. **Include setup instructions**: How to prepare environment

**Example Structure**:
```python
# file: example_ros2_publisher.py
# Purpose: Demonstrates basic ROS 2 publisher node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    A basic ROS 2 publisher node that publishes messages to a topic.

    This example demonstrates:
    - Node initialization
    - Publisher creation
    - Timer callback for periodic publishing
    """

    def __init__(self):
        super().__init__('simple_publisher')
        # Create publisher on 'example_topic' with String message type
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)

        # Create timer that calls timer_callback every 1.0 seconds
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Callback function executed by timer to publish messages."""
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Manual Testing

**Before including code in chapter**:

1. **Setup test environment**:
   ```bash
   # For Python examples
   python3 -m venv test_env
   source test_env/bin/activate
   pip install -r requirements.txt

   # For ROS 2 examples
   source /opt/ros/humble/setup.bash
   ```

2. **Run the code**:
   ```bash
   # Python
   python3 example_script.py

   # ROS 2
   ros2 run package_name node_name
   ```

3. **Verify expected behavior**:
   - Check output matches documentation
   - Confirm no error messages
   - Test with edge cases if applicable

4. **Document results**:
   ```markdown
   **Test Results**:
   - Environment: Python 3.10, Ubuntu 22.04
   - Date Tested: 2025-12-05
   - Expected Output: Prints "Hello ROS 2: N" every second
   - Actual Output: ✅ Matches expected
   - Edge Cases Tested: None (basic example)
   ```

### Step 3: Code Review Checklist

Before marking code as validated:

- [ ] Code runs without errors in clean environment
- [ ] Dependencies are documented and versions specified
- [ ] Inline comments explain educational concepts (not just what code does)
- [ ] Variable names are clear and descriptive
- [ ] Follows language style guide (PEP 8 for Python, etc.)
- [ ] Includes docstrings or function documentation
- [ ] Expected output is documented
- [ ] Setup instructions are clear enough for students
- [ ] Error handling is appropriate (don't hide failures)
- [ ] Code is formatted consistently (use `black` for Python, `prettier` for JS)

### Step 4: Integration into Chapter

When adding code to chapter markdown:

1. **Include setup section**:
   ```markdown
   ### Prerequisites

   Before running this example, ensure you have:
   - ROS 2 Humble installed
   - Python 3.8 or higher
   - Required packages: `pip install rclpy`
   ```

2. **Add code block with language identifier**:
   ````markdown
   ```python
   # Your code here
   ```
   ````

3. **Explain the code**:
   ```markdown
   **Key Points**:
   - Line 5: Creates a publisher on the 'example_topic'
   - Line 8: Timer triggers callback every 1.0 seconds
   - Line 12: Publishes a String message with counter
   ```

4. **Document expected output**:
   ```markdown
   **Expected Output**:
   ```
   [INFO] [1638360000.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 0"
   [INFO] [1638360001.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 1"
   [INFO] [1638360002.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 2"
   ```
   ```

### Step 5: Validation Badge

Add validation badge to chapter frontmatter:
```yaml
---
code_validated: true
validation_date: 2025-12-05
test_environment: "Python 3.10, ROS 2 Humble, Ubuntu 22.04"
---
```

---

## Common Issues and Solutions

### Issue: Code works locally but not for students

**Solution**:
- Document all environment setup steps explicitly
- Include dependency versions
- Test in clean Docker container
- Provide troubleshooting section

### Issue: Code example too complex for students

**Solution**:
- Break into smaller examples
- Add intermediate steps
- Provide commented version and clean version
- Link to simpler prerequisites

### Issue: Code becomes outdated

**Solution**:
- Include version information in comments
- Schedule annual code review
- Mark deprecated examples clearly
- Provide migration notes for newer versions

---

## Testing Environments

### Recommended Docker Setup

For consistent testing across all code examples:

```dockerfile
# Dockerfile for testing environment
FROM ros:humble-ros-base-jammy

# Install Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Install common Python packages
RUN pip3 install numpy opencv-python pyyaml

# Setup ROS 2 workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy test scripts
COPY tests/ /workspace/tests/

CMD ["/bin/bash"]
```

**Usage**:
```bash
# Build test environment
docker build -t textbook-test-env .

# Run code validation
docker run -it --rm -v $(pwd):/workspace textbook-test-env bash
source /opt/ros/humble/setup.bash
python3 tests/validate_examples.py
```

---

## Validation Status Tracking

Track validation status in `docs/validation-status.md`:

| Chapter | Code Examples | Validated | Last Tested | Environment | Status |
|---------|---------------|-----------|-------------|-------------|--------|
| Module 1 - Ch 2 | 3 | ✅ | 2025-12-05 | Python 3.10, ROS 2 Humble | PASS |
| Module 1 - Ch 3 | 5 | ✅ | 2025-12-05 | Python 3.10, ROS 2 Humble | PASS |
| Module 2 - Ch 2 | 2 | ⏳ | Pending | N/A | IN PROGRESS |

---

## Continuous Validation (Future Enhancement)

For automated testing:

```yaml
# .github/workflows/validate-code.yml
name: Validate Code Examples

on:
  push:
    paths:
      - 'docs/**/*.md'

jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Extract and test code examples
        run: |
          python3 scripts/extract_code_blocks.py
          python3 scripts/run_tests.py
```

---

**Validation Workflow Summary**:
1. ✅ Create code example with comments
2. ✅ Test in clean environment
3. ✅ Review against checklist
4. ✅ Document setup and output
5. ✅ Add validation badge to chapter
6. ✅ Track status in validation table

**All code examples must pass validation before chapter is marked complete.**
