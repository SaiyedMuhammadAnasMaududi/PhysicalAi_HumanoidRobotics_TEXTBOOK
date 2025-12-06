---
title: "Publishers and Subscribers in Python"
description: "Implement your first ROS 2 nodes using Python to publish and subscribe to topics for continuous data streaming"
keywords: ["ros2", "python", "publisher", "subscriber", "rclpy", "nodes"]
sidebar_position: 4
learning_objectives:
  - "Create a ROS 2 publisher node in Python using rclpy"
  - "Implement a subscriber node to process incoming messages"
  - "Understand node lifecycle and best practices"
  - "Build and run custom packages in a ROS 2 workspace"
---

# Publishers and Subscribers in Python

## Introduction

In previous chapters, you learned the concepts behind ROS 2's publish-subscribe communication and used command-line tools to inspect running systems. Now it's time to write code!

This chapter teaches you to implement **publisher** and **subscriber** nodes in Python using `rclpy` (ROS Client Library for Python). You'll create a custom package, write nodes that communicate via topics, and understand best practices for node design.

By the end of this chapter, you'll have hands-on experience creating robot software components that exchange continuous data streams—a fundamental skill for all subsequent modules.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create a ROS 2 publisher node in Python using rclpy
- Implement a subscriber node to process incoming messages
- Understand node lifecycle and best practices
- Build and run custom packages in a ROS 2 workspace

## Prerequisites

- ROS 2 Humble installed with development tools (Chapter 2)
- Understanding of computation graphs, nodes, and topics (Chapter 3)
- Basic Python programming knowledge (functions, classes, imports)
- Active ROS 2 workspace (`~/ros2_ws`)

---

## Creating a ROS 2 Python Package

### Package Structure

A ROS 2 Python package has this structure:

```
my_robot_controller/
├── package.xml          # Package metadata and dependencies
├── setup.py            # Python package configuration
├── setup.cfg           # Entry points configuration
├── resource/           # Package marker files
│   └── my_robot_controller
└── my_robot_controller/  # Python module
    ├── __init__.py
    ├── publisher_node.py
    └── subscriber_node.py
```

### Creating the Package

```bash
# Navigate to workspace src directory
cd ~/ros2_ws/src

# Create package with rclpy dependency
ros2 pkg create --build-type ament_python my_robot_controller \
  --dependencies rclpy std_msgs

# Expected output:
# going to create a new package
# package name: my_robot_controller
# destination directory: /home/<user>/ros2_ws/src
# ...
```

**Parameters explained:**
- `--build-type ament_python`: Python package (not C++)
- `--dependencies rclpy std_msgs`: Automatically adds dependencies to `package.xml`

---

## Building a Publisher Node

### Understanding the Publisher Pattern

A **publisher node**:
1. Initializes the ROS 2 Python client library (`rclpy`)
2. Creates a node instance
3. Creates a publisher object for a specific topic and message type
4. Periodically publishes messages using a timer callback
5. Spins (processes callbacks) until shutdown

### Complete Publisher Example

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/publisher_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A simple ROS 2 publisher node that demonstrates basic pub-sub communication.

    This node publishes String messages to the '/robot_status' topic at 1 Hz,
    counting the number of messages published.
    """

    def __init__(self):
        # Initialize the node with name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher on topic '/robot_status' with String message type
        # Queue size of 10 means ROS will buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Create a timer that calls timer_callback every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.counter = 0

        # Log initialization
        self.get_logger().info('Publisher node has been started')

    def timer_callback(self):
        """
        Timer callback executed every timer_period seconds.
        Publishes a String message with current counter value.
        """
        # Create message instance
        msg = String()
        msg.data = f'Robot operational - Message count: {self.counter}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message (visible in terminal)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.counter += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node instance
    minimal_publisher = MinimalPublisher()

    # Spin the node (process callbacks) until Ctrl+C
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    # Cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Code Breakdown

**Key components:**

1. **Class inheritance from Node**: All ROS 2 nodes inherit from `rclpy.node.Node`
2. **Publisher creation**: `self.create_publisher(msg_type, topic_name, queue_size)`
3. **Timer for periodic publishing**: `self.create_timer(period, callback)`
4. **Logging**: `self.get_logger().info(message)` for debug output
5. **Main function**: Initializes rclpy, creates node, spins, and cleans up

:::tip
**Best Practice**: Always name your nodes descriptively (not just "node1"). Use logging extensively for debugging. Handle Ctrl+C gracefully with try/except.
:::

---

## Building a Subscriber Node

### Understanding the Subscriber Pattern

A **subscriber node**:
1. Initializes rclpy and creates a node
2. Creates a subscriber object for a specific topic and message type
3. Registers a callback function to process incoming messages
4. Spins to process callbacks whenever messages arrive
5. Cleans up on shutdown

### Complete Subscriber Example

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/subscriber_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A simple ROS 2 subscriber node that demonstrates receiving messages.

    This node subscribes to the '/robot_status' topic and logs received messages.
    """

    def __init__(self):
        # Initialize the node with name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription to '/robot_status' topic
        # Queue size of 10 matches publisher for consistent behavior
        self.subscription = self.create_subscription(
            String,                    # Message type
            'robot_status',            # Topic name
            self.listener_callback,    # Callback function
            10                         # Queue size (QoS)
        )

        # Prevent unused variable warning (subscription must be stored)
        self.subscription

        # Log initialization
        self.get_logger().info('Subscriber node has been started')

    def listener_callback(self, msg):
        """
        Callback function executed whenever a message arrives on the topic.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node instance
    minimal_subscriber = MinimalSubscriber()

    # Spin the node (process callbacks) until Ctrl+C
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    # Cleanup
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Code Breakdown

**Key components:**

1. **Subscription creation**: `self.create_subscription(msg_type, topic, callback, qos)`
2. **Callback function**: Automatically invoked when messages arrive
3. **Message parameter**: Callback receives the message as parameter
4. **Asynchronous processing**: Callback must return quickly (don't block)

:::warning
**Performance Consideration**: Subscriber callbacks should execute quickly. If processing takes significant time, use threading or move heavy computation to separate nodes to avoid blocking message reception.
:::

---

## Configuring Package Entry Points

### Editing setup.py

To make your nodes executable, add entry points to `setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple publisher-subscriber example package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_robot_controller.publisher_node:main',
            'subscriber = my_robot_controller.subscriber_node:main',
        ],
    },
)
```

**Entry points explained:**
- `'publisher'`: Command name you'll use with `ros2 run`
- `my_robot_controller.publisher_node:main`: Python module and function to execute

---

## Building and Running the Package

### Build the Workspace

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_robot_controller

# Expected output:
# Starting >>> my_robot_controller
# Finished <<< my_robot_controller [1.23s]
#
# Summary: 1 package finished [1.45s]
```

### Source the Workspace

```bash
# Source the overlay to access your package
source ~/ros2_ws/install/setup.bash

# Verify package is available
ros2 pkg list | grep my_robot_controller
# Should output: my_robot_controller
```

### Run the Publisher

```bash
# Run publisher node
ros2 run my_robot_controller publisher

# Expected output:
# [INFO] [minimal_publisher]: Publisher node has been started
# [INFO] [minimal_publisher]: Publishing: "Robot operational - Message count: 0"
# [INFO] [minimal_publisher]: Publishing: "Robot operational - Message count: 1"
# [INFO] [minimal_publisher]: Publishing: "Robot operational - Message count: 2"
# ...
```

### Run the Subscriber (in a new terminal)

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Run subscriber node
ros2 run my_robot_controller subscriber

# Expected output:
# [INFO] [minimal_subscriber]: Subscriber node has been started
# [INFO] [minimal_subscriber]: I heard: "Robot operational - Message count: 5"
# [INFO] [minimal_subscriber]: I heard: "Robot operational - Message count: 6"
# [INFO] [minimal_subscriber]: I heard: "Robot operational - Message count: 7"
# ...
```

### Verify Communication

In a third terminal:

```bash
# List nodes
ros2 node list
# /minimal_publisher
# /minimal_subscriber

# List topics
ros2 topic list
# /parameter_events
# /robot_status
# /rosout

# Echo topic
ros2 topic echo /robot_status
```

---

## Best Practices for Node Development

### 1. Node Naming

✅ **Good**: Descriptive, lowercase with underscores
```python
super().__init__('camera_image_processor')
```

❌ **Bad**: Vague or generic names
```python
super().__init__('node1')  # Too generic
```

### 2. Logging Levels

Use appropriate log levels:

```python
self.get_logger().debug('Detailed diagnostic information')
self.get_logger().info('Informational message')
self.get_logger().warn('Warning condition')
self.get_logger().error('Error condition')
self.get_logger().fatal('Critical failure')
```

### 3. Parameter Management

Make nodes configurable using parameters:

```python
# Declare parameter with default value
self.declare_parameter('timer_period', 1.0)

# Get parameter value
timer_period = self.get_parameter('timer_period').value

# Use in timer creation
self.timer = self.create_timer(timer_period, self.callback)
```

Run with custom parameter:
```bash
ros2 run my_robot_controller publisher --ros-args -p timer_period:=0.5
```

### 4. Graceful Shutdown

Always handle cleanup properly:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

---

## Exercises

### Exercise 1: Modify Publishing Rate

**Objective**: Experiment with timer periods

**Instructions**:
1. Edit `publisher_node.py` to change `timer_period` from 1.0 to 0.5 seconds
2. Rebuild: `colcon build --packages-select my_robot_controller`
3. Source and run publisher
4. Verify messages publish at 2 Hz using `ros2 topic hz /robot_status`

**Expected Outcome**: Publisher sends messages twice per second

### Exercise 2: Custom Message Content

**Objective**: Modify published message data

**Instructions**:
1. Change the message to include a timestamp
2. Use `import time` and `time.time()` to add current timestamp
3. Example: `msg.data = f'Timestamp: {time.time()}'`
4. Rebuild and verify subscriber receives new format

**Expected Outcome**: Messages include timestamp information

### Exercise 3: Multiple Subscribers

**Objective**: Understand many-to-many communication

**Instructions**:
1. Run one publisher node
2. Run two subscriber nodes in separate terminals (both using same executable)
3. Observe both subscribers receive identical messages

**Expected Outcome**: Both subscribers receive all messages demonstrating pub-sub broadcasting

### Exercise 4: Add Parameter Configuration

**Objective**: Make nodes configurable

**Instructions**:
1. Add parameter to publisher for custom message prefix
2. Use `self.declare_parameter('message_prefix', 'Robot')`
3. Retrieve and use in message construction
4. Run with: `ros2 run my_robot_controller publisher --ros-args -p message_prefix:="Custom"`

**Expected Outcome**: Messages use custom prefix from parameter

### Exercise 5: Implement a Relay Node

**Objective**: Combine publisher and subscriber in one node

**Instructions**:
1. Create `relay_node.py` that subscribes to `/robot_status`
2. Republish received messages to `/robot_status_relay`
3. Transform messages (e.g., convert to uppercase)
4. Test: Publisher → Relay → Subscriber chain

**Expected Outcome**: Demonstrates multi-hop communication

---

## Summary

Key takeaways from this chapter:

- **Package Creation**: Use `ros2 pkg create` with `--build-type ament_python` for Python packages
- **Publisher Implementation**: Create publisher with `create_publisher()`, use timers for periodic publishing, log activity for debugging
- **Subscriber Implementation**: Create subscription with `create_subscription()`, implement callback for message processing
- **Build System**: Configure `setup.py` entry points, build with `colcon build`, source workspace overlay before running
- **Best Practices**: Use descriptive node names, appropriate logging levels, parameter configuration, and graceful shutdown handling

You now have hands-on experience writing ROS 2 nodes in Python. You've created a custom package, implemented publisher-subscriber communication, and understand the node lifecycle. These foundational coding skills enable you to build sophisticated robot software systems in subsequent modules.

---

## What's Next?

In [Chapter 5: Humanoid URDF Models](./humanoid-urdf), you'll learn to define robot structure using URDF (Unified Robot Description Format), visualize robot models in RViz2, and understand coordinate transformations.

---

## References

1. Open Robotics. (2023). *Writing a simple publisher and subscriber (Python)*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

2. Open Robotics. (2023). *Creating a package*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

3. Open Robotics. (2023). *rclpy API documentation*. Retrieved from https://docs.ros.org/en/humble/p/rclpy/

4. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, *7*(66), eabm6074.

---

## Additional Resources

- [rclpy Examples](https://github.com/ros2/examples/tree/humble/rclpy) - Official Python example nodes
- [ROS 2 Python Package Template](https://github.com/ros2/ros2_tracing) - Package structure reference
- [Python Style Guide for ROS](http://wiki.ros.org/PyStyleGuide) - Coding conventions

---

**Word Count**: ~2,100 words
**Reading Level**: Flesch-Kincaid Grade 8.7
**Last Updated**: 2025-12-05
**Code Validation Status**: ⏳ Pending validation (ROS 2 Humble, Python 3.10)
