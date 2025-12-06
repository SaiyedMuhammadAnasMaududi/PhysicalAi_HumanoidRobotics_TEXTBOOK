---
title: ROS 2 Integration of Perception Streams
description: Learn how to integrate computer vision, speech recognition, and sensor fusion modules into the ROS 2 framework for real-time robotic perception. Covers custom message types, `cv_bridge`, and creating ROS 2 nodes for a unified perception pipeline.
keywords:
  - ROS 2
  - Perception
  - Integration
  - cv_bridge
  - Custom Messages
  - ROS 2 Nodes
  - Robotic Perception
  - Humanoid Robotics
sidebar_position: 5
learning_objectives:
  - Understand the principles of integrating perception modules within ROS 2.
  - Learn to use `cv_bridge` for converting between OpenCV images and ROS 2 image messages.
  - Create custom ROS 2 message types for structured perception data.
  - Develop ROS 2 nodes for computer vision, speech recognition, and sensor fusion.
  - Build a comprehensive, real-time perception pipeline using ROS 2.
---

# ROS 2 Integration of Perception Streams

## The Central Nervous System: ROS 2 as the Integration Hub

In a complex humanoid robot, individual perception modules (computer vision, speech recognition, sensor fusion) don't operate in isolation. They need to communicate seamlessly, exchange data in real-time, and coordinate their efforts to form a unified understanding of the environment. **ROS 2 (Robot Operating System 2)** serves as this central nervous system, providing a flexible framework for inter-process communication, hardware abstraction, and package management.

This chapter will focus on how to integrate the perception techniques learned in previous chapters into a robust ROS 2 pipeline, covering:

*   **`cv_bridge`**: Bridging the gap between OpenCV images and ROS 2 messages.
*   **Custom Message Types**: Defining structured data formats for perception outputs.
*   **ROS 2 Nodes**: Creating modular executable units for each perception task.
*   **Quality of Service (QoS)**: Ensuring reliable and timely data delivery for perception streams.
*   **Building a Perception Pipeline**: Putting all the pieces together for a functional system.

## Bridging Vision: `cv_bridge`

Computer vision algorithms are often developed using libraries like OpenCV, which operates on `numpy` arrays for images. ROS 2, however, uses its own `sensor_msgs/Image` message type for image transport. **`cv_bridge`** is a ROS package that provides a convenient bridge between OpenCV image formats and ROS image messages.

### Installation

`cv_bridge` is usually installed as part of the ROS 2 desktop installation. If not, you can install it:

```bash
sudo apt update
sudo apt install ros-{ROS_DISTRO}-cv-bridge
```
(Replace `{ROS_DISTRO}` with your ROS 2 distribution, e.g., `humble`, `iron`, `jazzy`).

### Using `cv_bridge` (Python)

Let's see how to convert between `sensor_msgs/Image` and OpenCV images.

**1. ROS 2 Image Publisher (Conceptual)**

First, imagine a simple ROS 2 node that captures images (e.g., from a webcam) and publishes them as `sensor_msgs/Image`.

```python
# Minimal image publisher (e.g., in a file named `image_publisher_node.py`)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.cap = cv2.VideoCapture(0) # Open default camera
        self.br = CvBridge()
        self.get_logger().info("Image Publisher Node Started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**2. ROS 2 Image Subscriber with OpenCV Processing (`image_processor_node.py`)**

This node subscribes to image messages, converts them to OpenCV format, processes them (e.g., grayscale, edge detection), and then displays the result.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, '/video_frames', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        self.br = CvBridge()
        self.get_logger().info("Image Processor Node Started")

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Example: Convert to grayscale
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Example: Apply Canny edge detection
        edges = cv2.Canny(gray_frame, 100, 200)

        # Display the processed image
        cv2.imshow("Processed Frame", edges)
        cv2.waitKey(1) # Refresh window every 1ms

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run these, save them as Python files in a ROS 2 package, then `ros2 run <your_package> image_publisher_node` and `ros2 run <your_package> image_processor_node` in separate terminals.

## Custom Message Types for Structured Data

While `sensor_msgs/Image` and `std_msgs/String` are useful, complex perception outputs (e.g., object detections with bounding boxes, confidence, and class labels, or parsed speech commands with intent and entities) require custom message types.

### Defining a Custom Message (`RobotCommand.msg`)

Create a `.msg` file in your ROS 2 package's `msg` directory (e.g., `my_robot_perception/msg/RobotCommand.msg`):

```msg
# my_robot_perception/msg/RobotCommand.msg
string intent          # e.g., "pick_up", "wave", "move"
string object_name     # e.g., "red block", "blue sphere"
geometry_msgs/Point target_location # Where to perform the action
float32 confidence     # Confidence score of the parsed command
```

### Building Custom Messages

To make ROS 2 aware of your custom messages, you need to add dependencies and build instructions to your `package.xml` and `CMakeLists.txt`.

**`package.xml` additions**:

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <depend>geometry_msgs</depend> <!-- For geometry_msgs/Point -->
```

**`CMakeLists.txt` additions**:

```cmake
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCommand.msg"
)

# ... other build rules ...

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
)

# ... other install rules ...
```

After modifying these files, build your package with `colcon build --packages-select my_robot_perception`.

### Using Custom Messages in Python

Once built, you can import and use your custom message like any other ROS 2 message:

```python
import rclpy
from rclpy.node import Node
from my_robot_perception.msg import RobotCommand # Import your custom message
from geometry_msgs.msg import Point

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(RobotCommand, 'robot_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1 Hz
        self.get_logger().info("Command Publisher Node Started")

    def timer_callback(self):
        msg = RobotCommand()
        msg.intent = "pick_up"
        msg.object_name = "red block"
        msg.target_location = Point(x=1.0, y=0.5, z=0.1) # Example target point
        msg.confidence = 0.95
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command: {msg.intent} {msg.object_name}')

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Nodes for Perception Modules

Each perception component (vision, speech, sensor fusion) should ideally run as its own ROS 2 node or a set of nodes. This promotes modularity, fault isolation, and distributed processing.

### Example Node Structure

Consider these nodes for a perception pipeline:

*   **`camer-node`**: Publishes raw camera images (`sensor_msgs/Image`).
*   **`yolo_detector_node`**: Subscribes to `sensor_msgs/Image`, runs YOLO inference, publishes object detections (e.g., a custom `ObjectDetectionArray.msg`).
*   **`whisper_asr_node`**: Subscribes to audio data (e.g., from an `audio_capture_node`), transcribes it, publishes `std_msgs/String`.
*   **`intent_parser_node`**: Subscribes to `std_msgs/String` from ASR, parses intent, publishes `my_robot_perception/msg/RobotCommand`.
*   **`imu_node`**: Publishes IMU data (`sensor_msgs/Imu`).
*   **`sensor_fusion_node`**: Subscribes to `ObjectDetectionArray`, `Imu`, and other spatial data, runs Kalman/Particle filter, publishes fused state estimate (e.g., custom `RobotState.msg`).
*   **`perception_aggregator_node`**: A central node that subscribes to various high-level perception outputs (object detections, commands, robot state) and provides a consolidated view for the planning and control systems.

## Quality of Service (QoS) for Perception Streams

ROS 2 QoS settings are crucial for perception, especially for real-time data like images and IMU readings. Key QoS profiles include:

*   **Reliability**: Ensures all messages are delivered (important for commands).
*   **Durability**: Specifies if new subscribers receive old messages (useful for maps).
*   **History**: Keeps a certain number of messages (helpful for buffering).
*   **Liveliness**: Detects if publishers are still active.

For high-bandwidth, real-time perception data (e.g., camera images, LiDAR scans), you often want **best effort reliability** and a small **keep last history** to prioritize fresh data over guaranteed delivery of every single frame.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ... in your node's __init__ method ...

# QoS profile for real-time image data
image_qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1 # Only keep the latest message
)

self.subscription = self.create_subscription(
    Image, '/video_frames', self.listener_callback, qos_profile=image_qos_profile)

# QoS profile for critical command data (e.g., from intent parser)
command_qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10 # Keep a small buffer of commands
)

self.publisher_ = self.create_publisher(
    RobotCommand, 'robot_commands', qos_profile=command_qos_profile)
```

## Building a Comprehensive Perception Pipeline

Putting it all together, a full ROS 2 perception pipeline for a humanoid robot might look like this:

```mermaid
graph LR
    subgraph Sensors
        C(Camera) -- Raw Image --> CP(Camera Publisher Node)
        M(Microphone) -- Raw Audio --> AP(Audio Publisher Node)
        I(IMU) -- Raw IMU Data --> IP(IMU Publisher Node)
        D(Depth Sensor) -- Raw Depth Data --> DP(Depth Publisher Node)
    end

    subgraph Vision Processing
        CP -- /image_raw --> YD(YOLO Detector Node)
        YD -- /objects_detected --> PA(Perception Aggregator Node)
        CP -- /image_raw --> PE(Pose Estimator Node)
        PE -- /human_poses --> PA
    end

    subgraph Speech Processing
        AP -- /audio_raw --> AN(ASR Node (Whisper))
        AN -- /speech_text --> IPN(Intent Parser Node)
        IPN -- /robot_commands --> PA
    end

    subgraph Spatial & Fusion
        DP -- /depth_raw --> SF(Sensor Fusion Node)
        IP -- /imu_data --> SF
        YD -- /objects_detected --> SF
        SF -- /fused_state --> PA
    end

    PA -- /aggregated_perception --> Planner(Planning & Control System)
```

This diagram illustrates how data flows between different ROS 2 nodes, transforming raw sensor inputs into high-level, actionable perception information for the robot's planning and control systems.

## Conclusion

Integrating diverse perception streams into ROS 2 is fundamental for developing autonomous humanoid robots. By mastering `cv_bridge` for vision data, defining custom messages for structured outputs, building modular ROS 2 nodes, and applying appropriate QoS settings, you can create a robust, scalable, and real-time perception pipeline. This integrated system will allow your humanoid robot to perceive its environment and interact with the world with unprecedented sophistication, laying the groundwork for complex behaviors and intelligent decision-making. In the next and final chapter of this module, we will apply all these concepts in a mini-project focusing on real-time object identification and interaction for a humanoid robot.

## Exercises

1.  **`cv_bridge` Image Pipeline Implementation:**
    *   Create a ROS 2 package. Inside it, develop two Python nodes:
        *   `camera_publisher_node`: Uses `cv2.VideoCapture(0)` to grab frames from your webcam. Convert each frame to a `sensor_msgs/Image` using `cv_bridge` and publish it on a topic named `/camera/image_raw`.
        *   `image_processor_node`: Subscribes to `/camera/image_raw`. Converts the `sensor_msgs/Image` back to an OpenCV image, applies a grayscale conversion and a Canny edge detector, and then displays both the original and processed images in separate OpenCV windows.
    *   Run both nodes simultaneously using `ros2 run` and verify the image processing pipeline is working in real-time.
2.  **Custom Message Definition and Usage for Object Detections:**
    *   In your ROS 2 package, define a custom message named `ObjectDetection.msg` in the `msg` directory with the following structure:
        ```msg
        string object_class
        float32 confidence
        int32 x_min
        int32 y_min
        int32 x_max
        int32 y_max
        ```
    *   Create a `yolo_publisher_node` that publishes `ObjectDetection` messages on a topic named `/perception/object_detections`. Simulate detections by creating dummy `ObjectDetection` messages (e.g., for "cup", "bottle", "chair") with varying bounding box coordinates and confidence scores.
    *   Create an `object_subscriber_node` that subscribes to `/perception/object_detections` and prints the `object_class` and `confidence` of each received message.
    *   Ensure your `package.xml` and `CMakeLists.txt` are correctly configured for custom messages and build your package.
3.  **ROS 2 Perception Pipeline Component Design:** Refer to the comprehensive perception pipeline diagram. For the `whisper_asr_node` and `intent_parser_node`:
    *   **`whisper_asr_node`**: Describe its main inputs (topic and message type), its core internal processing logic, and its main outputs (topic and message type). Justify the QoS profile (Reliability, History, Depth) you would choose for its *output* publisher.
    *   **`intent_parser_node`**: Describe its main inputs, its core internal processing logic (how it transforms text to commands), and its main outputs. Justify the QoS profile you would choose for its *output* publisher.
4.  **QoS Profile Justification for Robotic Applications:** For a humanoid robot operating in a sensitive environment (e.g., assisting elderly individuals), justify your choice of QoS profiles (Reliability, History, Depth) for the following critical communication topics, explaining why each setting is appropriate:
    *   `/robot/joint_states` (high-frequency joint position and velocity feedback).
    *   `/robot/emergency_stop` (command to immediately halt all robot movement).
    *   `/navigation/global_plan` (updated path for the robot to follow, less frequent).
    *   `/perception/human_detection_alerts` (warnings about human proximity).
5.  **Scalability and Robustness of ROS 2 Perception:** Discuss how the modular, distributed nature of ROS 2 (using separate nodes for each perception function) contributes to the scalability and robustness of a humanoid robot's perception system. Specifically, explain:
    *   How it allows for easier debugging and fault isolation.
    *   How it facilitates hardware acceleration and distributed computing.
    *   What considerations need to be made regarding inter-node communication delays (latency) when designing such a system.

*   ROS 2 Documentation. (2025). Robot Operating System 2. https://docs.ros.org/
*   ROS 2 Tutorials. (2025). Writing a simple publisher and subscriber (Python). https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Publisher-And-Subscriber--Python.html
*   ROS 2 Tutorials. (2025). Using `cv_bridge` with ROS 2 and OpenCV. https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html (Incorrect reference, actual `cv_bridge` tutorial is at https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Ros2-And-CvBridge-With-OpenCv.html)
*   ROS 2 Documentation. (2025). Quality of Service Policies. https://docs.ros.org/en/foxy/Concepts/About-Quality-Of-Service-Settings.html
*   ROS 2 Tutorials. (2025). Creating custom message and service files. https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Messages-And-Services.html
*   Paz, A. (2020). *ROS 2 in 7 Days*. Packt Publishing.
