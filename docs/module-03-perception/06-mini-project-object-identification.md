---
title: Mini-Project: See and Identify
description: A hands-on mini-project to build a real-time object identification system for a humanoid robot using integrated computer vision and ROS 2. Combines object detection, pose estimation, and perception fusion.
keywords: [Mini-Project, Object Identification, Real-time Robotics, Computer Vision, ROS 2, Perception Fusion, Humanoid Robotics, YOLO, MediaPipe, `cv_bridge`]
sidebar_position: 6
learning_objectives:
  - Apply computer vision techniques (object detection, pose estimation) in a practical context.
  - Integrate various perception modules into a cohesive ROS 2 pipeline.
  - Develop a real-time system for identifying and localizing objects in a simulated environment.
  - Understand how perception outputs inform robot actions in a mini-project scenario.
  - Troubleshoot and optimize a multi-sensor perception system for performance and robustness.
---

# Mini-Project: See and Identify

## Bringing it All Together: A Hands-on Perception System

Throughout this module, you've learned about the individual components of robotic perception: computer vision, speech recognition, and sensor fusion, all integrated within the ROS 2 framework. Now, it's time to combine these concepts into a practical mini-project. This "See and Identify" project challenges you to build a system that enables a simulated humanoid robot to visually identify specific objects in its environment in real-time and, conceptually, prepare for interaction.

**Project Goal**: Create a ROS 2-based perception pipeline that uses a simulated camera feed to detect and identify predefined objects (e.g., a "red block," "blue cylinder") and, optionally, human poses/gestures, publishing this information for potential robot interaction.

## Project Setup

We'll assume you have a basic ROS 2 workspace set up and a simulated robot (e.g., in Gazebo) publishing camera images. If you don't, refer back to Module 2 for setting up Gazebo and publishing sensor data.

### Required Components:

1.  **ROS 2 Package**: Create a new ROS 2 Python package (e.g., `perception_mini_project`).
2.  **Simulated Camera**: A Gazebo simulation publishing `sensor_msgs/Image` on a topic like `/camera/image_raw`.
3.  **Dependencies**: `rclpy`, `sensor_msgs`, `cv_bridge`, `opencv-python`, `ultralytics` (for YOLOv8), `mediapipe`.

### Project Structure:

```text
perception_mini_project/
├── package.xml
├── CMakeLists.txt
├── resource/
├── setup.py
├── setup.cfg
└── perception_mini_project/
    ├── __init__.py
    ├── object_detector_node.py
    ├── pose_estimator_node.py
    ├── perception_fusion_node.py
    └── launch/
        └── mini_project_launch.py
```

## Task 1: Real-time Object Detection Node (`object_detector_node.py`)

Create a ROS 2 node that subscribes to the simulated camera feed, uses `cv_bridge` to convert it to an OpenCV image, runs YOLOv8 for object detection, and publishes the detected objects.

### Message Type for Detections

First, define a custom message type for your object detections (e.g., `ObjectDetection.msg` and `ObjectDetectionArray.msg` in `perception_mini_project/msg/`):

**`ObjectDetection.msg`**:

```msg
# perception_mini_project/msg/ObjectDetection.msg
string class_name
float32 confidence
int32 x_min
int32 y_min
int32 x_max
int32 y_max
```

**`ObjectDetectionArray.msg`**:

```msg
# perception_mini_project/msg/ObjectDetectionArray.msg
std_msgs/Header header
ObjectDetection[] detections
```

Remember to update `package.xml` and `CMakeLists.txt` for custom messages (as shown in the ROS 2 Integration chapter) and build your package.

### Node Implementation (`object_detector_node.py`)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

from perception_mini_project.msg import ObjectDetection, ObjectDetectionArray # Custom messages

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(ObjectDetectionArray, '/object_detections', 10)
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt') # Load pre-trained YOLOv8 nano model
        self.get_logger().info("Object Detector Node Started")

    def image_callback(self, data):
        self.get_logger().info('Receiving image frame for detection')
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")

        results = self.model(current_frame, verbose=False) # Run YOLO inference

        detection_array_msg = ObjectDetectionArray()
        detection_array_msg.header = data.header # Use the image's timestamp and frame_id

        for r in results:
            for box in r.boxes:
                detection_msg = ObjectDetection()
                detection_msg.class_name = self.model.names[int(box.cls[0])] # Get class name
                detection_msg.confidence = float(box.conf[0])
                # Bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detection_msg.x_min = x1
                detection_msg.y_min = y1
                detection_msg.x_max = x2
                detection_msg.y_max = y2
                detection_array_msg.detections.append(detection_msg)

                # Optional: Draw bounding boxes on the image for visualization
                cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(current_frame, detection_msg.class_name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        self.publisher_.publish(detection_array_msg)

        # Optional: Display the frame with detections
        cv2.imshow("YOLO Detections", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Task 2 (Optional): Human Pose Estimator Node (`pose_estimator_node.py`)

This task is optional but highly recommended for human-robot interaction. Create a node that subscribes to the camera feed, uses MediaPipe for human pose estimation, and publishes detected human joint landmarks.

### Message Type for Pose Estimation

Define a custom message type for human pose landmarks (e.g., `HumanPose.msg` in `perception_mini_project/msg/`). You can leverage `geometry_msgs/Point` for landmark coordinates.

**`HumanPose.msg`**:

```msg
# perception_mini_project/msg/HumanPose.msg
std_msgs/Header header
string[] landmark_names
geometry_msgs/Point[] landmark_positions
```

### Node Implementation (`pose_estimator_node.py`)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

from perception_mini_project.msg import HumanPose # Custom message
from geometry_msgs.msg import Point

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(HumanPose, '/human_poses', 10)
        self.br = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.pose_estimator = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.get_logger().info("Pose Estimator Node Started")

    def image_callback(self, data):
        self.get_logger().info('Receiving image frame for pose estimation')
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        image_rgb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False

        results = self.pose_estimator.process(image_rgb)

        pose_msg = HumanPose()
        pose_msg.header = data.header

        if results.pose_landmarks:
            self.get_logger().info("Pose landmarks detected")
            for id, lm in enumerate(results.pose_landmarks.landmark):
                pose_msg.landmark_names.append(self.mp_pose.PoseLandmark(id).name) # Append landmark name
                pose_msg.landmark_positions.append(Point(x=lm.x, y=lm.y, z=lm.z))

            # Optional: Draw landmarks on the image for visualization
            image_rgb.flags.writeable = True
            annotated_image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            self.mp_drawing.draw_landmarks(
                annotated_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            cv2.imshow('MediaPipe Pose', annotated_image)
            cv2.waitKey(1)

        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Task 3: Perception Fusion Node (`perception_fusion_node.py`)

Create a node that subscribes to the outputs of the object detector and (optionally) the pose estimator. This node will fuse the information to provide a consolidated understanding of the environment and potentially trigger higher-level robot behaviors.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For aggregated output

from perception_mini_project.msg import ObjectDetectionArray, HumanPose # Custom messages

class PerceptionFusionNode(Node):
    def __init__(self):
        super().__init__('perception_fusion_node')
        self.object_sub = self.create_subscription(
            ObjectDetectionArray, '/object_detections', self.object_callback, 10)
        self.pose_sub = self.create_subscription(
            HumanPose, '/human_poses', self.pose_callback, 10) # Optional, if using pose

        self.fused_publisher_ = self.create_publisher(String, '/fused_perception_output', 10)

        self.latest_objects = None
        self.latest_pose = None
        self.get_logger().info("Perception Fusion Node Started")

    def object_callback(self, msg):
        self.latest_objects = msg
        self.fuse_and_publish()

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.fuse_and_publish()

    def fuse_and_publish(self):
        fused_output = ""
        if self.latest_objects:
            for det in self.latest_objects.detections:
                fused_output += f"Detected {det.class_name} with confidence {det.confidence:.2f} at [{det.x_min},{det.y_min},{det.x_max},{det.y_max}]. "

        if self.latest_pose:
            # Simple fusion: just report if a human pose is detected
            if len(self.latest_pose.landmark_names) > 0:
                fused_output += "Human pose detected. "

        if fused_output:
            msg = String()
            msg.data = f"Fused Perception: {fused_output.strip()}"
            self.fused_publisher_.publish(msg)
            self.get_logger().info(f'Publishing fused output: {msg.data}')
        else:
            self.get_logger().info("No detections or poses to fuse yet.")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Task 4: Launch File (`mini_project_launch.py`)

Create a launch file to start all your perception nodes simultaneously.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_mini_project',
            executable='object_detector_node',
            name='object_detector',
            output='screen',
            parameters=[{'use_sim_time': True}] # Important for simulated environments
        ),
        Node(
            package='perception_mini_project',
            executable='pose_estimator_node',
            name='pose_estimator',
            output='screen',
            parameters=[{'use_sim_time': True}] # Important for simulated environments
        ),
        Node(
            package='perception_mini_project',
            executable='perception_fusion_node',
            name='perception_fusion',
            output='screen',
            parameters=[{'use_sim_time': True}] # Important for simulated environments
        ),
        # Add other nodes here, e.g., your camera publisher node from Module 2
    ])
```

## Running the Mini-Project

1.  **Build your ROS 2 package**: Navigate to your workspace root and run `colcon build --packages-select perception_mini_project`.
2.  **Source the setup files**: `source install/setup.bash` (or `setup.zsh`, etc.).
3.  **Start your simulated environment**: Launch your Gazebo world with a camera publishing images.
4.  **Launch the perception pipeline**: `ros2 launch perception_mini_project mini_project_launch.py`.

You should see output from the nodes detecting objects and poses, and the fusion node aggregating this information. You can use `ros2 topic echo /fused_perception_output` to see the combined perception stream.

## Project Extension Ideas

*   **Targeted Object Recognition**: Modify the fusion node to look for specific objects (e.g., "red block") and publish a boolean `target_found` message.
*   **Gesture-based Commands**: Map specific human poses or gestures (e.g., waving) to robot commands (e.g., "wave back").
*   **3D Object Localization**: Combine object detection with depth data to get the 3D coordinates of detected objects.
*   **Speech-Guided Interaction**: Integrate the speech recognition and intent parsing from the previous chapter to allow vocal commands to control what the perception system is looking for.
*   **Performance Optimization**: Profile your nodes and optimize them for lower latency and higher throughput on embedded hardware.

## Conclusion

This mini-project has provided a hands-on experience in building a multi-modal perception pipeline for a humanoid robot using ROS 2, OpenCV, YOLOv8, and MediaPipe. You've learned to integrate various sensors and algorithms to enable a robot to "see" and interpret its 3D environment in real-time. The ability to perceive and understand the world is a fundamental step towards creating truly autonomous and intelligent humanoid robots capable of complex interactions and tasks. The principles and techniques you've applied here form the basis for advanced robotic applications. This concludes Module 3 on Perception.

## Exercises

1.  **Filtered Object Detection:** Modify the `object_detector_node.py` to allow for configurable object detection. The node should accept a ROS 2 parameter (e.g., `target_classes`) which is a list of strings representing the names of objects to detect. Only publish `ObjectDetection` messages for objects whose `class_name` is in this list. If the list is empty, detect all objects. Test this by launching the node with different `target_classes` parameters.
2.  **3D Object Localization with Depth Data:**
    *   Assume your simulated camera also publishes depth images on a topic like `/camera/depth/image_raw` (e.g., `sensor_msgs/Image` with `16UC1` or `32FC1` encoding for depth in millimeters or meters).
    *   Extend the `perception_fusion_node.py` to subscribe to this depth topic.
    *   When an `ObjectDetection` message is received, use its `x_min`, `y_min`, `x_max`, `y_max` to find the central pixel of the bounding box. Read the depth value at this pixel from the depth image.
    *   Publish an enhanced fused message (perhaps a new custom message type `LocalizedObject.msg` with `geometry_msgs/Point` for 3D location) that includes the object's class, confidence, and its estimated 3D position relative to the camera.
3.  **Simple Gesture-Based Interaction (Conceptual Design):** Building upon the `pose_estimator_node.py` and `perception_fusion_node.py`, design a conceptual system where a specific human gesture triggers a robot action. For example, if a human points towards a detected object (e.g., by checking the relative positions of `WRIST` and `INDEX_FINGER_TIP` landmarks from `HumanPose.msg` in relation to an `ObjectDetection` bounding box):
    *   Describe the logic within `perception_fusion_node` to detect this "pointing" gesture and associate it with a nearby detected object.
    *   Outline a new message type (e.g., `GestureCommand.msg`) that `perception_fusion_node` would publish, including the gesture type and the ID/class of the targeted object.
4.  **Performance Profiling and Optimization:**
    *   Use ROS 2 tracing tools (e.g., `ros2 trace`) or Python's `cProfile` module to profile the `object_detector_node.py` for CPU and memory usage during real-time operation. Identify the most computationally expensive parts of the code.
    *   Propose and implement one optimization (e.g., using a smaller YOLOv8 model like `yolov8n.pt` if not already, or optimizing OpenCV image conversions) and quantify its impact on the node's processing frame rate and latency.
5.  **Voice-Guided Object Search:** Integrate the `whisper_asr_node` and `intent_parser_node` (from the previous chapter's exercises) into this mini-project's launch file. Modify the `perception_fusion_node.py` to subscribe to the `/robot_commands` topic from the `intent_parser_node`. If a command like "look for the red block" is received, the `perception_fusion_node` should then filter its incoming `ObjectDetectionArray` messages and only publish a confirmation to `/fused_perception_output` when the requested object is actually detected.

*   ROS 2 Documentation. (2025). Robot Operating System 2. https://docs.ros.org/
*   YOLOv8 Documentation. (2025). Ultralytics. https://docs.ultralytics.com/
*   MediaPipe Solutions. (2025). Google AI. https://developers.google.com/mediapipe/solutions
*   ROS 2 Tutorials. (2025). Creating custom message and service files. https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Messages-And-Services.html
*   ROS 2 Tutorials. (2025). Launching nodes. https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html
