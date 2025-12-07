---
title: "Mini-Project: Voice-Controlled Pick-and-Place"
description: "Build an integrated system where a humanoid executes pick-and-place tasks via voice commands using VLA architecture, behavior trees, and safety constraints"
keywords: ["mini-project", "voice control", "pick-and-place", "integration project", "VLA implementation", "behavior tree control"]
learning_objectives:
  - "Integrate perception, planning, and control systems into a cohesive application"
  - "Implement voice-controlled robot manipulation in simulation"
  - "Apply safety constraints and error handling to real-world scenarios"
  - "Test and validate end-to-end autonomous behavior"
---

# Mini-Project: Voice-Controlled Pick-and-Place

## Project Overview

This mini-project synthesizes all concepts from Module 4 into a working demonstration: a humanoid robot that listens to voice commands, plans appropriate actions, and executes safe pick-and-place operations in Gazebo simulation.

**System Capabilities:**
- **Voice Input**: Process natural language commands (e.g., "Pick up the red block")
- **Scene Understanding**: Detect and localize objects using computer vision
- **Task Planning**: Generate action sequences using behavior trees
- **Safe Execution**: Enforce joint limits, collision avoidance, and error recovery
- **Feedback**: Provide status updates and handle failures gracefully

**Technologies Used:**
- **ROS 2 Humble** (middleware and communication)
- **Gazebo Classic/Harmonic** (physics simulation)
- **Whisper** (speech recognition, from Module 3)
- **GPT-4 / local LLM** (natural language understanding)
- **YOLO** (object detection, from Module 3)
- **BehaviorTree.CPP** (task execution)
- **MoveIt** (motion planning, optional)

**Expected Outcome:** A user can launch the simulation, issue a voice command like "Pick up the blue cube and place it on the table," and observe the humanoid autonomously complete the task.

## Project Architecture

### System Components

```
┌──────────────────┐
│ User Voice       │
│ Command          │
└────────┬─────────┘
         │
         ▼
┌────────────────────────────────┐
│ Speech Recognition Node        │
│ (Whisper)                      │
│ ┌────────────────────────────┐ │
│ │ Audio → Text Transcription │ │
│ └────────────────────────────┘ │
└────────┬───────────────────────┘
         │ /voice_command (String)
         ▼
┌────────────────────────────────┐
│ VLA Control Node               │
│ (LLM + Vision Integration)     │
│ ┌────────────────────────────┐ │
│ │ 1. Parse command (LLM)     │ │
│ │ 2. Identify objects (YOLO) │ │
│ │ 3. Generate action plan    │ │
│ └────────────────────────────┘ │
└────────┬───────────────────────┘
         │ /task_goal (Action)
         ▼
┌────────────────────────────────┐
│ Behavior Tree Executor         │
│ (BT.CPP)                       │
│ ┌────────────────────────────┐ │
│ │ Sequence:                  │ │
│ │ ├─ Approach Object         │ │
│ │ ├─ Grasp                   │ │
│ │ ├─ Lift                    │ │
│ │ ├─ Navigate to Target      │ │
│ │ └─ Release                 │ │
│ └────────────────────────────┘ │
└────────┬───────────────────────┘
         │ /joint_commands
         ▼
┌────────────────────────────────┐
│ Safety Monitor                 │
│ ┌────────────────────────────┐ │
│ │ - Joint limit checks       │ │
│ │ - Collision detection      │ │
│ │ - Emergency stop handler   │ │
│ └────────────────────────────┘ │
└────────┬───────────────────────┘
         │ (validated commands)
         ▼
┌────────────────────────────────┐
│ Gazebo Simulation              │
│ (Humanoid + Environment)       │
└────────────────────────────────┘
```

## Implementation Steps

### Step 1: Environment Setup

**1.1 Create Gazebo World**

Define a simple environment with a table and objects.

**File: `worlds/pick_place_world.world`**
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="pick_place_world">
    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Table -->
    <model name="table">
      <pose>1.0 0 0.4 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="surface">
          <geometry>
            <box>
              <size>1.0 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.05</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Red Block -->
    <model name="red_block">
      <pose>0.8 0.2 0.45 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.1</mass>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Blue Cube -->
    <model name="blue_cube">
      <pose>1.2 -0.1 0.45 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.1</mass>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.06 0.06 0.06</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.06 0.06 0.06</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**1.2 Launch Simulation**

**File: `launch/pick_place_sim.launch.py`**
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('humanoid_control'),
        'worlds',
        'pick_place_world.world'
    )

    return LaunchDescription([
        # Launch Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_file}.items()
        ),

        # Spawn humanoid robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-file', '/path/to/humanoid.urdf']
        ),

        # Camera node for object detection
        Node(
            package='image_tools',
            executable='cam2image',
            name='camera_publisher'
        ),
    ])
```

### Step 2: Speech Recognition Integration

Reuse Whisper integration from Module 3.

**File: `scripts/speech_to_command_node.py`**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import tempfile
import wave

class SpeechToCommandNode(Node):
    def __init__(self):
        super().__init__('speech_to_command')
        self.publisher = self.create_publisher(String, '/voice_command', 10)

        # Load Whisper model
        self.model = whisper.load_model("base")
        self.get_logger().info('Whisper model loaded. Ready for voice commands.')

        # Start listening loop
        self.timer = self.create_timer(5.0, self.listen_and_transcribe)

    def listen_and_transcribe(self):
        """Record audio and transcribe to text"""
        self.get_logger().info('Listening for command... (speak now)')

        # Record 5 seconds of audio
        duration = 5  # seconds
        fs = 16000  # sample rate
        recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
        sd.wait()

        # Save to temporary WAV file
        temp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
        with wave.open(temp_wav.name, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(fs)
            wf.writeframes(recording.tobytes())

        # Transcribe with Whisper
        result = self.model.transcribe(temp_wav.name)
        command_text = result['text'].strip()

        if command_text:
            self.get_logger().info(f'Transcribed: "{command_text}"')
            msg = String()
            msg.data = command_text
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: VLA Control Node

Combine vision (YOLO) and language (LLM) to generate action plans.

**File: `scripts/vla_control_node.py`**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import openai
import json

class VLAControlNode(Node):
    def __init__(self):
        super().__init__('vla_control_node')

        # Subscribers
        self.voice_sub = self.create_subscription(String, '/voice_command', self.voice_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.action_pub = self.create_publisher(String, '/task_action', 10)

        # Vision model
        self.yolo_model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.latest_image = None

        # LLM API key (load from environment)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        self.get_logger().info('VLA Control Node ready')

    def image_callback(self, msg):
        """Store latest camera frame"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def voice_callback(self, msg):
        """Process voice command through VLA pipeline"""
        command = msg.data
        self.get_logger().info(f'Processing command: "{command}"')

        if self.latest_image is None:
            self.get_logger().warn('No camera image available yet')
            return

        # Step 1: Detect objects in scene
        scene_objects = self.detect_objects(self.latest_image)

        # Step 2: Parse command with LLM
        action_plan = self.parse_command(command, scene_objects)

        # Step 3: Validate safety
        if self.is_safe_action(action_plan):
            self.publish_action(action_plan)
        else:
            self.get_logger().error(f'Unsafe action rejected: {action_plan}')

    def detect_objects(self, image):
        """Use YOLO to detect objects"""
        results = self.yolo_model(image)
        detected = []

        for detection in results[0].boxes:
            class_name = self.yolo_model.names[int(detection.cls)]
            confidence = float(detection.conf)
            bbox = detection.xyxy[0].tolist()

            # Extract color (simplified: check dominant color in bbox region)
            roi = image[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])]
            color = self.get_dominant_color(roi)

            detected.append({
                'name': class_name,
                'color': color,
                'bbox': bbox,
                'confidence': confidence
            })

        return detected

    def get_dominant_color(self, roi):
        """Determine dominant color (red, blue, green, etc.)"""
        # Simplified: average RGB and classify
        avg_color = roi.mean(axis=(0, 1))
        r, g, b = avg_color

        if r > g and r > b:
            return 'red'
        elif b > r and b > g:
            return 'blue'
        elif g > r and g > b:
            return 'green'
        else:
            return 'unknown'

    def parse_command(self, command, scene_objects):
        """Use LLM to parse command into action plan"""
        scene_desc = ', '.join([f"{obj['color']} {obj['name']}" for obj in scene_objects])

        prompt = f"""
Scene objects: {scene_desc}
Command: "{command}"

Parse into JSON action plan:
{{
  "action": "pick_and_place",
  "target_object": "color + object name",
  "target_location": "destination",
  "constraints": ["gentle_grasp"]
}}

If command is ambiguous or unsafe, return: {{"action": "clarify", "reason": "..."}}
"""
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0
        )

        return json.loads(response.choices[0].message.content)

    def is_safe_action(self, action_plan):
        """Validate action safety"""
        if action_plan.get('action') == 'clarify':
            return False  # Ambiguous command

        # Check for unsafe keywords
        unsafe_keywords = ['throw', 'smash', 'break']
        action_str = json.dumps(action_plan).lower()

        return not any(keyword in action_str for keyword in unsafe_keywords)

    def publish_action(self, action_plan):
        """Send action plan to behavior tree executor"""
        msg = String()
        msg.data = json.dumps(action_plan)
        self.action_pub.publish(msg)
        self.get_logger().info(f'Action published: {action_plan}')

def main(args=None):
    rclpy.init(args=args)
    node = VLAControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Behavior Tree for Pick-and-Place

**File: `config/pick_place_behavior_tree.xml`**
```xml
<root main_tree_to_execute="PickAndPlace">
  <BehaviorTree ID="PickAndPlace">
    <Sequence name="MainSequence">
      <!-- Safety check before starting -->
      <Condition ID="IsBatteryOK"/>
      <Condition ID="IsWorkspaceClear"/>

      <!-- Execute pick-and-place -->
      <Sequence name="PickObject">
        <Action ID="MoveToObject" target="{target_object}"/>
        <Action ID="AlignGripper" target="{target_object}"/>
        <RetryNode num_attempts="3">
          <Action ID="CloseGripper"/>
        </RetryNode>
        <Condition ID="IsGrasping"/>
        <Action ID="LiftObject"/>
      </Sequence>

      <Sequence name="PlaceObject">
        <Action ID="NavigateToLocation" target="{target_location}"/>
        <Action ID="PositionAboveTarget"/>
        <Action ID="LowerObject"/>
        <Action ID="OpenGripper"/>
        <Action ID="RetractArm"/>
      </Sequence>

      <!-- Confirmation -->
      <Action ID="SayTaskComplete"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Step 4.1: Implement Custom Action Nodes**

**File: `src/custom_actions.cpp`** (simplified example)
```cpp
#include <behaviortree_cpp_v3/action_node.h>

class MoveToObject : public BT::SyncActionNode {
public:
    MoveToObject(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("target") };
    }

    BT::NodeStatus tick() override {
        auto target = getInput<std::string>("target");
        if (!target) {
            return BT::NodeStatus::FAILURE;
        }

        // TODO: Call motion planner to move arm to target object
        ROS_INFO("Moving to object: %s", target.value().c_str());

        // Simulate success
        return BT::NodeStatus::SUCCESS;
    }
};

// Register node
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<MoveToObject>("MoveToObject");
}
```

### Step 5: Safety Monitor

**File: `scripts/safety_monitor_node.py`**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # Publish emergency stop signal
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Joint limits (degrees)
        self.joint_limits = {
            'shoulder_pitch': (-90, 180),
            'elbow_pitch': (0, 150),
        }

    def joint_callback(self, msg):
        """Check joint positions against limits"""
        for i, name in enumerate(msg.name):
            if name in self.joint_limits:
                position_deg = msg.position[i] * 180 / 3.14159

                min_limit, max_limit = self.joint_limits[name]
                if position_deg < min_limit or position_deg > max_limit:
                    self.get_logger().error(
                        f'Joint limit violation: {name} = {position_deg:.1f}° (limits: {min_limit}, {max_limit})')
                    self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        """Activate emergency stop"""
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### Test Plan

**Test 1: Command Parsing**
- Input: "Pick up the red block"
- Expected: VLA node outputs `{"action": "pick_and_place", "target_object": "red block", ...}`

**Test 2: Object Detection**
- Launch Gazebo world with red and blue blocks
- Verify YOLO detects both objects with correct colors

**Test 3: Behavior Tree Execution**
- Manually trigger behavior tree with test action plan
- Verify sequence: Approach → Grasp → Lift → Navigate → Release

**Test 4: Safety Constraints**
- Command robot to move arm beyond joint limit
- Verify safety monitor rejects command and logs error

**Test 5: End-to-End Integration**
- Issue voice command: "Place the blue cube on the table"
- Observe complete autonomous execution

### Success Criteria

- [ ] Speech transcription accuracy > 90% (clear commands)
- [ ] Object detection correctly identifies color + class
- [ ] LLM parses commands into valid action plans
- [ ] Behavior tree executes pick-and-place without errors
- [ ] Safety monitor prevents all joint limit violations
- [ ] System recovers gracefully from grasp failures (retry logic)
- [ ] End-to-end latency < 10 seconds (command to execution start)

## Troubleshooting Common Issues

**Issue 1: No objects detected by YOLO**
- **Cause**: Camera not publishing images or YOLO model mismatch
- **Solution**: Check `/camera/image_raw` topic with `ros2 topic echo`, verify YOLO model loaded correctly

**Issue 2: LLM returns "clarify" for all commands**
- **Cause**: Scene description empty or LLM API key invalid
- **Solution**: Verify object detection outputs non-empty list, check `OPENAI_API_KEY` environment variable

**Issue 3: Behavior tree stuck in RUNNING state**
- **Cause**: Action node blocking without returning status
- **Solution**: Add timeout decorators to all actions, ensure nodes return SUCCESS/FAILURE

**Issue 4: Robot moves too fast, violates safety**
- **Cause**: Velocity limits not enforced
- **Solution**: Add velocity clipping in joint controller (see Chapter 6)

## Extensions and Improvements

**Extension 1: Multi-Object Tasks**
- Command: "Stack the red block on the blue cube"
- Requires: Sequential pick-and-place with intermediate placement

**Extension 2: Dynamic Replanning**
- If grasp fails, ask LLM for alternative approach
- Example: "Grasp from side instead of top"

**Extension 3: Human Confirmation**
- After parsing command, ask user: "I will pick up the red block. Proceed? (yes/no)"
- Wait for verbal confirmation before executing

**Extension 4: Real Robot Deployment**
- Replace Gazebo with real humanoid hardware
- Integrate force/torque sensors for grasp verification
- Add camera calibration for accurate object pose estimation

## Summary

This mini-project demonstrates a complete voice-controlled pick-and-place system integrating:
- **Module 3** (Perception): Speech recognition (Whisper), Object detection (YOLO)
- **Module 4** (Control): Task planning (LLM), Execution (Behavior Trees), Safety (Constraint monitoring)

**Key Achievements:**
- End-to-end pipeline from voice command to physical action
- Modular architecture enabling easy component replacement
- Safety-first design with multiple validation layers
- Real-world applicability (extendable to hardware)

**Next Steps:**
- Proceed to **Capstone Project** (Module 5) to integrate digital twin simulation and advanced perception
- Explore multi-modal control (voice + gestures)
- Deploy on physical humanoid platform

## Final Checklist

Before considering this project complete, verify:
- [ ] All code files created and organized in ROS 2 package
- [ ] Launch files tested and documented
- [ ] README with setup instructions written
- [ ] Video demo recorded (minimum 2 minutes showing successful execution)
- [ ] Code commented and follows ROS 2 conventions
- [ ] Safety mechanisms tested and validated
- [ ] Project presentation prepared (5-10 slides summarizing architecture and results)

## References

ROS 2 Documentation. (2024). Writing Action Servers and Clients (Python). Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

BehaviorTree.CPP Examples. (2024). Retrieved from https://github.com/BehaviorTree/BehaviorTree.CPP/tree/master/examples

OpenAI API Documentation. (2024). Chat Completions. Retrieved from https://platform.openai.com/docs/guides/text-generation

Gazebo Tutorials. (2024). Build a World. Retrieved from https://classic.gazebosim.org/tutorials?tut=build_world

Whisper GitHub Repository. (2024). OpenAI Whisper. Retrieved from https://github.com/openai/whisper
