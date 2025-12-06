---
title: "Humanoid Robot URDF Models"
description: "Define robot structure using URDF, visualize models in RViz2, and understand coordinate transformations"
keywords: ["ros2", "urdf", "robot model", "rviz", "tf2", "coordinate frames"]
sidebar_position: 5
learning_objectives:
  - "Understand URDF syntax for defining robot structure"
  - "Create a simple humanoid robot model with joints and links"
  - "Visualize robot models in RViz2"
  - "Comprehend coordinate frame transformations (TF2)"
---

# Humanoid Robot URDF Models

## Introduction

Up to this point, you've been working with abstract nodes exchanging messages. But robots are physical entities with geometry, mass, joints, and sensors. How do you describe a robot's structure in ROS 2?

This chapter introduces **URDF (Unified Robot Description Format)**—an XML-based language for defining robot kinematic and dynamic properties. You'll learn to create humanoid robot models, visualize them in RViz2, and understand how ROS 2 manages coordinate transformations between robot components.

By the end of this chapter, you'll be able to model robot structure, essential for simulation, motion planning, and sensor integration in subsequent modules.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand URDF syntax for defining robot structure
- Create a simple humanoid robot model with joints and links
- Visualize robot models in RViz2
- Comprehend coordinate frame transformations (TF2)

## Prerequisites

- ROS 2 Humble with RViz2 installed (Chapter 2)
- Understanding of nodes and topics (Chapters 3-4)
- Basic knowledge of 3D coordinates (X, Y, Z axes)
- Familiarity with XML syntax

---

## What is URDF?

### Unified Robot Description Format

**URDF** is an XML specification for describing:
- **Links**: Rigid bodies (robot parts like torso, arms, legs)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Kinematic chain**: Parent-child relationships forming robot structure
- **Visual properties**: Geometry and colors for visualization
- **Collision properties**: Simplified geometry for collision detection
- **Inertial properties**: Mass and inertia for physics simulation

URDF files are standard across ROS 1 and ROS 2, enabling model reuse.

### Why URDF Matters

Without a formal robot description:
- Sensors don't know their position relative to the robot base
- Motion planners can't compute valid joint configurations
- Simulators don't know what geometry to render
- Coordinate transformations between components are manual and error-prone

URDF provides a **single source of truth** for robot geometry that all ROS 2 tools consume.

---

## URDF Basics: Links and Joints

### Links

A **link** represents a rigid body in the robot:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.4 0.3 0.6"/>  <!-- Width, Depth, Height in meters -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>  <!-- R G B Alpha -->
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.4 0.3 0.6"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="50.0"/>  <!-- kg -->
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

**Components:**
- `<visual>`: How the link appears in visualization
- `<collision>`: Simplified geometry for collision checking
- `<inertial>`: Mass and inertia for physics simulation

### Joints

A **joint** connects two links with a specific motion type:

```xml
<joint name="torso_to_left_shoulder" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.0 0.2 0.25" rpy="0 0 0"/>  <!-- Position relative to parent -->
  <axis xyz="0 1 0"/>  <!-- Rotation axis (Y-axis) -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

**Joint types:**
- `revolute`: Rotates around an axis with limits
- `continuous`: Rotates around an axis without limits (wheels)
- `prismatic`: Slides along an axis (telescope, elevator)
- `fixed`: No motion (connects rigidly)
- `floating`: 6 degrees of freedom (rarely used)
- `planar`: Moves in a plane

---

## Simple Humanoid Model Example

### Minimal Humanoid URDF

Create `~/ros2_ws/src/my_robot_controller/urdf/simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link -->
  <link name="base_link"/>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="1.0 0.8 0.6 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Z-axis rotation (yaw) -->
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.2 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Right Arm (symmetric) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.2 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

</robot>
```

**Model structure:**
- `base_link`: Root of kinematic tree
- `torso`: Main body (box geometry)
- `head`: Sphere attached via revolute neck joint
- `left_upper_arm`, `right_upper_arm`: Cylinders for arms

:::tip
**Best Practice**: Always start with a `base_link` as the root. Use descriptive link and joint names. Keep visual geometry simple initially—complexity can be added later.
:::

---

## Visualizing in RViz2

### Launching RViz with URDF

#### Method 1: Using robot_state_publisher

Create launch file `~/ros2_ws/src/my_robot_controller/launch/display_humanoid.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_controller'),
        'urdf',
        'simple_humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # robot_state_publisher: publishes robot TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui: control joints interactively
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
```

**Install required packages:**
```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

**Update CMakeLists.txt or setup.py** to install URDF and launch files (details in package documentation).

**Launch:**
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 launch my_robot_controller display_humanoid.launch.py
```

### Configuring RViz

When RViz opens:
1. **Fixed Frame**: Set to `base_link` (in Global Options)
2. **Add RobotModel display**:
   - Click "Add" button
   - Select "RobotModel"
   - Topic: `/robot_description` (auto-populated)
3. **Adjust view**: Use mouse to rotate, zoom

You should see your humanoid model! Use the Joint State Publisher GUI sliders to move joints.

:::note
If the model doesn't appear, check:
- Fixed Frame matches a link in your URDF
- RobotModel display is enabled (checkbox)
- `/robot_description` topic is being published (`ros2 topic list`)
:::

---

## Coordinate Frames and TF2

### Understanding TF2

**TF2 (Transform Framework 2)** manages coordinate transformations between robot components:
- Every link in URDF has an associated coordinate frame
- Joints define transformations between frames
- `robot_state_publisher` broadcasts these transformations as TF messages
- Any node can query transforms (e.g., "What is the camera's position relative to the base?")

### Viewing TF Tree

```bash
# Install TF tools
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# Generates frames.pdf showing transform hierarchy
evince frames.pdf
```

### Querying Transforms

```bash
# Echo transform from base_link to head
ros2 run tf2_ros tf2_echo base_link head

# Output shows translation (xyz) and rotation (quaternion)
```

### Why TF2 Matters

Example use cases:
- **Perception**: Camera sees object at (x, y, z) in camera frame; TF transforms to base frame
- **Motion Planning**: Planner needs gripper position relative to target object
- **Sensor Fusion**: Combine data from sensors at different positions on robot

---

## Exercises

### Exercise 1: Add Legs to Humanoid

**Objective**: Extend the URDF model with lower body

**Instructions**:
1. Add `left_upper_leg` and `right_upper_leg` links (cylinders)
2. Create `left_hip` and `right_hip` joints (revolute, attach to torso)
3. Rebuild and visualize in RViz

**Expected Outcome**: Humanoid has moveable legs

### Exercise 2: Modify Joint Limits

**Objective**: Understand joint constraints

**Instructions**:
1. Change neck joint limits to `lower="-1.57" upper="1.57"` (±90°)
2. Rebuild and test with Joint State Publisher GUI
3. Observe head rotation range increases

**Expected Outcome**: Neck joint has wider range of motion

### Exercise 3: Add Sensor to Model

**Objective**: Include sensor in URDF

**Instructions**:
1. Create `camera` link (small box) as child of `head`
2. Use fixed joint to attach camera facing forward
3. Visualize in RViz and query TF transform from base to camera

**Expected Outcome**: Camera appears on head; can query its transform

### Exercise 4: Multi-Link Arm

**Objective**: Create kinematic chain

**Instructions**:
1. Add `left_lower_arm` link (cylinder)
2. Create `left_elbow` joint connecting upper and lower arm
3. Add `left_hand` link
4. Create `left_wrist` joint

**Expected Outcome**: Left arm has 3 links and 3 joints forming a chain

### Exercise 5: Visualize Custom Geometry

**Objective**: Use mesh files for visual representation

**Instructions**:
1. Download a simple .stl or .dae mesh file
2. Replace `<geometry><box/></geometry>` with `<geometry><mesh filename="path/to/file.stl"/></geometry>`
3. Rebuild and visualize

**Expected Outcome**: Custom 3D model appears in RViz

---

## Summary

Key takeaways from this chapter:

- **URDF Purpose**: XML format describing robot kinematic structure, geometry, and physical properties—the single source of truth for robot models
- **Links and Joints**: Links are rigid bodies; joints connect links with defined motion types (revolute, prismatic, fixed)
- **Visualization**: `robot_state_publisher` publishes TF transforms; RViz2 displays robot models; joint_state_publisher_gui enables interactive control
- **TF2 Framework**: Manages coordinate transformations between robot components, enabling spatial queries essential for perception and planning
- **Best Practices**: Start with base_link root, use descriptive names, keep initial geometry simple, validate in RViz frequently

You can now define robot structure using URDF, visualize models in RViz2, and understand coordinate frame transformations. These skills are foundational for simulation (Module 2), perception (Module 3), and motion control (Module 4).

---

## What's Next?

In [Chapter 6: Mini-Project - ROS 2 Communication](./mini-project-ros2-communication), you'll integrate everything learned—nodes, topics, URDF—into a hands-on project where you build a complete voice-controlled robot communication system.

---

## References

1. Open Robotics. (2023). *URDF tutorials*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

2. Open Robotics. (2023). *URDF XML specification*. Retrieved from http://wiki.ros.org/urdf/XML

3. Open Robotics. (2023). *Using URDF with robot_state_publisher*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

4. Foote, T. (2013). tf: The transform library. *IEEE International Conference on Technologies for Practical Robot Applications (TePRA)*, 1-6. https://doi.org/10.1109/TePRA.2013.6556373

5. Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics* (2nd ed.). Springer. https://doi.org/10.1007/978-3-319-32552-1

---

## Additional Resources

- [URDF Visual Guide](https://wiki.ros.org/urdf/Tutorials) - Step-by-step URDF creation
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide) - Comprehensive RViz documentation
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) - Transform framework deep dive
- [Xacro (XML Macros)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) - Advanced URDF with macros

---

**Word Count**: ~1,900 words
**Reading Level**: Flesch-Kincaid Grade 9.0
**Last Updated**: 2025-12-05
**Code Validation Status**: ⏳ Pending validation (ROS 2 Humble, RViz2)
