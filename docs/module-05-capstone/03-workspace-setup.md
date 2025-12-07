# Workspace Setup and Environment Configuration

## Overview

This chapter guides you through setting up a complete development environment for the capstone project. By the end, you'll have a functional workspace with ROS 2, Gazebo, perception libraries, and all dependencies properly configured.

## Prerequisites

Before starting, ensure you have:
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- 20 GB free disk space
- Stable internet connection for downloading dependencies
- Basic command-line familiarity

## Step 1: Install ROS 2 Humble

### Option A: Binary Installation (Recommended)

```bash
# Setup locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

### Verify Installation

```bash
source /opt/ros/humble/setup.bash
ros2 --version
# Expected output: ros2 doctor 0.23.0 (or similar)
```

### Add to .bashrc (Auto-source ROS 2)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Create ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build empty workspace
colcon build
source install/setup.bash

# Add workspace to .bashrc
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
```

## Step 3: Install Gazebo

### For ROS 2 Humble (Gazebo Classic 11)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y
```

### Verify Gazebo Installation

```bash
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### Test Gazebo Launch

```bash
gazebo --verbose
# Should open Gazebo GUI without errors. Press Ctrl+C to exit.
```

## Step 4: Install ROS 2 Control Packages

```bash
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y
sudo apt install ros-humble-controller-manager -y
sudo apt install ros-humble-joint-state-publisher -y
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt install ros-humble-robot-state-publisher -y
```

## Step 5: Install Perception Dependencies

### OpenCV

```bash
sudo apt install python3-opencv -y
pip3 install opencv-contrib-python
```

### OpenAI Whisper (Speech Recognition)

```bash
pip3 install openai-whisper
# Or for faster inference:
pip3 install faster-whisper
```

### YOLO for Object Detection (Optional but Recommended)

```bash
pip3 install ultralytics
```

### Additional Python Libraries

```bash
pip3 install numpy scipy matplotlib
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118  # CUDA 11.8
# Or CPU-only version:
# pip3 install torch torchvision torchaudio
```

## Step 6: Install Behavior Tree Library

```bash
sudo apt install ros-humble-behaviortree-cpp-v3 -y
pip3 install py-trees py-trees-ros
```

## Step 7: Install MoveIt 2 (Motion Planning)

```bash
sudo apt install ros-humble-moveit -y
```

## Step 8: Create Humanoid Assistant Package

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_assistant \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs trajectory_msgs vision_msgs
```

### Package Structure

```bash
cd humanoid_assistant
mkdir -p \
  humanoid_assistant/perception \
  humanoid_assistant/planning \
  humanoid_assistant/control \
  humanoid_assistant/integration \
  config \
  launch \
  urdf \
  worlds \
  models
```

## Step 9: Configure URDF and Gazebo World

### Copy Example Humanoid URDF

Create a simple humanoid URDF for testing:

```bash
cat > urdf/simple_humanoid.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Add camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate> 30.0</update_rate>
      <camera name="head_camera">
        <horizontal_fov> 1.3962634</horizontal_fov>
        <image>
          <width> 640</width>
          <height> 480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near> 0.02</near>
          <far> 300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate> 0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
EOF
```

### Create Simple Gazebo World

```bash
cat > worlds/simple_world.world << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Red block for testing -->
    <model name="red_block">
      <static>false</static>
      <pose> 1 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size> 0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size> 0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient> 1 0 0 1</ambient>
            <diffuse> 1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF
```

## Step 10: Create Launch File

```bash
cat > launch/test_simulation.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_assistant')
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_humanoid.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_humanoid', '-topic', '/robot_description'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Joint state publisher (for manual control in testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
    ])
EOF
```

## Step 11: Build and Test

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_assistant
source install/setup.bash

# Test launch
ros2 launch humanoid_assistant test_simulation.launch.py
```

Expected behavior:
- Gazebo opens with a simple humanoid robot
- Joint state publisher GUI appears
- You can move the neck joint and see the robot respond

## Step 12: Install Additional Tools

### RQT Tools (Debugging and Visualization)

```bash
sudo apt install ros-humble-rqt -y
sudo apt install ros-humble-rqt-common-plugins -y
```

### TF2 Tools (Transform Debugging)

```bash
sudo apt install ros-humble-tf2-tools -y
```

### PlotJuggler (Data Visualization)

```bash
sudo apt install ros-humble-plotjuggler-ros -y
```

## Environment Variables and Configuration

### Create .env File for API Keys (If Using Cloud LLMs)

```bash
cat > ~/humanoid_ws/.env << 'EOF'
OPENAI_API_KEY=your_key_here
# Add other API keys as needed
EOF

# Add to .bashrc to load environment variables
echo "export $(cat ~/humanoid_ws/.env | xargs)" >> ~/.bashrc
```

### ROS 2 Domain ID (For Multi-Machine Setups)

```bash
# Add to .bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

## Workspace Validation Checklist

Verify your setup by running these tests:

```bash
# Test 1: ROS 2 installation
ros2 doctor --report

# Test 2: Check installed packages
ros2 pkg list | grep humanoid_assistant

# Test 3: Verify Gazebo integration
ros2 pkg list | grep gazebo_ros

# Test 4: Check Python libraries
python3 -c "import cv2; import whisper; print('Perception libraries OK')"

# Test 5: List available topics (after launching simulation)
ros2 topic list

# Test 6: Visualize node graph
rqt_graph
```

## Troubleshooting Common Setup Issues

### Issue 1: "Package 'ros-humble-...' has no installation candidate"

**Solution**: Update package lists
```bash
sudo apt update
```

### Issue 2: Gazebo crashes with "libGL error"

**Solution**: Install GPU drivers or use software rendering
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Issue 3: "Module 'cv2' not found"

**Solution**: Reinstall OpenCV for Python
```bash
pip3 uninstall opencv-python opencv-contrib-python
pip3 install opencv-contrib-python
```

### Issue 4: colcon build fails with "No module named 'setuptools'"

**Solution**: Install setuptools
```bash
pip3 install setuptools
```

## Next Steps

With your workspace configured, you're ready to:
- **Chapter 4**: Implement perception integration (vision + speech)
- **Chapter 5**: Build planning and control pipelines
- **Chapter 6**: Complete system implementation

Keep this chapter as a reference—you'll often need to reinstall dependencies or troubleshoot environment issues during development.

---

## Exercises

### Exercise 1: Customize Your Robot
Modify `simple_humanoid.urdf` to add arms with shoulder and elbow joints. Update the launch file and test in Gazebo.

### Exercise 2: Add More Objects to the World
Edit `simple_world.world` to include a table and multiple colored blocks.

### Exercise 3: Verify Transform Tree
Launch your simulation and run:
```bash
ros2 run tf2_tools view_frames.py
```
Open the generated PDF and verify all transforms are connected.

### Exercise 4: Camera Verification
Launch the simulation and check camera output:
```bash
ros2 topic echo /camera/image_raw --no-arr
```
Verify images are being published at ~30 Hz.

### Exercise 5: Write a Diagnostic Script
Create a bash script that runs all validation checks and reports setup status.
