# Demo and Deployment Guide

## Overview

This chapter provides a comprehensive guide for testing, recording a demonstration video, and deploying your autonomous voice-controlled humanoid assistant.

## Testing Your System

### Pre-Deployment Testing Checklist

Before recording your demo or deploying, verify:

#### 1. **ROS 2 Node Health**
```bash
# Launch your system
ros2 launch humanoid_assistant full_system.launch.py

# In separate terminals, verify:
ros2 node list  # All nodes running?
ros2 topic list  # All topics present?
ros2 topic hz /camera/image_raw  # ~30 Hz?
ros2 topic hz /joint_states  # ~100 Hz?
```

#### 2. **Perception System Validation**
```bash
# Test vision
ros2 topic echo /detected_objects --no-arr

# Test speech recognition (speak a command)
ros2 topic echo /voice_commands
```

#### 3. **Planning and Control Validation**
```bash
# Manually trigger a simple command
ros2 topic pub /voice_commands std_msgs/String "data: 'wave'" --once

# Verify robot executes motion
# Check safety constraints (no collisions)
```

### End-to-End Test Scenarios

#### Scenario 1: Object Manipulation
**Setup**: Place red block in simulation at (1, 0, 0.5)

**Test**:
```bash
# Speak command: "Pick up the red block"
# Expected behavior:
# 1. Vision detects red block
# 2. Planner generates grasp trajectory
# 3. Robot moves to block
# 4. Gripper closes
# 5. Robot lifts block
```

**Success Criteria**:
- Object detected with greater than 85% confidence
- Trajectory collision-free
- Grasp successful (block moves with hand)
- Total time less than 30 seconds

#### Scenario 2: Multi-Step Task
**Test**: "Pick up the block and place it on the table"

**Success Criteria**:
- Both subtasks (pick, place) execute correctly
- Robot transitions between subtasks smoothly
- Final block position matches target

#### Scenario 3: Error Handling
**Test**: "Pick up the blue block" (no blue block present)

**Success Criteria**:
- Vision reports no blue block detected
- System provides voice feedback: "I don't see a blue block"
- No attempt to execute invalid motion

## Recording Your Demonstration Video

### Video Requirements

- **Duration**: 3-5 minutes
- **Format**: MP4, 1080p (1920x1080), 30 fps
- **Content**: Show end-to-end capabilities with 3-5 different commands
- **Narration**: Explain what's happening (optional but recommended)

### Recommended Tools

**Linux Screen Recording**:
```bash
# Install SimpleScreenRecorder
sudo apt install simplescreenrecorder

# Or use OBS Studio
sudo apt install obs-studio
```

### Video Structure

**Segment 1: Introduction (30 seconds)**
- Show system architecture diagram
- Briefly explain capabilities
- State objectives

**Segment 2: Demonstration (3-4 minutes)**
- **Command 1**: Simple gesture (e.g., "wave")
- **Command 2**: Object manipulation (e.g., "pick up red block")
- **Command 3**: Multi-step task (e.g., "pick and place")
- **Command 4**: Error handling (e.g., invalid command)
- **Command 5**: Complex scenario (optional)

**Segment 3: Conclusion (30 seconds)**
- Summarize achievements
- Mention future improvements

### Recording Tips

1. **Clean Background**: Close unnecessary windows, use fullscreen Gazebo
2. **Clear Audio**: Use a decent microphone if adding narration
3. **Show ROS Topics**: Picture-in-picture with `rqt_graph` or topic echoes
4. **Highlight Key Moments**: Slow down or replay successful grasps
5. **Edit Failures**: Cut out failed attempts or show debugging process

### Example Narration Script

```
"This is my autonomous voice-controlled humanoid assistant built with ROS 2.

First, let me demonstrate a simple gesture.
[Speak: 'wave']
As you can see, the speech recognition detected the command, the planner generated a wave trajectory, and the robot executed the motion smoothly.

Next, I'll show object manipulation.
[Speak: 'pick up the red block']
The vision system identified the red block, calculated its position, and the planner generated a collision-free grasp trajectory. The robot successfully picked up the block.

Finally, here's error handling.
[Speak: 'pick up the blue block']
Since no blue block exists, the system correctly reports the error and doesn't attempt an invalid action."
```

## Deployment Options

### Option 1: GitHub Pages (Static Documentation Only)

If you're only deploying documentation (not running the robot remotely):

```bash
# Build Docusaurus site
cd ~/humanoid_ws/docs
npm run build

# Deploy to GitHub Pages
git add .
git commit -m "Deploy capstone project documentation"
git push origin main
```

### Option 2: Docker Container (Full System)

Package your entire system for reproducible deployment:

**Dockerfile**:
```dockerfile
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-behaviortree-cpp-v3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-contrib-python openai-whisper ultralytics

# Copy workspace
COPY humanoid_ws /root/humanoid_ws

# Build workspace
WORKDIR /root/humanoid_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Setup entry point
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/humanoid_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
```

**Build and run**:
```bash
docker build -t humanoid_assistant .
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  humanoid_assistant
```

### Option 3: Cloud Deployment (AWS, GCP, Azure)

For remote access or scaling:

**AWS EC2 Setup**:
1. Launch EC2 instance (Ubuntu 22.04, g4dn.xlarge for GPU)
2. Install ROS 2 and dependencies
3. Clone your repository
4. Use VNC or X11 forwarding for GUI access

**Example X11 forwarding**:
```bash
ssh -X ubuntu@<ec2-ip-address>
cd ~/humanoid_ws
ros2 launch humanoid_assistant full_system.launch.py
```

## Documentation Deliverables

### 1. README.md

Create a comprehensive README in your repository root:

```markdown
# Autonomous Voice-Controlled Humanoid Assistant

## Overview
This project implements a fully autonomous humanoid robot capable of understanding voice commands, detecting objects, and executing complex manipulation tasks.

## Features
- Natural language voice command recognition
- Real-time object detection and tracking
- Task planning with behavior trees
- Safe motion control with collision avoidance
- Gazebo simulation environment

## Demo Video
[Link to YouTube demo]

## Installation
[Step-by-step setup instructions]

## Usage
```bash
ros2 launch humanoid_assistant full_system.launch.py
```

## Architecture
[Include architecture diagram]

## Supported Commands
- "wave"
- "pick up [color] block"
- "place on table"
- "navigate to [location]"

## Testing
```bash
ros2 launch humanoid_assistant test_suite.launch.py
```

## Contributing
[Link to CONTRIBUTING.md]

## License
MIT
```

### 2. System Architecture Diagram

Use tools like draw.io or Mermaid to create:
- Node graph showing all ROS 2 nodes
- Data flow diagram (sensors → perception → planning → control)
- State machine diagram

### 3. API Documentation

Document all custom ROS 2 messages and services:

```markdown
## Custom Messages

### VoiceCommand.msg
```
std_msgs/Header header
string command
float32 confidence
```

### DetectedObject.msg
```
string object_class
string color
geometry_msgs/Point position
float32 confidence
```
```

## Performance Benchmarking

### Metrics to Report

1. **Latency**:
   - Voice command to motion start: less than 2 seconds
   - Object detection frequency: 30 Hz
   - Control loop rate: 100 Hz

2. **Accuracy**:
   - Speech recognition: greater than 90% for trained commands
   - Object detection: greater than 85% recall, greater than 80% precision
   - Grasp success rate: greater than 75%

3. **Robustness**:
   - System uptime: greater than 95% (no crashes during 10-minute test)
   - Error recovery success: greater than 90%

### Benchmark Script

```python
#!/usr/bin/env python3
import rclpy
import time
from std_msgs.msg import String

def benchmark_latency():
    node = rclpy.create_node('benchmark')
    pub = node.create_publisher(String, '/voice_commands', 10)

    start_time = time.time()
    msg = String()
    msg.data = "wave"
    pub.publish(msg)

    # Wait for motion to start (monitor /joint_states)
    # Calculate latency

    latency = time.time() - start_time
    print(f"Command-to-motion latency: {latency:.2f}s")
```

## Common Deployment Issues

### Issue 1: "Cannot connect to X server"
**Solution**: Enable X11 forwarding or use VNC

```bash
# Enable X11 for Docker
xhost +local:docker
```

### Issue 2: GPU not detected in container
**Solution**: Use NVIDIA Container Toolkit

```bash
sudo apt install nvidia-container-toolkit
docker run --gpus all -it humanoid_assistant
```

### Issue 3: Gazebo simulation too slow
**Solution**: Reduce sensor update rates or disable shadows

```xml
<!-- In .world file -->
<physics type="ode">
  <real_time_update_rategreater than 1000</real_time_update_rate>
  <max_step_sizegreater than 0.001</max_step_size>
</physics>
```

## Submission Checklist

Before final submission:

- [ ] All code compiles without errors
- [ ] All tests pass (`colcon test`)
- [ ] Demo video recorded and uploaded
- [ ] README.md complete with installation instructions
- [ ] Architecture diagram included
- [ ] Code is well-commented
- [ ] Repository is public and accessible
- [ ] License file added
- [ ] CONTRIBUTING.md created
- [ ] .gitignore includes build artifacts

## Future Enhancements

Potential improvements for future iterations:

1. **Hardware Integration**: Deploy on real humanoid robot
2. **Advanced Perception**: 3D object detection, SLAM
3. **Learning from Demonstration**: Imitation learning
4. **Multi-Robot Coordination**: Team of humanoids
5. **Cloud Integration**: Remote monitoring and control
6. **Natural Language Dialogue**: Conversational capabilities beyond single commands

## Conclusion

Congratulations on completing the capstone project! You've built a sophisticated autonomous system that integrates perception, planning, and control. This project demonstrates your readiness to tackle real-world robotics challenges.

---

## Exercises

### Exercise 1: Create Your Demo Video
Record a 3-5 minute demonstration following the guidelines in this chapter.

### Exercise 2: Write Performance Benchmark Report
Measure and document all key metrics (latency, accuracy, robustness).

### Exercise 3: Dockerize Your System
Create a Dockerfile that packages your entire system for one-command deployment.

### Exercise 4: Deploy to Cloud
Set up your system on AWS EC2 or equivalent and enable remote access.

### Exercise 5: Write Installation Guide
Create step-by-step instructions that a new user can follow to replicate your system.
