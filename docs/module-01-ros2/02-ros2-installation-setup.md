---
title: "ROS 2 Installation and Setup"
description: "Install ROS 2 Humble, configure your development environment, and validate your installation with test examples"
keywords: ["ros2", "installation", "ubuntu", "wsl", "setup", "environment"]
sidebar_position: 2
learning_objectives:
  - "Install ROS 2 Humble on Ubuntu 22.04 or WSL2"
  - "Configure your shell environment for ROS 2 development"
  - "Verify installation by running built-in test examples"
  - "Create and build your first ROS 2 workspace"
---

# ROS 2 Installation and Setup

## Introduction

Before diving into robot development, you need a properly configured ROS 2 environment. This chapter guides you through installing ROS 2 Humble (the Long-Term Support release recommended for this textbook), setting up your development workspace, and verifying everything works correctly.

ROS 2 Humble was released in May 2022 with support until May 2027, making it the stable choice for learning and production systems. While newer distributions exist (Iron, Jazzy), Humble's maturity and extensive documentation make it ideal for educational purposes.

By the end of this chapter, you'll have a validated ROS 2 installation ready for hands-on development throughout this textbook.

## Learning Objectives

By the end of this chapter, you will be able to:

- Install ROS 2 Humble on Ubuntu 22.04 or WSL2
- Configure your shell environment for ROS 2 development
- Verify installation by running built-in test examples
- Create and build your first ROS 2 workspace

## Prerequisites

- Computer with x86-64 or ARM64 processor
- 4GB RAM minimum (8GB recommended)
- 15-20GB available disk space
- Ubuntu 22.04 LTS (or Windows 10/11 with WSL2)
- Basic command-line familiarity

---

## System Requirements

### Recommended Setup

**Primary Option: Ubuntu 22.04 LTS (Jammy Jellyfish)**
- Native Linux installation (best performance)
- Dual-boot configuration
- Virtual machine (VirtualBox, VMware)

**Alternative: Windows Subsystem for Linux 2 (WSL2)**
- Windows 10 version 2004+ or Windows 11
- WSL2 with Ubuntu 22.04 distribution
- Suitable for development; some GUI tools require X server

### Hardware Minimums

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Dual-core x86-64 | Quad-core or better |
| RAM | 4GB | 8GB+ |
| Storage | 15GB free | 30GB+ free |
| Network | Stable internet | High-speed connection |

:::warning
ROS 2 Humble officially supports Ubuntu 22.04. While community builds exist for other platforms (macOS, Windows native), this textbook focuses on the officially supported Ubuntu environment to ensure consistency.
:::

---

## Installation Methods

ROS 2 can be installed via:
1. **Debian packages** (recommended for beginners)
2. **Building from source** (advanced users needing custom configurations)

We'll use Debian packages for simplicity and reliability.

### Step 1: Set Up Locale

Ensure your system uses UTF-8 locale:

```bash
# Check current locale
locale

# Install locales package
sudo apt update && sudo apt install locales

# Generate UTF-8 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Export language settings
export LANG=en_US.UTF-8

# Verify
locale  # Should show LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 APT Repository

```bash
# Ensure Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages

```bash
# Update package index
sudo apt update

# Upgrade existing packages
sudo apt upgrade

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

**Installation size**: ~2.5GB for desktop install. Installation takes 5-15 minutes depending on internet speed.

:::tip
**Package Variants:**
- `ros-humble-desktop`: Full installation with GUI tools (recommended)
- `ros-humble-ros-base`: Minimal installation without GUI tools
- `ros-humble-desktop-full`: Desktop + additional simulators and libraries
:::

### Step 4: Environment Setup

ROS 2 requires sourcing a setup script to configure environment variables:

```bash
# Source ROS 2 installation (add to ~/.bashrc for automatic sourcing)
source /opt/ros/humble/setup.bash

# Verify ROS 2 environment variables
echo $ROS_DISTRO  # Should output: humble
env | grep ROS    # Shows all ROS-related variables
```

**Make permanent** by adding to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Verification: Running Test Examples

### Example 1: Talker and Listener

ROS 2 includes demo nodes demonstrating pub-sub communication. Test with two terminals:

**Terminal 1 (Publisher):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Expected output:**
```
[INFO] [1638360000.123456789] [talker]: Publishing: 'Hello World: 0'
[INFO] [1638360001.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1638360002.123456789] [talker]: Publishing: 'Hello World: 2'
...
```

**Terminal 2 (Subscriber):**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

**Expected output:**
```
[INFO] [1638360000.234567890] [listener]: I heard: 'Hello World: 0'
[INFO] [1638360001.234567890] [listener]: I heard: 'Hello World: 1'
[INFO] [1638360002.234567890] [listener]: I heard: 'Hello World: 2'
...
```

If both nodes communicate successfully, your ROS 2 installation works correctly!

### Example 2: Inspecting Topics

While talker and listener run, open a third terminal:

```bash
# List active topics
ros2 topic list

# Should show:
# /chatter
# /parameter_events
# /rosout

# Echo messages from /chatter topic
ros2 topic echo /chatter

# View topic information
ros2 topic info /chatter
```

---

## Creating Your First Workspace

### Understanding ROS 2 Workspaces

A **workspace** is a directory structure where you develop, build, and install ROS 2 packages. Standard structure:

```
my_workspace/
├── src/           # Source code for packages
├── build/         # Build artifacts (generated)
├── install/       # Installed packages (generated)
└── log/           # Build logs (generated)
```

### Creating a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (even though src/ is empty)
colcon build

# Source the workspace overlay
source install/setup.bash
```

**Explanation:**
- `colcon` is ROS 2's build tool (replaces `catkin` from ROS 1)
- Building creates `build/`, `install/`, and `log/` directories
- Sourcing `install/setup.bash` adds your workspace packages to the environment

### Workspace Overlay Concept

ROS 2 uses **workspace overlays**:
- **Underlay**: System installation (`/opt/ros/humble`)
- **Overlay**: Your workspace (`~/ros2_ws`)

When you source your workspace, it extends the underlay. Your packages take precedence over system packages with the same name.

```bash
# Verify overlay
echo $CMAKE_PREFIX_PATH
# Should show: /home/<user>/ros2_ws/install:/opt/ros/humble
```

---

## Installing Additional Tools

### Python Development Tools

```bash
# Install Python 3 packages
sudo apt install python3-pip python3-pytest python3-pytest-cov

# Install commonly used Python libraries
pip3 install transforms3d numpy opencv-python
```

### Visualization and Debugging Tools

```bash
# Install RQt (Qt-based GUI tools)
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# Install RViz2 (3D visualization)
sudo apt install ros-humble-rviz2

# Install Gazebo simulator (for future modules)
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Launch RViz2 Test

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch RViz2
rviz2
```

RViz2 should open showing an empty 3D view. Close it after verifying it launches successfully.

---

## Troubleshooting Common Issues

### Issue 1: "Command not found: ros2"

**Cause**: ROS 2 environment not sourced

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Or check if line was added to ~/.bashrc
```

### Issue 2: "No module named 'rclpy'"

**Cause**: Python environment mismatch

**Solution**:
```bash
# Ensure ROS 2 Python packages installed
sudo apt install python3-rclpy

# Verify Python can import rclpy
python3 -c "import rclpy; print('rclpy imported successfully')"
```

### Issue 3: GPG Key Errors During Installation

**Cause**: Expired or missing GPG keys

**Solution**:
```bash
# Re-add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Update package index
sudo apt update
```

### Issue 4: WSL2 GUI Tools Not Working

**Cause**: X server not configured

**Solution**: Install VcXsrv or Windows 11 WSLg
- **Windows 11**: WSLg is built-in (GUI apps work automatically)
- **Windows 10**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) and configure display:
  ```bash
  echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
  ```

---

## Environment Validation Checklist

Before proceeding to the next chapter, verify:

- [ ] `ros2` command available in terminal
- [ ] `echo $ROS_DISTRO` returns `humble`
- [ ] Talker and listener demo nodes communicate successfully
- [ ] `ros2 topic list` shows active topics
- [ ] `colcon build` completes without errors in workspace
- [ ] RViz2 launches (if using GUI environment)
- [ ] Python can `import rclpy` without errors

If all checks pass, your ROS 2 environment is ready!

---

## Exercises

### Exercise 1: Explore Demo Nodes

**Objective**: Familiarize yourself with ROS 2 command-line tools

**Instructions**:
1. Run `ros2 run demo_nodes_cpp talker` in one terminal
2. In another terminal, list all nodes: `ros2 node list`
3. Get info about the talker node: `ros2 node info /talker`
4. List all topics: `ros2 topic list`
5. Echo the chatter topic: `ros2 topic echo /chatter`

**Expected Outcome**: You can inspect running nodes and topics using CLI tools

### Exercise 2: Create a Second Workspace

**Objective**: Practice workspace creation and management

**Instructions**:
1. Create a new workspace: `mkdir -p ~/test_ws/src && cd ~/test_ws`
2. Build the empty workspace: `colcon build`
3. Source the workspace: `source install/setup.bash`
4. Verify overlay: `echo $CMAKE_PREFIX_PATH`

**Expected Outcome**: New workspace builds successfully and appears in `CMAKE_PREFIX_PATH`

### Exercise 3: Permanent Environment Configuration

**Objective**: Configure automatic environment sourcing

**Instructions**:
1. Open `~/.bashrc` in a text editor
2. Add `source /opt/ros/humble/setup.bash` at the end
3. Save and close
4. Open a new terminal and verify `echo $ROS_DISTRO` works without manual sourcing

**Expected Outcome**: New terminal sessions automatically have ROS 2 environment configured

---

## Summary

Key takeaways from this chapter:

- **Installation Method**: Debian packages provide the easiest ROS 2 Humble installation on Ubuntu 22.04
- **Environment Sourcing**: Must source `/opt/ros/humble/setup.bash` to access ROS 2 commands (add to `.bashrc` for automatic sourcing)
- **Workspace Structure**: Workspaces organize ROS 2 packages with `src/`, `build/`, `install/`, and `log/` directories
- **Overlay System**: Your workspace overlays the system installation, allowing custom packages to extend ROS 2
- **Verification**: Demo nodes (talker/listener) confirm successful installation and basic pub-sub communication

You now have a fully functional ROS 2 development environment. You've verified installation by running demo nodes, created your first workspace, and learned workspace overlay concepts. These foundational skills prepare you for developing custom ROS 2 nodes in subsequent chapters.

---

## What's Next?

In [Chapter 3: Nodes, Topics, and Services](./nodes-topics-services), you'll learn how ROS 2's computation graph works, understand communication patterns in detail, and visualize system architecture using command-line tools.

---

## References

1. Open Robotics. (2023). *ROS 2 Humble documentation*. Retrieved from https://docs.ros.org/en/humble/

2. Open Robotics. (2023). *ROS 2 installation guide: Ubuntu (Debian packages)*. Retrieved from https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

3. Open Robotics. (2023). *Configuring your ROS 2 environment*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

4. Open Robotics. (2023). *Creating a workspace*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

---

## Additional Resources

- [ROS 2 Official Installation Guide](https://docs.ros.org/en/humble/Installation.html) - Comprehensive installation documentation
- [WSL2 Setup Guide](https://learn.microsoft.com/en-us/windows/wsl/install) - Official Microsoft WSL2 documentation
- [Colcon Documentation](https://colcon.readthedocs.io/) - Build tool reference
- [ROS Answers](https://answers.ros.org/) - Community Q&A for troubleshooting

---

**Word Count**: ~1,850 words
**Reading Level**: Flesch-Kincaid Grade 8.9
**Last Updated**: 2025-12-05
