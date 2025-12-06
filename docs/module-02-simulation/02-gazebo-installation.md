---
title: "Gazebo Installation and Setup"
description: "Learn to install and configure Gazebo for robust robot simulations, focusing on Gazebo Harmonic and migration from Classic."
keywords: ["Gazebo", "robot simulation", "Gazebo installation", "Gazebo Harmonic", "Gazebo Classic"]
learning_objecgtives:
  - "Understand the role of Gazebo in robotics simulation"
  - "Differentiate between Gazebo Classic and Gazebo Harmonic"
  - "Install Gazebo Harmonic on a Linux system"
  - "Perform basic verification of Gazebo installation"
---

# Gazebo Installation and Setup

Gazebo is a powerful open-source 3D robotics simulator that allows you to accurately simulate populations of robots in complex indoor and outdoor environments. It provides robust physics engines, high-quality graphics, and convenient programmatic and graphical interfaces. Gazebo is an essential tool for robotics research and development, enabling engineers to test algorithms, design robots, and perform experiments in a safe, repeatable virtual environment.

## Gazebo Classic vs. Gazebo Harmonic

It's important to understand the evolution of Gazebo to choose the right version for your learning and development. Historically, **Gazebo Classic** (also known as `gazebo-sim`) has been the prevalent version. However, Gazebo Classic reached its **end-of-life in January 2025**, and users are strongly encouraged to migrate to newer versions.

The successor to Gazebo Classic is the new **Gazebo** ecosystem (sometimes referred to as Gazebo Ignition, now simply Gazebo). The latest collection, **Gazebo Jetty**, is slated for release on September 30, 2025, and is the recommended version for all new projects and learning. This module will focus primarily on Gazebo Harmonic, which is part of the newer Gazebo ecosystem and offers improved performance, modularity, and integration capabilities.

**Recommendation**: For new installations and long-term development, **always prefer Gazebo Harmonic or later versions**. While some legacy projects might still use Gazebo Classic, focusing on the modern Gazebo ecosystem will provide access to the latest features, better support, and future compatibility.

## Installing Gazebo Harmonic (Recommended)

Gazebo Harmonic is typically installed on Linux distributions, with Ubuntu LTS versions being the most common. The following instructions are tailored for Ubuntu 22.04 LTS or newer. Always refer to the [Official Gazebo Documentation](https://gazebosim.org/docs/harmonic/install) for the most up-to-date and detailed installation instructions for your specific operating system.

### Prerequisites

Ensure your system is up-to-date and you have curl installed:

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y curl
```

### Step-by-Step Installation

1.  **Add the OSRF Repository Key:**
    ```bash
sudo curl -sSL http://gazebosim.org/distributions/gazebo/gazebo.key -o /usr/share/keyrings/gazebo-archive-keyring.gpg
    ```

2.  **Add the OSRF Repository to your sources.list:**
    ```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://gazebosim.org/distributions/gazebo/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/gazebo-latest.list > /dev/null
    ```

3.  **Update your package list and install Gazebo Harmonic:**
    ```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs
    ```
    *Note: The `ros-humble-gazebo-ros-pkgs` package includes Gazebo and its ROS 2 integration. If you are using a different ROS 2 distribution (e.g., Iron), replace `ros-humble` with the appropriate prefix.* For standalone Gazebo without ROS 2 integration, refer to the official documentation.

## Verifying Your Gazebo Installation

After installation, you can verify that Gazebo is correctly set up by launching a simple world:

1.  **Launch an Empty World:**
    ```bash
gazebo
    ```
    This command should open the Gazebo GUI with an empty 3D world. If you see the GUI without errors, your basic installation is successful.

2.  **Launch a ROS 2 integrated world (if ROS 2 is installed):**
    If you have ROS 2 installed (e.g., ROS Humble), you can test the integration by launching a simple world through ROS 2:
    ```bash
ros2 launch gazebo_ros gazebo.launch.py
    ```
    This command should also launch Gazebo, demonstrating that the ROS 2 bridge is working.

If you encounter any issues during installation or verification, refer to the [Gazebo Troubleshooting Guide](https://gazebosim.org/docs/harmonic/troubleshooting) or consult community forums.

## Summary

Installing Gazebo is a critical first step in building digital twins for robotics. By prioritizing Gazebo Harmonic, you ensure access to the latest features and a sustainable development path. With a verified installation, you are now ready to delve deeper into creating and simulating complex robot models in the virtual environment.

## Exercises

1.  **Gazebo Classic vs. Harmonic**: Explain the key differences between Gazebo Classic and Gazebo Harmonic. Why is it recommended to use Gazebo Harmonic (or later versions) for new projects?
2.  **Installation Prerequisites**: Before installing Gazebo, what are the recommended prerequisites for your Linux system? List the commands you would use to ensure these are met.
3.  **Verify Installation**: Describe the two primary methods to verify a successful Gazebo installation, one for standalone Gazebo and one for ROS 2 integrated Gazebo.
4.  **Troubleshooting Scenario**: You've followed the installation steps, but when you run `gazebo`, nothing happens or you get an error message. What are your first two steps to troubleshoot this issue based on the documentation?
