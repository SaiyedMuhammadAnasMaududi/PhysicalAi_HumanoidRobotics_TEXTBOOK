---
title: "URDF and Xacro: Robot Model Composition"
description: "Master Unified Robot Description Format (URDF) and Xacro macros for creating modular and complex robot models in simulation."
keywords: ["URDF", "Xacro", "robot model", "robot description", "model composition"]
learning_objectives:
  - "Understand the purpose and structure of URDF for robot description"
  - "Identify key URDF elements like links and joints"
  - "Learn how Xacro macros enhance URDF modularity and readability"
  - "Apply Xacro to compose complex robot models from reusable components"
---

# URDF and Xacro: Robot Model Composition

To effectively simulate robots, we need a standardized way to describe their physical characteristics. The **Unified Robot Description Format (URDF)** serves this purpose within the ROS ecosystem. URDF is an XML format used for describing all elements of a robot, including its visual appearance, collision properties, kinematic and dynamic characteristics, and sensor placements.

## Understanding URDF: The Robot's Blueprint

A URDF file defines a robot as a collection of `links` (rigid bodies) connected by `joints` (revolute or prismatic connections). Each link can have associated visual and collision geometries, while joints specify the relationships and movement limits between links.

### Key URDF Elements:

-   **`<robot>`**: The root element that encapsulates the entire robot description.
-   **`<link>`**: Represents a rigid body of the robot (e.g., a torso, a wheel, a sensor housing). Links have:
    -   **`<visual>`**: Defines the graphical properties (mesh, color) for rendering the link in a simulator.
    -   **`<collision>`**: Defines the geometric properties used for collision detection.
    -   **`<inertial>`**: Specifies the mass, center of mass, and inertia matrix for physical simulations.
-   **`<joint>`**: Defines a connection between two links. Joints have:
    -   **`name`**: A unique identifier for the joint.
    -   **`type`**: Specifies the joint type (e.g., `revolute`, `continuous`, `prismatic`, `fixed`).
    -   **`<parent>`** and **`<child>`**: Specify the links connected by this joint.
    -   **`<origin>`**: Defines the pose of the child link with respect to the parent link.
    -   **`<axis>`**: Specifies the axis of rotation for revolute joints or translation for prismatic joints.
    -   **`<limit>`**: Defines the upper and lower limits for revolute/prismatic joints.

Here’s a simplified example of a single link and joint in URDF:

```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.05 0" rpy="1.57075 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Xacro: Enhancing URDF with Macros

While URDF is powerful, it can become verbose and repetitive for complex robots with many similar components (e.g., identical wheels, multiple fingers). **Xacro (XML Macros for URDF)** is an XML macro language that allows you to use macros, properties, and mathematical expressions within your URDF files. This significantly improves readability, reusability, and maintainability.

Xacro files are processed by a Python script that expands the macros and generates a standard URDF file. This means simulators only ever see a plain URDF file, but you benefit from Xacro's abstraction during development.

### Key Xacro Features:

-   **Properties**: Define variables that can be reused throughout the file.
    ```xml
    <?xml version="1.0"?>
    <robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:property name="wheel_radius" value="0.05" />
      <xacro:property name="wheel_width" value="0.02" />
      <!-- ... use ${wheel_radius} and ${wheel_width} later ... -->
    </robot>
    ```

-   **Macros**: Define reusable blocks of URDF/Xacro code that can be instantiated multiple times with different arguments.

    ```xml
    <?xml version="1.0"?>
    <robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

      <xacro:macro name="wheel" params="prefix parent_link x_offset">
        <link name="${prefix}_wheel_link">
          <visual>
            <geometry>
              <cylinder radius="0.05" length="0.02"/>
            </geometry>
          </visual>
        </link>

        <joint name="${parent_link}_to_${prefix}_wheel" type="continuous">
          <parent link="${parent_link}"/>
          <child link="${prefix}_wheel_link"/>
          <origin xyz="${x_offset} 0.05 0" rpy="1.57075 0 0"/>
          <axis xyz="0 1 0"/>
        </joint>
      </xacro:macro>

      <!-- Instantiate the macro -->
      <xacro:wheel prefix="left" parent_link="base_link" x_offset="0.1" />
      <xacro:wheel prefix="right" parent_link="base_link" x_offset="-0.1" />

    </robot>
    ```

-   **Mathematical Expressions**: Perform calculations directly within the Xacro file.
    ```xml
    <origin xyz="${(wheel_offset + wheel_radius)} 0 0" />
    ```

### Model Composition with Xacro

Xacro allows for powerful model composition by enabling you to define parts of your robot (e.g., a gripper, a leg, a sensor array) as separate Xacro macros. These macros can then be included and instantiated in a main robot description file, making it easy to:

-   **Reuse Components**: Define a wheel macro once and use it for all wheels on your robot.
-   **Modular Design**: Break down complex robots into smaller, manageable Xacro files.
-   **Parameterization**: Easily change properties (e.g., size, mass, position) of a component by passing arguments to a macro.

This modularity is crucial for developing complex humanoid robots, where components like arms, legs, and sensors are often replicated or require slight variations.

## Summary

URDF provides the foundational language for describing your robot's physical structure, while Xacro supercharges URDF by introducing macros, properties, and expressions. By mastering URDF and Xacro, you gain the ability to create highly modular, readable, and maintainable robot models, which is a fundamental skill for advanced robot simulation and development. In the next chapter, we will explore how to integrate these models into Gazebo and simulate various sensors.

## Exercises

1.  **URDF vs. Xacro**: Explain why Xacro is beneficial for complex robot models compared to plain URDF. Provide at least two specific features of Xacro that demonstrate this advantage.
2.  **URDF Element Identification**: Given a simple URDF snippet, identify and describe the purpose of `<link>`, `<joint>`, `<visual>`, `<collision>`, and `<inertial>` elements.
3.  **Xacro Macro Application**: You need to create a robot with four identical wheels. Write a simple Xacro macro for a wheel and demonstrate how you would instantiate it four times with different parameters (e.g., `prefix` and `x_offset`) to place the wheels on a `base_link`.
4.  **Modular Design**: Discuss how Xacro promotes a modular design approach in robot modeling. Why is this important for large-scale robotics projects?
