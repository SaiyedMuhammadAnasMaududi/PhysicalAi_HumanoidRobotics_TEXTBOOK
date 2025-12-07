---
title: "Control Basics: PID and Trajectory Planning"
description: "Fundamental concepts of robot control, including PID control for joint regulation and trajectory generation for smooth motion."
keywords: ["PID control", "trajectory planning", "robot kinematics", "joint control", "robot dynamics"]
learning_objectives:
  - "Explain the principles of PID control and its application in robotics."
  - "Understand how trajectories are planned for robotic manipulators."
  - "Differentiate between forward and inverse kinematics."
---

# Control Basics: PID and Trajectory Planning

Effective robotic operation hinges on precise control, allowing robots to execute desired movements accurately and smoothly. This chapter delves into the fundamental principles of robot control, focusing on Proportional-Integral-Derivative (PID) control—a cornerstone for regulating robot joints—and the essentials of trajectory planning, which ensures graceful and efficient motion.

## PID Control

PID (Proportional-Integral-Derivative) control is a widely used feedback control loop mechanism in industrial control systems and a fundamental component in robotics. It calculates an "error" value as the difference between a measured process variable and a desired setpoint. The controller attempts to minimize the error by adjusting the process control inputs. The three components—Proportional, Integral, and Derivative—each contribute to the control action:

-   **Proportional (P) Term**: This component produces an output value that is proportional to the current error. A larger proportional gain (Kp) means a larger output response for a given error. While it helps reduce the error quickly, a high Kp can lead to oscillations.

-   **Integral (I) Term**: The integral term accumulates past errors over time. It helps eliminate steady-state errors (offset) that a proportional controller might not fully correct. A larger integral gain (Ki) gives a stronger response to cumulative errors, but too high Ki can cause overshoot and instability.

-   **Derivative (D) Term**: The derivative term predicts future errors based on the current rate of change of the error. It provides a damping effect, reducing overshoot and settling time. A larger derivative gain (Kd) means a stronger response to rapid changes, but it can amplify noise if too high.

### PID in Robotics

In robotics, PID controllers are commonly used to control individual joints, ensuring they reach and maintain desired positions or velocities. For instance, a PID controller might be employed to control the angle of a robotic arm's elbow joint. The setpoint would be the target angle, the process variable would be the current angle measured by an encoder, and the output would be the torque or voltage applied to the motor.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDControllerNode(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('pid_controller')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = 0.0
        self.current_value = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        self.setpoint_sub = self.create_subscription(Float64, 'setpoint', self.setpoint_callback, 10)
        self.feedback_sub = self.create_subscription(Float64, 'feedback', self.feedback_callback, 10)
        self.control_pub = self.create_publisher(Float64, 'control_effort', 10)
        self.timer = self.create_timer(0.01, self.compute_control)

    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def feedback_callback(self, msg):
        self.current_value = msg.data

    def compute_control(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt == 0:
            return

        error = self.setpoint - self.current_value
        self.error_sum += error * dt
        error_derivative = (error - self.last_error) / dt

        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_derivative

        control_msg = Float64()
        control_msg.data = output
        self.control_pub.publish(control_msg)

        self.last_error = error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDControllerNode(Kp=1.0, Ki=0.1, Kd=0.05) # Example gains
    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Trajectory Planning

While PID control handles *how* a robot reaches a specific state, trajectory planning addresses *what path* it should take to get there smoothly and efficiently. A trajectory is a time-parameterized path, meaning it specifies not only a sequence of positions but also the velocities and accelerations at each point in time. This is crucial for avoiding jerky movements, minimizing energy consumption, and operating within physical constraints.

### Key Concepts

-   **Path vs. Trajectory**: A *path* defines a sequence of spatial points, while a *trajectory* adds a time component, dictating the timing of motion along that path.
-   **Joint Space vs. Cartesian Space**:
    -   **Joint Space**: Trajectories are planned for each individual joint angle. This is often simpler computationally but can lead to unexpected end-effector paths.
    -   **Cartesian Space**: Trajectories are planned for the robot's end-effector (e.g., its gripper) in 3D space. This is more intuitive for tasks but requires solving inverse kinematics continuously.
-   **Kinematics**:
    -   **Forward Kinematics**: Calculates the end-effector's position and orientation given the joint angles.
    -   **Inverse Kinematics**: Determines the required joint angles to achieve a desired end-effector position and orientation. This is a more complex problem and often has multiple solutions or no solution.

### Trajectory Generation Methods

Common methods for generating trajectories include:

1.  **Polynomial Interpolation**: Using polynomials (e.g., cubic, quintic) to define the position, velocity, and acceleration profiles between waypoints. This ensures smooth transitions and allows for control over the initial and final conditions.

2.  **Spline Interpolation**: Similar to polynomial interpolation but uses piecewise polynomial functions, providing more flexibility to fit complex paths while maintaining smoothness.

3.  **Minimum-Jerk/Minimum-Snap Trajectories**: These methods optimize trajectories to minimize the jerk (rate of change of acceleration) or snap (rate of change of jerk) of the robot, resulting in very smooth and human-like movements.

### Example: Simple Cubic Trajectory in Joint Space

Consider moving a single joint from an initial position \(q_0\) to a final position \(q_f\) over a time \(T\), with zero initial and final velocity. A cubic polynomial can be used:

\[ q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 \]

With boundary conditions:

-   \(q(0) = q_0\)
-   \(q(T) = q_f\)
-   \(\dot{q}(0) = 0\)
-   \(\dot{q}(T) = 0\)

Solving for the coefficients gives:

-   \(a_0 = q_0\)
-   \(a_1 = 0\)
-   \(a_2 = \frac{3}{T^2} (q_f - q_0)\)
-   \(a_3 = -\frac{2}{T^3} (q_f - q_0)\)

```python
def generate_cubic_trajectory(q0, qf, T, num_points=100):
    t = [i * T / (num_points - 1) for i in range(num_points)]
    trajectory = []

    a0 = q0
    a1 = 0
    a2 = (3 / (T**2)) * (qf - q0)
    a3 = (-2 / (T**3)) * (qf - q0)

    for ti in t:
        qi = a0 + a1 * ti + a2 * (ti**2) + a3 * (ti**3)
        trajectory.append(qi)
    return t, trajectory

# Example usage:
q_start = 0.0
q_end = 1.57 # radians (90 degrees)
t_duration = 5.0 # seconds

t_points, joint_positions = generate_cubic_trajectory(q_start, q_end, t_duration)

# You would then send these joint_positions to your robot's controllers over time.
# For demonstration, let's print a few points:
# for i in range(0, len(t_points), len(t_points)//5):
#     print(f"Time: {t_points[i]:.2f}s, Position: {joint_positions[i]:.2f} rad")
```

## Conclusion

PID control provides the robust feedback mechanisms necessary for individual joint regulation, ensuring that a robot's physical components precisely follow commands. Trajectory planning complements this by generating smooth, efficient, and physically feasible paths for the robot to follow, transforming high-level goals into low-level movements. Together, these techniques form the bedrock of responsive and agile robotic systems. The next chapter will build on these foundational control concepts by exploring how robots can reason about and execute complex tasks.
