---
title: "Behavior Trees for Robot Control"
description: "Master behavior tree design patterns for reactive, modular robot control systems using BehaviorTree.CPP and ROS 2"
keywords: ["behavior trees", "BT.CPP", "reactive control", "state machines", "robot behavior", "modular AI"]
learning_objectives:
  - "Understand behavior tree architecture and node types"
  - "Design behavior trees for complex robot tasks"
  - "Implement behavior trees using BehaviorTree.CPP library"
  - "Integrate behavior trees with ROS 2 control systems"
---

# Behavior Trees for Robot Control

## Introduction

**Behavior Trees (BTs)** are a control architecture that combines the strengths of finite state machines and hierarchical planners. Originally developed for game AI, behavior trees have become increasingly popular in robotics due to their **modularity**, **reactivity**, and **readability**.

Unlike traditional state machines that require explicit transitions between every state pair, behavior trees organize behaviors hierarchically with simple, reusable building blocks. This makes them easier to design, debug, and extend as robot capabilities grow.

**Why Behavior Trees for Humanoids:**
- **Modularity**: Subtrees can be reused across different tasks
- **Reactivity**: Trees re-evaluate conditions every tick, enabling dynamic responses
- **Composability**: Complex behaviors emerge from simple primitives
- **Debugging**: Tree visualization makes logic transparent
- **Scalability**: Add new behaviors without rewriting existing logic

**Real-World Applications:**
- Warehouse robots switching between navigation, picking, and charging
- Humanoid assistants handling interruptions during task execution
- Service robots adapting to environmental changes (e.g., obstacle appears)

## Behavior Tree Fundamentals

### Tree Structure

A behavior tree is a **directed tree** where:
- **Root**: The topmost node, executed every tick
- **Internal Nodes**: Control flow (Sequence, Fallback, Parallel)
- **Leaf Nodes**: Actions or Conditions that interact with the robot

**Execution Model:**
1. Start at root node
2. Traverse tree depth-first, left-to-right
3. Each node returns a status: `SUCCESS`, `FAILURE`, or `RUNNING`
4. Parent nodes control child execution based on statuses
5. Repeat ("tick") at fixed frequency (e.g., 10 Hz)

### Node Types

#### 1. Action Nodes (Leaf Nodes)

Actions perform tasks like "move arm," "grasp object," or "publish message." They return status based on execution outcome.

**Example Action:**
```python
class MoveToPosition(ActionNode):
    def tick(self):
        if self.reached_goal():
            return SUCCESS
        elif self.is_moving():
            return RUNNING  # Still in progress
        else:
            self.start_movement()
            return RUNNING
```

**Characteristics:**
- **Asynchronous**: Can span multiple ticks (return `RUNNING`)
- **Side Effects**: Modify robot state or environment
- **Failure Handling**: Return `FAILURE` on errors (e.g., unreachable target)

#### 2. Condition Nodes (Leaf Nodes)

Conditions check world state without side effects. They return `SUCCESS` or `FAILURE` instantly (never `RUNNING`).

**Example Condition:**
```python
class IsBatteryLow(ConditionNode):
    def tick(self):
        battery_level = self.read_battery_sensor()
        return SUCCESS if battery_level < 20.0 else FAILURE
```

**Best Practices:**
- **No Side Effects**: Only read state, never modify
- **Fast Execution**: Should complete in < 1 ms
- **Clear Naming**: Use boolean phrasing ("IsBatteryLow", "ObjectDetected")

#### 3. Sequence Node (Control Flow)

Executes children left-to-right. Returns:
- `SUCCESS` if **all** children succeed
- `FAILURE` if **any** child fails (short-circuits)
- `RUNNING` if current child is running

**Use Case**: Sequential steps that must all succeed (e.g., "approach AND grasp AND lift")

**Pseudocode:**
```python
class SequenceNode:
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == FAILURE:
                return FAILURE
            elif status == RUNNING:
                return RUNNING
        return SUCCESS  # All children succeeded
```

**Example: Pick Object**
```
Sequence
├─ MoveToObject
├─ AlignGripper
├─ CloseGripper
└─ LiftObject
```
If `AlignGripper` fails, the sequence aborts (returns `FAILURE`).

#### 4. Fallback Node (Selector)

Executes children left-to-right. Returns:
- `SUCCESS` if **any** child succeeds (short-circuits)
- `FAILURE` if **all** children fail
- `RUNNING` if current child is running

**Use Case**: Fallback strategies (e.g., "try primary approach OR secondary approach OR give up")

**Pseudocode:**
```python
class FallbackNode:
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == SUCCESS:
                return SUCCESS
            elif status == RUNNING:
                return RUNNING
        return FAILURE  # All children failed
```

**Example: Grasp with Retry**
```
Fallback
├─ GraspFromTop
├─ GraspFromSide
└─ RequestHumanHelp
```
If `GraspFromTop` fails, try `GraspFromSide`. If both fail, request help.

#### 5. Parallel Node

Executes **all** children simultaneously. Returns based on policy (e.g., "succeed when N children succeed").

**Use Case**: Concurrent behaviors (e.g., "navigate to goal WHILE avoiding obstacles WHILE monitoring battery")

**Policies:**
- `SucceedOnAll`: Return `SUCCESS` when all children succeed
- `SucceedOnOne`: Return `SUCCESS` when at least one child succeeds
- `FailOnAll`: Return `FAILURE` when all children fail

**Example: Navigate with Monitoring**
```
Parallel (SucceedOnOne)
├─ NavigateToGoal
├─ AvoidObstacles
└─ MonitorBattery
```

#### 6. Decorator Nodes

Modifies child behavior (e.g., repeat, invert, timeout).

**Common Decorators:**
- **Inverter**: Flips child's `SUCCESS` ↔ `FAILURE`
- **Retry(N)**: Re-execute child up to N times on failure
- **Timeout(T)**: Fail if child doesn't succeed within T seconds
- **ForceSuccess**: Always return `SUCCESS` regardless of child result

**Example: Retry Failed Grasp**
```
Retry(3)
└─ GraspObject
```

## Designing Behavior Trees for Humanoids

### Design Pattern 1: Priority-Based Behaviors

Higher-priority behaviors execute before lower-priority ones using Fallback nodes.

**Example: Autonomous Patrol with Interrupts**
```
Fallback
├─ Sequence (Handle Emergency)
│  ├─ EmergencyDetected?
│  └─ PerformShutdown
├─ Sequence (Recharge if Low Battery)
│  ├─ IsBatteryLow?
│  └─ NavigateToCharger
└─ PatrolArea (Default Behavior)
```

**Execution Logic:**
1. Check for emergency → If detected, shutdown
2. Check battery → If low, recharge
3. Otherwise, continue patrol

### Design Pattern 2: State Monitoring with Actions

Monitor conditions while executing actions using Parallel nodes.

**Example: Pick Object with Safety Monitoring**
```
Parallel (SucceedOnAll)
├─ Sequence (Main Task)
│  ├─ MoveToObject
│  ├─ GraspObject
│  └─ LiftObject
└─ Sequence (Safety Monitor)
   ├─ ObstacleDetected?
   └─ EmergencyStop
```

### Design Pattern 3: Hierarchical Subtrees

Encapsulate reusable behaviors in subtrees for modularity.

**Example: Deliver Object**
```
Sequence
├─ SubTree: PickObject(red_block)
├─ SubTree: NavigateToLocation(table)
└─ SubTree: PlaceObject(red_block)
```

Each subtree is independently designed and tested.

## Implementing Behavior Trees with BehaviorTree.CPP

### Installation

BehaviorTree.CPP is a C++ library for behavior trees with ROS 2 integration.

```bash
# Install dependencies
sudo apt install ros-humble-behaviortree-cpp-v3

# Or build from source
cd ~/ros2_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/ros2_ws
colcon build --packages-select behaviortree_cpp_v3
```

### Creating Custom Action Nodes

**C++ Example:**
```cpp
#include <behaviortree_cpp_v3/action_node.h>

class SayHello : public BT::SyncActionNode {
public:
    SayHello(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        std::cout << "Hello from humanoid!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};
```

**Python Example (using py_trees):**
```python
import py_trees

class MoveToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_x, target_y):
        super().__init__(name)
        self.target = (target_x, target_y)

    def update(self):
        # Simulate movement logic
        current_position = self.get_current_position()
        if self.distance(current_position, self.target) < 0.1:
            return py_trees.common.Status.SUCCESS
        else:
            self.move_towards(self.target)
            return py_trees.common.Status.RUNNING

    def distance(self, pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
```

### Defining Tree Structure (XML)

BehaviorTree.CPP supports XML-based tree definitions for easy visualization and modification.

**Example XML:**
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <Condition ID="IsBatteryLow"/>
      <Fallback>
        <Action ID="FindCharger"/>
        <Action ID="RequestHelp"/>
      </Fallback>
      <Action ID="NavigateToCharger"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Loading and Executing:**
```cpp
#include <behaviortree_cpp_v3/bt_factory.h>

int main() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<FindCharger>("FindCharger");
    factory.registerNodeType<NavigateToCharger>("NavigateToCharger");

    auto tree = factory.createTreeFromFile("behavior_tree.xml");

    // Execute tree at 10 Hz
    while (rclcpp::ok()) {
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
```

## Integrating Behavior Trees with ROS 2

### ROS 2 Action Node Integration

Behavior tree actions can call ROS 2 action servers for long-running tasks.

**Example: Navigation Action Node**
```cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class NavigateToPose : public BT::StatefulActionNode {
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;

    NavigateToPose(const std::string& name, const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node) {
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            node_, "navigate_to_pose");
    }

    BT::NodeStatus onStart() override {
        // Send goal to action server
        auto goal = NavigateToPoseAction::Goal();
        goal.pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose").value();

        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
        goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        // Check if action completed
        if (goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto result = goal_handle_future_.get();
            return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Cancel action if tree is interrupted
        action_client_->async_cancel_goal(goal_handle_future_.get());
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    std::shared_future<NavigateToPoseAction::Result> goal_handle_future_;
};
```

### Blackboard for Data Sharing

The **Blackboard** is a shared key-value store for passing data between nodes.

**Example:**
```cpp
// Set data in blackboard
config().blackboard->set("target_object", "red_block");

// Get data in another node
auto target = getInput<std::string>("target_object").value();
```

## Behavior Trees vs. State Machines vs. Planners

| Feature | Behavior Trees | State Machines | Classical Planners |
|---------|---------------|----------------|-------------------|
| **Modularity** | High (subtrees) | Low (monolithic) | Medium (actions) |
| **Reactivity** | High (re-evaluate each tick) | Medium (event-driven) | Low (offline planning) |
| **Readability** | High (tree visualization) | Medium (state diagrams) | Low (search logic) |
| **Ease of Extension** | High (add subtrees) | Low (add states/transitions) | Medium (add actions) |
| **Best For** | Reactive, hierarchical behaviors | Well-defined state transitions | Goal-oriented reasoning |

**When to Use Each:**
- **Behavior Trees**: Reactive tasks with frequent environmental changes (e.g., patrol robot)
- **State Machines**: Systems with clear, stable states (e.g., traffic light controller)
- **Planners**: Goal-oriented tasks requiring long-term reasoning (e.g., multi-step assembly)

## Summary

Behavior trees provide a powerful, modular framework for designing reactive robot control systems. Their hierarchical structure and intuitive execution model make them ideal for humanoid robots operating in dynamic environments.

**Key Takeaways:**
- **Node Types**: Actions (tasks), Conditions (checks), Control Flow (Sequence, Fallback, Parallel), Decorators (modifiers)
- **Design Patterns**: Priority behaviors, parallel monitoring, hierarchical subtrees
- **Implementation**: BehaviorTree.CPP (C++) and py_trees (Python) libraries
- **ROS 2 Integration**: Action nodes for long-running tasks, Blackboard for data sharing
- **Comparison**: BTs excel at reactive, modular behaviors; complement planners for high-level reasoning

Next chapter explores **Vision-Language-Action (VLA) architectures**, which integrate behavior trees with LLM-based reasoning for natural language control.

## Exercises

1. **Design a Delivery Robot BT**: Create a behavior tree for a humanoid that must deliver an object from location A to location B. Include: object detection, pick-up, navigation, obstacle avoidance, and place-down. Use Sequence, Fallback, and Parallel nodes.

2. **Implement a Python Action Node**: Write a `GraspObject` action node using py_trees that simulates grasping by checking if a gripper sensor detects an object. Return `SUCCESS` if detected, `FAILURE` otherwise, and `RUNNING` while checking.

3. **Debug a Broken Tree**: Given this tree:
   ```
   Sequence
   ├─ IsBatteryLow?
   ├─ NavigateToCharger
   └─ ChargeUntilFull
   ```
   Explain what happens if the battery is NOT low. How would you fix this tree to only charge when needed?

4. **Compare with State Machine**: Redesign the "Patrol with Battery Check" behavior tree as a finite state machine. Which approach has fewer states/nodes? Which is easier to extend with new behaviors?

5. **ROS 2 Integration Challenge**: Sketch pseudocode for a behavior tree node that subscribes to a ROS 2 topic `/obstacle_detected` (type: `std_msgs/Bool`). If an obstacle is detected, the node should return `SUCCESS`; otherwise, `FAILURE`.

## References

Colledanchise, M., & Ögren, P. (2018). *Behavior Trees in Robotics and AI: An Introduction*. CRC Press.

Isla, D. (2005). Handling complexity in the Halo 2 AI. *Game Developers Conference (GDC)*.

BehaviorTree.CPP Documentation. (2024). Retrieved from https://www.behaviortree.dev/

py_trees Documentation. (2024). Retrieved from https://py-trees.readthedocs.io/

Paxton, C., Hundt, A., Jonathan, F., Guerin, K., & Hager, G. D. (2017). CoSTAR: Instructing collaborative robots with behavior trees and vision. *IEEE International Conference on Robotics and Automation (ICRA)*, 564-571.
