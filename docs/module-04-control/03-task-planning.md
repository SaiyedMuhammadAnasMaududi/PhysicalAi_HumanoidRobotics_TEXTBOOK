---
title: "Task Planning for Humanoid Robots"
description: "Learn hierarchical task planning, classical planning algorithms, and task decomposition strategies for autonomous robot behavior"
keywords: ["task planning", "HTN", "STRIPS", "PDDL", "hierarchical planning", "goal decomposition", "robotics"]
learning_objectives:
  - "Understand the fundamentals of automated task planning for robotics"
  - "Implement hierarchical task networks (HTN) for complex task decomposition"
  - "Apply classical planning algorithms to robot action sequencing"
  - "Integrate task planners with ROS 2 control systems"
---

# Task Planning for Humanoid Robots

## Introduction

Task planning is the process of determining **what** actions a robot should take to achieve a goal, while control determines **how** to execute those actions. When you command a humanoid to "pick up the red block and place it on the table," the robot must decompose this high-level instruction into a sequence of executable primitives like "move arm to block," "close gripper," "lift arm," and "move to table."

This chapter introduces **automated planning** techniques that enable robots to reason about their actions, handle complex multi-step tasks, and adapt to changing environments. Unlike hardcoded state machines, planners can generate action sequences dynamically based on the current world state and desired goals.

**Why Task Planning Matters:**
- **Flexibility**: Robots can handle unforeseen situations by replanning
- **Scalability**: Complex tasks decompose into manageable sub-tasks
- **Autonomy**: Reduces need for human intervention in routine operations
- **Reasoning**: Enables robots to consider action consequences before execution

## Classical Planning: STRIPS and PDDL

### STRIPS Representation

STRIPS (Stanford Research Institute Problem Solver) is a foundational planning formalism that represents the world as a set of **states**, **actions**, and **goals**.

**Components:**
1. **State**: A set of propositions describing the current world (e.g., `on_table(block_A)`, `gripper_empty`)
2. **Actions**: Operations that change the state, defined by:
   - **Preconditions**: What must be true before execution
   - **Effects**: What becomes true or false after execution
3. **Goal**: A desired state to achieve (e.g., `on_table(block_A)` AND `stacked(block_B, block_A)`)

**Example STRIPS Action:**
```python
Action: PickUp(block)
  Preconditions:
    - on_table(block)
    - clear(block)          # No other blocks on top
    - gripper_empty()
  Effects:
    - holding(block)         # Add
    - NOT on_table(block)    # Delete
    - NOT gripper_empty()    # Delete
```

### PDDL: Planning Domain Definition Language

PDDL is the standard language for defining planning problems in a machine-readable format. It separates the **domain** (available actions) from the **problem** (initial state and goal).

**PDDL Domain Example (Simplified):**
```lisp
(define (domain humanoid-manipulation)
  (:predicates
    (on_table ?obj)
    (holding ?obj)
    (gripper_empty)
    (clear ?obj))

  (:action pick_up
    :parameters (?obj)
    :precondition (and (on_table ?obj) (clear ?obj) (gripper_empty))
    :effect (and (holding ?obj) (not (on_table ?obj)) (not (gripper_empty))))

  (:action put_down
    :parameters (?obj)
    :precondition (holding ?obj)
    :effect (and (on_table ?obj) (gripper_empty) (not (holding ?obj))))
)
```

**PDDL Problem Example:**
```lisp
(define (problem pick-and-place-task)
  (:domain humanoid-manipulation)
  (:objects red_block blue_block table)
  (:init
    (on_table red_block)
    (on_table blue_block)
    (clear red_block)
    (clear blue_block)
    (gripper_empty))
  (:goal
    (and (holding red_block)))
)
```

**Planning Process:**
1. Planner searches for action sequences that transform `init` into `goal`
2. Outputs a plan: `[pick_up(red_block)]`
3. Robot executes plan actions via motion primitives

### Classical Planners

Common planning algorithms include:
- **Forward Search**: Start from initial state, apply actions until goal reached
- **Backward Search**: Start from goal, work backward to find initial state
- **GraphPlan**: Build a planning graph representing possible state transitions
- **Fast-Forward (FF)**: Heuristic-based planner optimized for speed

**Limitations:**
- Assumes deterministic actions (no uncertainty)
- Closed-world assumption (only known facts are true)
- Does not handle continuous state spaces natively

## Hierarchical Task Networks (HTN)

### Motivation

Classical planning struggles with complex, real-world tasks due to **action space explosion**. HTN planning addresses this by decomposing tasks hierarchically, mirroring how humans think about problems.

**Example: "Make breakfast"**
- **Top-Level Task**: `make_breakfast()`
- **Decomposition**:
  - `prepare_coffee()` → `grind_beans()`, `brew_coffee()`, `pour_coffee()`
  - `cook_eggs()` → `crack_eggs()`, `heat_pan()`, `scramble_eggs()`
  - `toast_bread()` → `slice_bread()`, `insert_in_toaster()`, `wait(2min)`, `retrieve_toast()`

### HTN Components

1. **Primitive Tasks**: Directly executable actions (e.g., `move_arm(x, y, z)`)
2. **Compound Tasks**: High-level tasks requiring decomposition (e.g., `pick_object(obj)`)
3. **Methods**: Rules for decomposing compound tasks into subtasks
4. **Preconditions & Constraints**: Define when methods are applicable

**HTN Method Example:**
```python
Method: pick_object(?obj)
  Preconditions:
    - gripper_empty()
    - clear(?obj)
  Decomposition:
    1. move_to(?obj)           # Primitive task
    2. align_gripper(?obj)     # Primitive task
    3. close_gripper()         # Primitive task
    4. verify_grasp(?obj)      # Primitive task (sensor check)
```

### HTN Planning Process

1. Start with high-level goal task (e.g., `pick_and_place(block_A, table)`)
2. Find applicable method to decompose task
3. Generate subtask sequence
4. Recursively decompose until only primitive tasks remain
5. Execute primitive tasks in sequence

**Advantages over Classical Planning:**
- **Domain Knowledge Encoding**: Methods capture expert strategies
- **Efficiency**: Reduces search space by constraining decompositions
- **Naturalism**: Matches human task conceptualization
- **Modularity**: Reusable methods across different problems

## Integrating Planning with ROS 2

### ROS 2 Action Server for Task Execution

Task planners generate action sequences that must be executed by ROS 2 nodes. The **Action** interface is ideal for long-running tasks with feedback.

**Example: Execute Pick-and-Place Plan**

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from manipulation_msgs.action import PickPlace  # Custom action

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor')
        self._action_server = ActionServer(
            self,
            PickPlace,
            'execute_task',
            self.execute_callback
        )
        self.get_logger().info('Task Executor Node started')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing: Pick {goal_handle.request.object_id}')

        # Simulate plan execution steps
        feedback = PickPlace.Feedback()
        plan_steps = ['approach', 'grasp', 'lift', 'transport', 'release']

        for step_idx, step in enumerate(plan_steps):
            feedback.current_step = step
            feedback.progress = (step_idx + 1) / len(plan_steps)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Step {step_idx + 1}/{len(plan_steps)}: {step}')
            rclpy.spin_once(self, timeout_sec=1.0)  # Simulate step execution time

        goal_handle.succeed()
        result = PickPlace.Result()
        result.success = True
        result.message = 'Pick-and-place completed successfully'
        return result

def main(args=None):
    rclpy.init(args=args)
    executor_node = TaskExecutorNode()
    rclpy.spin(executor_node)
    executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Replanning and Error Recovery

Real-world environments are dynamic—objects move, sensors fail, and actions may not achieve expected effects. **Replanning** allows robots to adapt when plans fail.

**Replanning Strategy:**
1. Monitor execution via sensor feedback
2. Detect discrepancies between expected and actual state
3. Trigger replanner with updated world model
4. Generate new action sequence
5. Resume execution

**Example Replan Trigger:**
```python
def monitor_grasp(self, expected_object):
    """Check if grasp succeeded"""
    current_grasp_state = self.read_gripper_sensor()

    if not current_grasp_state.object_detected:
        self.get_logger().warn('Grasp failed! Replanning...')
        self.trigger_replanner(current_world_state, goal)
        return False
    return True
```

## Practical Task Planning Example

### Scenario: Humanoid Stacks Blocks

**Goal**: Stack `block_B` on top of `block_A`

**Initial State:**
- `on_table(block_A)`, `on_table(block_B)`
- `clear(block_A)`, `clear(block_B)`
- `gripper_empty()`

**Generated Plan (HTN Decomposition):**
```
1. pick_and_place(block_B, block_A)
   ├─ pick_object(block_B)
   │  ├─ move_to(block_B_location)
   │  ├─ align_gripper(block_B)
   │  ├─ close_gripper()
   │  └─ verify_grasp(block_B)
   └─ place_object(block_B, block_A)
      ├─ move_to(block_A_location + offset)
      ├─ align_gripper_above(block_A)
      ├─ open_gripper()
      └─ verify_release(block_B)
```

**ROS 2 Integration:**
- Each primitive task publishes to control topics (e.g., `/arm/joint_commands`)
- Planner node subscribes to `/world_state` for current object positions
- Action feedback confirms each step completion before proceeding

## Summary

Task planning transforms high-level goals into executable action sequences, enabling autonomous robot behavior. Classical planning (STRIPS, PDDL) provides formal foundations, while Hierarchical Task Networks (HTN) offer scalability and domain knowledge integration. Modern humanoid systems combine planners with ROS 2 for real-time execution, monitoring, and replanning.

**Key Takeaways:**
- **Classical Planning**: Formal search-based approach (STRIPS, PDDL) for deterministic problems
- **HTN Planning**: Hierarchical decomposition mimics human reasoning and reduces search complexity
- **ROS 2 Integration**: Action servers enable long-running task execution with feedback
- **Replanning**: Essential for robustness in dynamic, uncertain environments

In the next chapter, we'll explore **Behavior Trees**, a reactive alternative to traditional planners that provides greater modularity and runtime flexibility.

## Exercises

1. **STRIPS Action Definition**: Define a PDDL action for `stack(block_x, block_y)` with appropriate preconditions and effects. Consider what must be true before stacking and what changes afterward.

2. **HTN Method Design**: Create an HTN method for the compound task `clean_table()` that decomposes into: identifying objects, picking each object, and placing them in a storage bin. Include preconditions.

3. **ROS 2 Planner Node**: Extend the `TaskExecutorNode` example to include error handling. If a step fails (simulated by random chance), log the error and attempt the step again up to 3 times before aborting.

4. **Plan Comparison**: Given the goal "humanoid moves from point A to point B while carrying an object," outline a plan using (a) classical forward search and (b) HTN decomposition. Which approach is more efficient? Why?

5. **Replanning Challenge**: Describe a scenario where replanning is necessary during a pick-and-place task. What sensor data would trigger replanning? How would the world state be updated?

## References

Ghallab, M., Nau, D., & Traverso, P. (2016). *Automated Planning: Theory and Practice*. Morgan Kaufmann.

Erol, K., Hendler, J., & Nau, D. S. (1994). HTN planning: Complexity and expressivity. *Proceedings of the Twelfth National Conference on Artificial Intelligence (AAAI-94)*, 1123-1128.

McDermott, D., Ghallab, M., Howe, A., Knoblock, C., Ram, A., Veloso, M., Weld, D., & Wilkins, D. (1998). PDDL—The Planning Domain Definition Language. *Technical Report CVC TR-98-003/DCS TR-1165*, Yale Center for Computational Vision and Control.

Nau, D., Au, T. C., Ilghami, O., Kuter, U., Murdock, J. W., Wu, D., & Yaman, F. (2003). SHOP2: An HTN planning system. *Journal of Artificial Intelligence Research*, 20, 379-404.

ROS 2 Actions Documentation. (2024). Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
