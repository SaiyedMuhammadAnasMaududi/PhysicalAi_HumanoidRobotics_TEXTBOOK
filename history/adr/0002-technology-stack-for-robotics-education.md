# ADR-0002: Technology Stack for Robotics Education

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-05
- **Feature:** Physical AI & Humanoid Robotics Textbook
- **Context:** Educational robotics content requires selecting programming languages, simulation platforms, and robotics frameworks that balance multiple competing concerns: accessibility for beginners, alignment with industry practices, cost constraints (hardware access), safety (avoiding physical damage), and pedagogical effectiveness. Students come from diverse backgrounds (computer science, engineering, self-learners) and have varying access to physical robot hardware. The textbook must prepare students for real-world robotics development while remaining universally accessible.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We adopt an integrated technology stack optimized for accessible robotics education:

- **Primary Language:** Python 3.8+ (with rclpy for ROS 2 integration)
- **Secondary Language:** JavaScript/TypeScript (ES6+) for web-based agent examples and MCP server development
- **Robotics Framework:** ROS 2 (Humble or Iron distributions) as the communication middleware and control abstraction
- **Simulation Platforms:**
  - Gazebo Classic/Harmonic (primary physics simulation)
  - Unity ML-Agents (optional, for advanced graphics and AI integration)
- **Computer Vision:** OpenCV 4.x (stable API subset)
- **Speech Recognition:** Whisper (OpenAI) with local deployment fallback
- **Deployment Strategy:** Simulation-first with optional hardware integration path
- **Hardware Requirement:** None (all examples functional in simulation); hardware integration documented as optional extensions

## Consequences

### Positive

- **Universal Accessibility:** Zero hardware cost barrier; all students can complete curriculum using simulation on standard laptops
- **Safe Learning Environment:** Simulation eliminates physical safety risks (collisions, falls, damage) during experimentation and debugging
- **Rapid Iteration:** Students can test code changes instantly in simulation without hardware setup delays or reset procedures
- **Beginner-Friendly:** Python's readable syntax and extensive CS curriculum adoption reduces language learning curve
- **Industry-Aligned:** ROS 2 is industry standard for robotics middleware; Python widely used in robotics research and education
- **Repeatable Experiments:** Simulation provides deterministic environments for consistent evaluation and grading
- **Agent Development Support:** JavaScript/TypeScript enables modern MCP server and agent workflow examples
- **Cost-Effective Scaling:** Institutions can teach robotics without purchasing expensive physical robots for every student

### Negative

- **Simulation Gap:** Physics simulators approximate real-world dynamics imperfectly (friction, sensor noise, material properties); may create false confidence
- **Performance Disconnect:** Python's interpreted nature and simulation overhead don't reflect real-time embedded systems constraints
- **Industry Standard Gap:** Production robotics often uses C++ for performance-critical components; students may need additional learning for industry roles
- **Hardware Skills Missing:** Students don't develop hands-on troubleshooting skills (wiring, calibration, debugging physical issues)
- **Limited Haptic Feedback:** Simulation lacks tactile feedback crucial for manipulation tasks and physical interaction understanding
- **Dependency on External Services:** Whisper requires OpenAI API or local deployment setup; Unity requires proprietary software
- **Gazebo Complexity:** Gazebo has a steep learning curve for custom environments and sensor configurations

## Alternatives Considered

### Alternative Stack A: C++ + ROS 2 + Gazebo + Hardware-Required
- **Languages:** C++ primary (industry standard for ROS 2)
- **Framework:** ROS 2 (retained)
- **Simulation:** Gazebo (retained)
- **Hardware:** Physical robots required for all students
- **Why Rejected:**
  - C++ steep learning curve for beginners; slows pedagogical progress on robotics concepts
  - Hardware cost barrier ($500–$5,000 per robot) excludes many students
  - Physical robots require lab space, supervision, and maintenance
  - Safety risks and equipment damage potential

### Alternative Stack B: Python Only + ROS 2 + Simulation-Only (No Hardware Path)
- **Languages:** Python only
- **Framework:** ROS 2 (retained)
- **Simulation:** Gazebo (retained)
- **Hardware:** No hardware integration path documented
- **Why Rejected:**
  - Limits students with hardware access from exploring physical robotics
  - No JavaScript examples exclude modern agent development patterns
  - Misses teaching opportunity for hardware considerations (latency, reliability, safety)

### Alternative Stack C: Python + ROS 1 (Noetic) + Gazebo + Simulation-First
- **Languages:** Python (retained)
- **Framework:** ROS 1 Noetic (legacy version)
- **Simulation:** Gazebo (retained)
- **Hardware:** Simulation-first (retained)
- **Why Rejected:**
  - ROS 1 officially end-of-life in 2025; teaching legacy technology
  - ROS 2 has better security, real-time capabilities, and industry adoption trajectory
  - Migration burden for students transitioning to current industry practices

### Alternative Stack D: MATLAB/Simulink + Proprietary Simulators + Hardware-Optional
- **Languages:** MATLAB (proprietary)
- **Simulation:** Simulink, Gazebo, or proprietary tools
- **Hardware:** Optional
- **Why Rejected:**
  - Expensive licensing ($50–$150/student/year); accessibility barrier
  - Less industry adoption in robotics compared to Python/ROS ecosystem
  - Closed-source ecosystem limits customization and transparency

## References

- Feature Spec: [specs/006-voice-controlled-humanoid/spec.md](../../specs/006-voice-controlled-humanoid/spec.md)
- Implementation Plan: [specs/006-voice-controlled-humanoid/plan.md](../../specs/006-voice-controlled-humanoid/plan.md) (sections 2.4, 2.5, 1.3)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle IV: Practical Application and Code Quality)
- Related ADRs: ADR-0001 (Content Generation and Publishing Architecture)
- Evaluator Evidence: Constitution Check in plan.md (Principle IV: PASS)
