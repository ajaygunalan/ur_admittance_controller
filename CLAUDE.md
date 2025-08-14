## Overview
6-DOF force-compliant motion control for Universal Robots
Multi-node ROS2 package that moves physical robot based on force input
Code edits only - user handles all building and execution

## Environment Reality
- Claude Code runs on WSL2 (no ROS2, no dependencies)
- ROS2 only exists in Docker container
- Files shared between WSL2 and Docker
- Can only: read files, edit files, ask for debug output
- Cannot: build, run commands, execute anything

## Working Process

### Planning
- Start in plan mode for complex tasks
- Think MVP - minimum viable solution
- Break down into clear steps
- Ask for user approval before implementing

### Implementation Cycle
1. Read code to understand issue
2. Ask user to run diagnostic commands in Docker
3. Analyze output
4. Edit files to fix
5. Ask user to build and test:
```bash
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash
```

### Debugging Requests
Ask user to run in Docker:
- ros2 topic echo /topic_name
- ros2 param get /node_name param_name
- ros2 node list
- ros2 run --prefix 'gdb' package_name node_name

## Documentation
- Package architecture: @README.md
- Dependencies: @docs/dependencies.md
- Debugging guide: @docs/debugging.md

## Technical Details

### Components
- Init Robot: Sets equilibrium pose
- Wrench Calibration: Gravity compensation setup
- Wrench Node: Processes force/torque data
- Admittance Node: Implements M·ẍ+D·ẋ+K·x=F

### Control Behavior
See README.md "Control Theory & Behavior" section for:
- Why robot works without wrench_node (F=0 case)
- Different control modes and equations
- Diagnostic commands