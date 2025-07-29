# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Plan & Review

### Before starting work
- Always in plan mode to make a plan
- After get the plan, make sure you Write the plan to .claude/tasks/TASK_NAME.md.
- The plan should be a detailed implementation plan and the reasoning behind them, as well as tasks broken down.
- If the task require external knowledge or certain package, also research to get latest knowledge (Use Task tool for research)
- Don't over plan it, always think MVP.
- Once you write the plan, firstly ask me to review it. Do not continue until I approve the plan.

### While implementing
- You should update the plan as you work.
- After you complete tasks in the plan, you should update and append detailed descriptions of the changes you made, so following tasks can be easily hand over to other engineers.

## Overview

ROS2 package for 6-DOF force-compliant motion control of Universal Robots. Uses admittance control with gravity compensation.

## Build Commands

```bash
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash && cd ~/ros2_ws/src/ur_admittance_controller

```

## Architecture

### Core Nodes

**Admittance Node** (`src/admittance_node.cpp`)
- Implements: M·ẍ + D·ẋ + K·x = F
- Subscribes: `/netft/proc_probe_base`, `/joint_states`
- Publishes: `/forward_velocity_controller/commands`
- Runs at 100Hz, uses KDL for kinematics

**Wrench Node** (`src/wrench_node.cpp`)
- Subscribes: `/netft/raw_sensor`
- Publishes: `/netft/proc_sensor`, `/netft/proc_probe`, `/netft/proc_probe_base`
- Applies gravity compensation from calibration

**Calibration Node** (`src/wrench_calibration_node.cpp`)
- Runs LROM calibration (32 poses)
- Saves to: `config/wrench_calibration.yaml`

**Init Robot** (`src/init_robot.cpp`)
- Sets equilibrium pose
- Saves to: `config/equilibrium.yaml`

### Variable Naming

Drake-style: `<Quantity>_<Point>_<Frame>`
- `F_P_B`: Force at Payload in Base frame
- `X_BP`: Transform from Base to Payload

### Error Handling

- Tier 1: `ENSURE()` for invariants
- Tier 2: Exceptions in setup
- Tier 3: `Status`/`Result<T>` in control loops

### Key Files

- `config/admittance_config.yaml` - Control parameters (M/K/D)
- `config/equilibrium.yaml` - Robot equilibrium pose
- `config/wrench_calibration.yaml` - Gravity compensation params

## Development

Debug mode:
```bash
ros2 run ur_admittance_controller admittance_node --ros-args --log-level debug
```

Runtime tuning:
```bash
ros2 param set /admittance_node admittance.mass "[5.0,5.0,5.0,2.0,2.0,2.0]"
```

## Dependencies

- KDL (kinematics)
- Eigen3 (math)
- tf2 (transforms)
- yaml-cpp (config files)
- tl::expected (error handling)
- fmt (formatting)