## System Instructions 
Never run a node or launch files directly
We are in a multi-node setup
You are only allowed to build - running the nodes is the user's responsibility
The nodes will move the physical robot when executed



## Plan & Review

### Before starting work
- Always start in plan mode to create a plan
- After creating the plan, write it to .claude/tasks/TASK_NAME.md
- The plan should include a detailed implementation approach, reasoning, and broken-down tasks
- If the task requires external knowledge or specific packages, research to get the latest information (Use Task tool for research)
- Don't over-plan - always think MVP (Minimum Viable Product)
- Once you write the plan, ask for review first. Do not continue until the plan is approved

### While implementing
- Update the plan as you work to reflect progress
- After completing tasks, append detailed descriptions of changes made
- Document implementation decisions for easy handover to other engineers



## Build Commands

```bash
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash && cd ~/ros2_ws/src/ur_admittance_controller
```

## Package Information

- For package overview and architecture, see @README.md
- For detailed dependencies, see @docs/dependencies.md

## Key Components

This package implements 6-DOF force-compliant motion control:
- **Init Robot**: Initializes robot to equilibrium pose
- **Wrench Calibration**: One-time calibration for gravity compensation
- **Wrench Node**: Processes force/torque sensor data
- **Admittance Node**: Implements admittance control law (M·ẍ+D·ẋ+K·x=F)