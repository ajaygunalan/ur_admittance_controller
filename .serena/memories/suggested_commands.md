# Suggested Commands for Development

## Building the Package
```bash
# Build the controller package
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller && source install/setup.bash && cd ~/ros2_ws/src/ur_admittance_controller

# Build with verbose output for debugging
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller --event-handlers console_direct+ && source install/setup.bash

# Clean build
cd ~/ros2_ws && rm -rf build/ur_admittance_controller install/ur_admittance_controller && colcon build --packages-select ur_admittance_controller
```

## Running the System
```bash
# 1. Launch robot simulation (in separate terminal)
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

# 2. Initialize robot to equilibrium position
ros2 run ur_admittance_controller init_robot

# 3. Run calibration (one-time, saves to config)
ros2 run ur_admittance_controller wrench_calibration_node

# 4. Switch to velocity controller
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller

# 5. Start wrench node (gravity compensation)
ros2 run ur_admittance_controller wrench_node

# 6. Start admittance controller
ros2 run ur_admittance_controller admittance_node
```

## Utility Commands
```bash
# Check node status
ros2 node list
ros2 topic list
ros2 topic echo /wrench_compensated

# Monitor joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# View robot description
ros2 param get /robot_state_publisher robot_description
```

## Development Commands
```bash
# Format check (no auto-formatter configured)
# Linting (no linter configured)
# Run tests (test structure exists but no tests implemented)

# Check compilation errors
cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller 2>&1 | grep -E "error:|warning:"

# List package executables
ros2 pkg executables ur_admittance_controller
```

## Git Commands
```bash
# Check current branch and status
git status
git branch

# View recent commits
git log --oneline -10

# Standard git workflow
git add <files>
git commit -m "feat/fix/docs: description"
git push origin <branch>
```