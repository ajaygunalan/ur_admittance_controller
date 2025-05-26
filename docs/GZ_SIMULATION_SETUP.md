# **UR Robot Simulation - Complete System Overview**

When you launch `ur_sim_control.launch.py`, you're starting a comprehensive robotic simulation environment. Here's what becomes available:

## **🤖 Core Robot System**

### **Active Controllers:**
- **`joint_state_broadcaster`** - Continuously publishes real-time joint data (positions, velocities, efforts) from all 6 robot joints
- **`scaled_joint_trajectory_controller`** - Accepts trajectory commands to move the robot arm smoothly between waypoints

### **Key Robot Topics:**
- **`/joint_states`** - Real-time feed of all joint positions, velocities, and efforts
- **`/robot_description`** - URDF model defining robot geometry, joints, and properties
- **`/scaled_joint_trajectory_controller/joint_trajectory`** - Send movement commands here
- **`/tf` & `/tf_static`** - Coordinate transformations between all robot links

## **🎮 Control & Monitoring Infrastructure**

### **Controller Management:**
The `/controller_manager/*` services let you:
- Load/unload new controllers dynamically
- Switch between different control modes
- Monitor controller health and performance
- List available hardware interfaces

### **Visualization & Interaction:**
- **`/clicked_point` & `/goal_pose`** - RViz interaction points for setting targets
- **`/initialpose`** - Set robot's initial position in RViz
- **`/diagnostics`** - System health monitoring

