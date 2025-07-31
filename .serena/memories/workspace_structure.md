# UR Admittance Controller Workspace Structure

## Project Location
The project is located at: `/home/ajay/ros2_ws/src/ur_admittance_controller`

## Directory Structure
```
ur_admittance_controller/
├── include/                    # Header files
│   ├── ur_admittance_controller/
│   │   ├── algorithms/        # Algorithm implementations
│   │   │   ├── admittance_control.hpp
│   │   │   ├── kinematics.hpp
│   │   │   ├── pose_error.hpp
│   │   │   ├── workspace_limits.hpp
│   │   │   └── wrench_compensation.hpp
│   │   ├── utilities/         # Utility functions and types
│   │   │   ├── conversions.hpp
│   │   │   ├── error.hpp
│   │   │   ├── kinematics_utils.hpp
│   │   │   ├── spatial_math.hpp
│   │   │   └── types.hpp
│   │   └── algorithms.hpp     # Convenience header
│   ├── fmt/                   # fmt library (header-only)
│   ├── tl/                    # expected library (header-only)
│   └── *.hpp                  # Node class headers
├── src/                       # Source files
│   ├── admittance_computations.cpp
│   ├── admittance_node.cpp
│   ├── init_robot.cpp
│   ├── wrench_calibration_algorithm.cpp
│   ├── wrench_calibration_node.cpp
│   ├── wrench_node.cpp
│   ├── temp/                  # Temporary/test files
│   └── utilities/             # Utility implementations
├── config/                    # Configuration files
│   ├── admittance_config.yaml
│   ├── equilibrium.yaml (generated)
│   └── wrench_calibration.yaml (generated)
├── launch/                    # ROS2 launch files
├── test/                      # Test structure (not implemented)
│   ├── unit/
│   ├── integration/
│   └── data/
├── docs/                      # Documentation
│   └── dependencies.md
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS2 package manifest
├── README.md                 # Project documentation
└── CLAUDE.md                 # Development guidelines
```

## Key Design Patterns
1. **Separation of Concerns**: Algorithms separated from node implementations
2. **Header-Only Libraries**: fmt and tl::expected included directly
3. **Namespace Organization**: Clean hierarchy with ur_admittance_controller as root
4. **Modular Architecture**: Each component (wrench, admittance, calibration) is independent