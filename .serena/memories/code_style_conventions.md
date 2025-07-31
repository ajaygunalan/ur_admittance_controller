# Code Style and Conventions

## Naming Conventions
- **Namespaces**: `ur_admittance_controller`, with sub-namespaces `algorithms`, `kinematics`, `utilities`
- **Classes**: PascalCase (e.g., `AdmittanceNode`, `KinematicsComponents`)
- **Functions**: camelCase (e.g., `computeForwardKinematics`, `initializeFromUrdf`)
- **Variables**: snake_case with trailing underscore for member variables (e.g., `q_current_`, `fk_pos_solver_`)
- **Constants**: UPPER_CASE or constexpr with snake_case (e.g., `DOF = 6`)

## File Organization
- Header files use `#pragma once`
- Headers organized in `include/ur_admittance_controller/` with subdirectories for `algorithms/` and `utilities/`
- Implementation in `src/` directory
- Third-party header-only libraries in `include/` (fmt, tl::expected)

## Type System
- Custom types defined in `utilities/types.hpp`
- Using Eigen types: `Vector3d`, `Matrix3d`, `Transform` (Isometry3d)
- 6D types: `Vector6d`, `Matrix6d`, `Wrench6d`
- Result<T> pattern for error handling using tl::expected

## Error Handling
- Result<T> pattern with tl::expected for function returns
- Error codes defined in `utilities/error.hpp`
- ENSURE macro for precondition checks
- Detailed error messages with context

## Documentation
- Doxygen-style comments for functions and classes
- @brief, @param, @return tags
- Inline comments for complex logic

## Logging
- RCLCPP logging macros (INFO, ERROR, DEBUG, WARN)
- Throttled logging for repetitive messages
- Structured log messages with context

## Best Practices
- Const correctness enforced
- Smart pointers for dynamic memory (std::unique_ptr, std::shared_ptr)
- References for large objects
- Inline functions in headers for performance
- Modular design with clear separation of concerns