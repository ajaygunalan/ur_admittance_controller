# UR Admittance Controller Refactoring Plan

## Executive Summary
This refactoring plan aims to reduce the codebase from ~44 files to ~20-25 files while maintaining all functionality. The refactoring will improve maintainability, reduce redundancy, and create a cleaner architecture.

## Phase 1: Immediate Cleanup (Day 1)

### 1.1 Delete Empty Files
- [ ] Remove 7 empty config files:
  - `config/admittance_base.yaml`
  - `config/application_presets.yaml`
  - `config/base_config.yaml`
  - `config/impedance_examples.yaml`
  - `config/safe_startup.yaml`
  - `config/ur_complete_system.yaml`
  - `config/ur_robot_common.yaml`

- [ ] Remove 3 empty launch files:
  - `launch/ft_sensor_broadcaster.launch.py`
  - `launch/impedance_demo.launch.py`
  - `launch/ur_admittance_system.launch.py`

- [ ] Remove 3 empty scripts:
  - `scripts/test_impedance_modes.py`
  - `scripts/test_safe_startup.py`
  - `scripts/system_status.py`

- [ ] Remove empty source files:
  - `src/parameter_validation.cpp`
  - `src/communication_interface.cpp`

- [ ] Remove unused header:
  - `include/admittance_controller_services.hpp`

### 1.2 Fix Include Paths
- [ ] Fix double tf2 include paths in `admittance_controller.hpp`
- [ ] Remove unused includes from all headers

## Phase 2: Source Code Consolidation (Day 2-3)

### 2.1 Merge Duplicate Controller Files
**Target: Combine `controller_setup.cpp` and `controller_lifecycle.cpp` → `admittance_controller.cpp`**
- [ ] Keep single constructor implementation
- [ ] Merge lifecycle methods (on_init, on_configure, on_activate, etc.)
- [ ] Remove duplicate interface configuration methods
- [ ] Consolidate transform caching logic

### 2.2 Merge Computation Files
**Target: Combine `control_computations.cpp` and `realtime_control_core.cpp` → `realtime_computations.cpp`**
- [ ] Merge matrix update methods
- [ ] Consolidate admittance computation logic
- [ ] Remove duplicate parameter update code
- [ ] Simplify damping calculation (remove transition zone)

### 2.3 Create Sensor Interface Module
**Target: Combine `sensor_processing.cpp` and communication parts → `sensor_interface.cpp`**
- [ ] Consolidate F/T sensor reading
- [ ] Merge state interface updates
- [ ] Centralize sensor data validation

### 2.4 Create Utilities Module
**Target: Combine `rt_logging.cpp` and matrix utilities → `utilities.cpp`**
- [ ] Merge RT-safe logging implementations
- [ ] Add parameter validation utilities
- [ ] Include matrix helper functions

### 2.5 Keep Separate
- [ ] `plugin_export.cpp` - Keep as-is for plugin registration
- [ ] `controller_integration.cpp` - Keep for kinematics interface

## Phase 3: Configuration Simplification (Day 4)

### 3.1 Configuration Files
- [ ] Keep only `admittance_config.yaml` as main configuration
- [ ] Move example configurations to documentation
- [ ] Create `config/examples/` directory for sample configs

### 3.2 Update CMakeLists.txt
- [ ] Update source file list to reflect new structure
- [ ] Remove references to deleted files
- [ ] Simplify build configuration

## Phase 4: Header Cleanup (Day 5)

### 4.1 Simplify Main Header
- [ ] Split `admittance_controller.hpp` into:
  - `admittance_controller.hpp` - Core controller class (reduced)
  - `admittance_impl.hpp` - Implementation details
  - `admittance_interfaces.hpp` - Interface definitions

### 4.2 Create Parameter Struct
- [ ] Move all parameters to dedicated struct in `admittance_types.hpp`
- [ ] Remove parameter duplication across files

## Phase 5: Documentation Consolidation (Day 6)

### 5.1 Merge Documentation
- [ ] Combine architecture docs → `ARCHITECTURE.md`
- [ ] Keep `README.md`, `NOTATION_GUIDE.md`
- [ ] Create `CONFIGURATION_GUIDE.md` from empty config files
- [ ] Remove redundant/empty docs

## Phase 6: Testing and Validation (Day 7)

### 6.1 Update Tests
- [ ] Update test scripts for new structure
- [ ] Ensure all functionality preserved
- [ ] Create integration tests

### 6.2 Build and Runtime Testing
- [ ] Test compilation with new structure
- [ ] Verify controller loads correctly
- [ ] Test with UR simulator
- [ ] Verify F/T sensor integration

## Expected Outcome

### File Count Reduction
- **Before**: ~44 files
- **After**: ~20-25 files

### New Structure:
```
ur_admittance_controller/
├── src/
│   ├── admittance_controller.cpp      (merged lifecycle + setup)
│   ├── realtime_computations.cpp      (merged computations + RT core)
│   ├── sensor_interface.cpp           (sensor processing)
│   ├── controller_integration.cpp     (keep as-is)
│   ├── utilities.cpp                  (logging + helpers)
│   └── plugin_export.cpp              (keep as-is)
├── include/
│   ├── admittance_controller.hpp      (simplified)
│   ├── admittance_types.hpp           (enhanced with param struct)
│   ├── admittance_constants.hpp       (keep as-is)
│   └── matrix_utilities.hpp           (keep as-is)
├── config/
│   ├── admittance_config.yaml         (main config)
│   └── examples/                      (sample configs)
├── launch/
│   ├── ur_admittance.launch.py        (main launcher)
│   └── test_runner.launch.py          (test launcher)
├── scripts/
│   ├── ur_admittance_utils.py         (utilities)
│   ├── ur_admittance_tests.py         (all tests)
│   └── validate_notation.py           (validation)
└── docs/
    ├── README.md
    ├── ARCHITECTURE.md                 (merged)
    ├── NOTATION_GUIDE.md
    └── CONFIGURATION_GUIDE.md          (new)
```

## Implementation Guidelines

1. **Preserve Functionality**: Every refactoring step must maintain existing functionality
2. **Test Continuously**: Run tests after each major change
3. **Commit Frequently**: Make atomic commits for each refactoring step
4. **Document Changes**: Update documentation as code changes
5. **Backward Compatibility**: Ensure config files remain compatible

## Risk Mitigation

1. Create feature branch for refactoring
2. Keep backup of original structure
3. Test each phase independently
4. Have rollback plan for each phase
5. Maintain detailed refactoring log

## Success Metrics

- [ ] All tests pass
- [ ] Controller loads and runs correctly
- [ ] F/T sensor integration works
- [ ] Code coverage maintained or improved
- [ ] Build time reduced
- [ ] File count reduced by 40-50%
- [ ] No functionality lost