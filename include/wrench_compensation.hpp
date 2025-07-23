#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace ur_admittance_controller {

// Joint state data for future dynamic compensation
struct JointState {
    Wrench positions{Wrench::Zero()};
    Wrench velocities{Wrench::Zero()};
    Wrench accelerations{Wrench::Zero()};
    double timestamp{0.0};
};

// Base interface for wrench compensation strategies
class WrenchCompensator {
public:
    virtual ~WrenchCompensator() = default;
    
    // Main Yu et al. compensation method - follows pulse_force_estimation exactly:
    // - f_raw_s: raw wrench in sensor frame
    // - X_EB: transform from end-effector to base (matching pulse_force_estimation)
    // - joint_state: optional for dynamic compensation
    virtual Wrench compensate(
        const Wrench& f_raw_s,
        const Transform& X_EB,
        const JointState& joint_state = JointState()) const = 0;
    
    
    virtual std::string getType() const = 0;
};


// LROMCalibrator class removed - functionality moved to WrenchCalibrationNode

// Utility function declarations  
void writeVec3Yaml(YAML::Emitter& out, const char* key, const Vector3d& v);

} // namespace ur_admittance_controller