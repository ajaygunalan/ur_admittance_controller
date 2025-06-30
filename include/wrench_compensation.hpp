#pragma once

#include "calibration_types.hpp"
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
    [[nodiscard]] virtual Wrench compensate(
        const Wrench& f_raw_s,
        const Transform& X_EB,
        const JointState& joint_state = JointState()) const = 0;
    
    
    [[nodiscard]] virtual std::string getType() const = 0;
};

// Gravity and bias compensator - parameter access only
class GravityCompensator : public WrenchCompensator {
public:
    explicit GravityCompensator(const GravityCompensationParams& params) : params_(params) {}
    
    // Legacy interface for compatibility (deprecated)
    [[nodiscard]] Wrench compensate(
        const Wrench& f_raw_s,
        const Transform& X_EB,
        const JointState& joint_state = JointState()) const override;
    
    [[nodiscard]] const GravityCompensationParams& getParams() const { return params_; }
    
    [[nodiscard]] std::string getType() const override { return "gravity_bias"; }
    [[nodiscard]] static Matrix3d skew_symmetric(const Vector3d& v);
    
private:
    GravityCompensationParams params_;
};

// LROM calibrator
class LROMCalibrator {
public:
    static constexpr size_t NUM_CALIBRATION_POSES = CalibrationConstants::NUM_POSES;
    static constexpr size_t SAMPLES_PER_POSE = CalibrationConstants::SAMPLES_PER_POSE;
    static constexpr size_t TOTAL_SAMPLES = CalibrationConstants::TOTAL_SAMPLES;
    
    [[nodiscard]] CalibrationResult calibrate(const std::vector<CalibrationSample>& samples);
    
private:
    [[nodiscard]] std::pair<Vector3d, Matrix3d> estimateGravityAndRotation(
        const std::vector<CalibrationSample>& samples) const;
    
    [[nodiscard]] Vector3d estimateForceBias(
        const std::vector<CalibrationSample>& samples,
        const Vector3d& gravity_in_base,
        const Matrix3d& rotation_s_to_e) const;
    
    [[nodiscard]] std::pair<Vector3d, Vector3d> estimateCOMAndTorqueBias(
        const std::vector<CalibrationSample>& samples,
        const Vector3d& force_bias) const;
    
    [[nodiscard]] std::pair<double, double> computeResiduals(
        const std::vector<CalibrationSample>& samples,
        const GravityCompensationParams& params) const;
};

// Utility function declarations  
void writeVec3Yaml(YAML::Emitter& out, const char* key, const Vector3d& v);

} // namespace ur_admittance_controller