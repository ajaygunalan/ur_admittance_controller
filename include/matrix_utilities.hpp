/**
 * @file matrix_utilities.hpp
 * @brief Utility functions for matrix computations in UR Admittance Controller
 * 
 * This header provides centralized matrix computation utilities to eliminate
 * code duplication and ensure consistent calculations across the controller.
 * 
 * @author Generated Code Improvement
 * @date 2025
 */

#ifndef UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_
#define UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_

#include "admittance_types.hpp"
#include "admittance_constants.hpp"
#include <array>
#include <cmath>

namespace ur_admittance_controller::utils {

/**
 * @brief Compute the damping matrix using mathematically correct formulas
 * 
 * This function implements the unified damping calculation logic that was
 * previously duplicated across multiple files. It handles the smooth transition
 * between pure admittance mode and full impedance mode.
 * 
 * @param mass Array of mass values (kg for translation, kg⋅m² for rotation)
 * @param stiffness Array of stiffness values (N/m for translation, Nm/rad for rotation) 
 * @param damping_ratio Array of damping ratio values (dimensionless)
 * @return Matrix6d The computed damping matrix
 */
inline Matrix6d computeDampingMatrix(
    const std::array<double, 6>& mass,
    const std::array<double, 6>& stiffness, 
    const std::array<double, 6>& damping_ratio)
{
    using namespace constants;
    
    Matrix6d damping = Matrix6d::Zero();
    
    for (size_t i = 0; i < 6; ++i) {
        const double mass_value = mass[i];
        const double stiffness_value = stiffness[i];
        const double damping_ratio_value = damping_ratio[i];
        
        if (stiffness_value <= 0.0) {
            // Pure admittance mode - use virtual stiffness for dimensional correctness
            // D = 2ζ√(M·K_virtual) ensures proper units [Ns/m or Nms/rad]
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * VIRTUAL_STIFFNESS);
        } 
        else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
            // Full impedance mode - standard critical damping formula
            // D = 2ζ√(M·K) for impedance control
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * stiffness_value);
        }
        else {
            // Smooth transition zone - blend between virtual and actual stiffness
            const double blend_factor = stiffness_value / STIFFNESS_BLEND_THRESHOLD;
            
            const double admittance_damping = 2.0 * damping_ratio_value * 
                                             std::sqrt(mass_value * VIRTUAL_STIFFNESS);
                
            const double impedance_damping = 2.0 * damping_ratio_value * 
                                            std::sqrt(mass_value * stiffness_value);
            
            // Smoothly blend between the two values
            damping(i, i) = (1.0 - blend_factor) * admittance_damping + 
                           blend_factor * impedance_damping;
        }
    }
    
    return damping;
}

/**
 * @brief Compute mass matrix inverse with numerical stability checks
 * 
 * @param mass Array of mass values
 * @return Matrix6d The computed mass inverse matrix
 */
inline Matrix6d computeMassInverse(const std::array<double, 6>& mass)
{
    using namespace constants;
    
    Matrix6d mass_matrix = Matrix6d::Zero();
    Matrix6d mass_inverse = Matrix6d::Zero();
    
    // Build diagonal mass matrix
    for (size_t i = 0; i < 6; ++i) {
        mass_matrix(i, i) = mass[i];
    }
    
    // Check condition number for numerical stability
    const double max_mass = mass_matrix.diagonal().maxCoeff();
    const double min_mass = mass_matrix.diagonal().minCoeff();
    const double condition_number = max_mass / min_mass;
    
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
        // Apply regularization to improve numerical stability
        for (size_t i = 0; i < 6; ++i) {
            mass_matrix(i, i) += REGULARIZATION_FACTOR;
        }
    }
    
    // For diagonal matrices, inverse is straightforward
    for (size_t i = 0; i < 6; ++i) {
        mass_inverse(i, i) = 1.0 / mass_matrix(i, i);
    }
    
    return mass_inverse;
}

/**
 * @brief Validate that a matrix is numerically stable (no NaN/Inf values)
 * 
 * @param matrix The matrix to validate
 * @return bool True if matrix is stable, false otherwise
 */
inline bool isMatrixStable(const Matrix6d& matrix)
{
    return matrix.allFinite();
}

/**
 * @brief Clamp a value between minimum and maximum bounds
 * 
 * @tparam T Numeric type
 * @param value Value to clamp
 * @param min_val Minimum bound
 * @param max_val Maximum bound
 * @return T Clamped value
 */
template<typename T>
constexpr T clamp(const T& value, const T& min_val, const T& max_val)
{
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * @brief Check if two floating-point values are approximately equal
 * 
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance (default uses PARAMETER_EPSILON)
 * @return bool True if values are approximately equal
 */
inline bool areEqual(double a, double b, double epsilon = constants::PARAMETER_EPSILON)
{
    return std::abs(a - b) < epsilon;
}

}  // namespace ur_admittance_controller::utils

#endif  // UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_
