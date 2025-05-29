/**
 * @file matrix_utilities.hpp
 * @brief Mathematical utilities for matrix operations in admittance control
 *
 * This file provides utility functions for matrix computations used in the
 * admittance controller, including damping matrix computation, mass matrix
 * inversion with regularization, and numerical stability checks.
 *
 * @author UR Robotics Team
 * @date 2024
 */

#ifndef UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_
#define UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_

#include "admittance_types.hpp"
#include "admittance_constants.hpp"
#include <array>
#include <cmath>

namespace ur_admittance_controller::utils {

/**
 * @brief Compute damping matrix based on mass, stiffness, and damping ratio
 *
 * This function computes the damping matrix using critical damping formula:
 * D = 2 * ζ * sqrt(M * K)
 *
 * Special handling is provided for:
 * - Pure admittance mode (K = 0): Uses virtual stiffness
 * - Transition region (0 < K < threshold): Blends between admittance and impedance
 * - Impedance mode (K >= threshold): Uses actual stiffness
 *
 * @param mass Array of mass values [kg, kg, kg, kg⋅m², kg⋅m², kg⋅m²]
 * @param stiffness Array of stiffness values [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
 * @param damping_ratio Array of damping ratios (dimensionless, typically 0.1-2.0)
 * @return 6x6 diagonal damping matrix
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
        
        // Pure admittance mode: use virtual stiffness for stability
        if (stiffness_value <= 0.0) {
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * VIRTUAL_STIFFNESS);
        } 
        // Full impedance mode: use actual stiffness
        else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * stiffness_value);
        }
        // Transition region: blend between admittance and impedance
        else {
            const double blend_factor = stiffness_value / STIFFNESS_BLEND_THRESHOLD;
            
            const double admittance_damping = 2.0 * damping_ratio_value * 
                                             std::sqrt(mass_value * VIRTUAL_STIFFNESS);
                
            const double impedance_damping = 2.0 * damping_ratio_value * 
                                            std::sqrt(mass_value * stiffness_value);
            
            damping(i, i) = (1.0 - blend_factor) * admittance_damping + 
                           blend_factor * impedance_damping;
        }
    }
    
    return damping;
}

/**
 * @brief Compute mass matrix inverse with numerical regularization
 *
 * This function computes the inverse of a diagonal mass matrix while
 * ensuring numerical stability through regularization. If the condition
 * number is too high or any mass value is non-positive, a small
 * regularization factor is added to the diagonal.
 *
 * @param mass Array of mass/inertia values
 * @return 6x6 diagonal inverse mass matrix
 *
 * @note Regularization prevents division by zero and improves conditioning
 * @note Condition number is checked to ensure numerical stability
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
    
    // Apply regularization if needed
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
        for (size_t i = 0; i < 6; ++i) {
            mass_matrix(i, i) += REGULARIZATION_FACTOR;
        }
    }
    
    // Compute inverse (diagonal matrix -> element-wise inverse)
    for (size_t i = 0; i < 6; ++i) {
        mass_inverse(i, i) = 1.0 / mass_matrix(i, i);
    }
    
    return mass_inverse;
}

/**
 * @brief Check if a matrix contains only finite values
 *
 * @param matrix Matrix to check
 * @return true if all elements are finite (not NaN or Inf)
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
 * @return Clamped value in range [min_val, max_val]
 */
template<typename T>
constexpr T clamp(const T& value, const T& min_val, const T& max_val)
{
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * @brief Compare two floating-point numbers for approximate equality
 *
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance for comparison (default: PARAMETER_EPSILON)
 * @return true if |a - b| < epsilon
 */
inline bool areEqual(double a, double b, double epsilon = constants::PARAMETER_EPSILON)
{
    return std::abs(a - b) < epsilon;
}

}  // namespace ur_admittance_controller::utils

#endif  // UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_

