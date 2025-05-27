
#ifndef UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_
#define UR_ADMITTANCE_CONTROLLER__MATRIX_UTILITIES_HPP_

#include "admittance_types.hpp"
#include "admittance_constants.hpp"
#include <array>
#include <cmath>

namespace ur_admittance_controller::utils {

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
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * VIRTUAL_STIFFNESS);
        } 
        else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * stiffness_value);
        }
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

inline Matrix6d computeMassInverse(const std::array<double, 6>& mass)
{
    using namespace constants;
    
    Matrix6d mass_matrix = Matrix6d::Zero();
    Matrix6d mass_inverse = Matrix6d::Zero();
    
    for (size_t i = 0; i < 6; ++i) {
        mass_matrix(i, i) = mass[i];
    }
    
    const double max_mass = mass_matrix.diagonal().maxCoeff();
    const double min_mass = mass_matrix.diagonal().minCoeff();
    const double condition_number = max_mass / min_mass;
    
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
        for (size_t i = 0; i < 6; ++i) {
            mass_matrix(i, i) += REGULARIZATION_FACTOR;
        }
    }
    
    for (size_t i = 0; i < 6; ++i) {
        mass_inverse(i, i) = 1.0 / mass_matrix(i, i);
    }
    
    return mass_inverse;
}

inline bool isMatrixStable(const Matrix6d& matrix)
{
    return matrix.allFinite();
}

template<typename T>
constexpr T clamp(const T& value, const T& min_val, const T& max_val)
{
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

inline bool areEqual(double a, double b, double epsilon = constants::PARAMETER_EPSILON)
{
    return std::abs(a - b) < epsilon;
}

}

#endif

