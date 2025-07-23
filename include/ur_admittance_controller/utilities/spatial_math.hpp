#pragma once

#include "types.hpp"

namespace ur_admittance_controller::spatial {

/**
 * Transform a wrench from one frame to another using adjoint transformation
 * 
 * The adjoint transformation for wrenches is:
 * w_B = Ad(X_BA) * w_A
 * 
 * Where:
 * - w_A is the wrench expressed in frame A
 * - w_B is the wrench expressed in frame B
 * - X_BA is the transform from frame A to frame B
 * 
 * @param wrench_A Wrench expressed in frame A [fx, fy, fz, tx, ty, tz]
 * @param X_BA Transform from frame A to frame B
 * @return Wrench expressed in frame B
 */
inline Wrench6d transformWrench(const Wrench6d& wrench_A, const Transform& X_BA) {
    Wrench6d wrench_B;
    
    // Extract rotation and translation
    const Matrix3d& R_BA = X_BA.rotation();
    const Vector3d& p_BA = X_BA.translation();
    
    // Extract force and torque components
    const Vector3d force_A = wrench_A.head<3>();
    const Vector3d torque_A = wrench_A.tail<3>();
    
    // Transform force: f_B = R_BA * f_A
    wrench_B.head<3>() = R_BA * force_A;
    
    // Transform torque: tau_B = R_BA * tau_A + p_BA × (R_BA * f_A)
    wrench_B.tail<3>() = R_BA * torque_A + p_BA.cross(R_BA * force_A);
    
    return wrench_B;
}

/**
 * Compute the adjoint matrix for wrench transformation
 * 
 * The 6x6 adjoint matrix Ad(X) is:
 * [ R     0   ]
 * [ [p]×R  R   ]
 * 
 * Where R is the rotation matrix and [p]× is the skew-symmetric matrix of p
 * 
 * @param X Transform
 * @return 6x6 adjoint matrix
 */
inline Matrix6d adjointMatrix(const Transform& X) {
    Matrix6d Ad = Matrix6d::Zero();
    
    const Matrix3d& R = X.rotation();
    const Vector3d& p = X.translation();
    
    // Top-left: R
    Ad.block<3, 3>(0, 0) = R;
    
    // Bottom-right: R
    Ad.block<3, 3>(3, 3) = R;
    
    // Bottom-left: [p]× * R
    Matrix3d p_cross;
    p_cross <<     0, -p.z(),  p.y(),
               p.z(),      0, -p.x(),
              -p.y(),  p.x(),      0;
    
    Ad.block<3, 3>(3, 0) = p_cross * R;
    
    return Ad;
}

/**
 * Compute the cross product matrix (skew-symmetric matrix)
 * 
 * For vector v = [x, y, z], the cross product matrix [v]× is:
 * [  0  -z   y ]
 * [  z   0  -x ]
 * [ -y   x   0 ]
 * 
 * Such that [v]× * u = v × u
 * 
 * @param v 3D vector
 * @return 3x3 skew-symmetric matrix
 */
inline Matrix3d crossMatrix(const Vector3d& v) {
    Matrix3d result;
    result <<     0, -v.z(),  v.y(),
              v.z(),      0, -v.x(),
             -v.y(),  v.x(),      0;
    return result;
}

/**
 * Transform a pose error (twist) from one frame to another
 * 
 * This is the same as wrench transformation since both use adjoint
 * 
 * @param twist_A Twist expressed in frame A [vx, vy, vz, wx, wy, wz]
 * @param X_BA Transform from frame A to frame B
 * @return Twist expressed in frame B
 */
inline Vector6d transformTwist(const Vector6d& twist_A, const Transform& X_BA) {
    // Twist transformation uses the same adjoint as wrench
    return transformWrench(twist_A, X_BA);
}

/**
 * Compute gravity compensation torque
 * 
 * When a force is applied at a distance from the reference point,
 * it creates a torque: τ = p × F
 * 
 * @param p_CoM Position of center of mass relative to reference
 * @param F_gravity Gravity force vector
 * @return Gravity-induced torque
 */
inline Vector3d gravityTorque(const Vector3d& p_CoM, const Vector3d& F_gravity) {
    return p_CoM.cross(F_gravity);
}

} // namespace ur_admittance_controller::spatial