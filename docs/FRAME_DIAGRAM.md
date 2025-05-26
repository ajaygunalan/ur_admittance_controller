# UR Admittance Controller - Frame Relationships Diagram

## Visual Frame Hierarchy

```
                              WORLD FRAME (W)
                              [Optional - often same as base]
                                      |
                                      | X_W_base (fixed)
                                      |
                              BASE FRAME (base)
                              [base_link - robot mount]
                                      |
                                      | X_base_J1
                                      |
                              Joint 1 (shoulder_pan)
                                      |
                                      | X_J1_J2
                                      |
                              Joint 2 (shoulder_lift)
                                      |
                                      | X_J2_J3
                                      |
                              Joint 3 (elbow)
                                      |
                                      | X_J3_J4
                                      |
                              Joint 4 (wrist_1)
                                      |
                                      | X_J4_J5
                                      |
                              Joint 5 (wrist_2)
                                      |
                                      | X_J5_J6
                                      |
                              Joint 6 (wrist_3)
                                      |
                                      | X_J6_tip
                                      |
                              TIP FRAME (tip)
                              [tool0 - end effector]
                                      |
                                      | X_tip_ft (often identity)
                                      |
                              F/T SENSOR FRAME (ft)
                              [Force/torque sensor]
```

## Key Transforms Used in Controller

### Primary Control Transform
```
X_base_tip = X_base_J1 * X_J1_J2 * ... * X_J6_tip
           = Forward Kinematics(joint_positions)
```

### Force Sensor Transform (Special Case)
```
When ft_frame == base_link:
    F_sensor_base = raw_wrench  (no transform needed)
    
When ft_frame == tool0:
    F_sensor_base = Adjoint(X_base_tip) * F_sensor_tip
```

## Transform Update Flow

```
1. Read joint positions from hardware
   ↓
2. Compute forward kinematics
   ↓
3. Update X_base_tip
   ↓
4. Transform sensor data if needed
   ↓
5. Compute control in base frame
   ↓
6. Send commands to joints
```

## Spatial Vector Transformations

### Velocity Transform
```
V_base_tip_base = Adjoint(X_base_tip) * V_tip_tip_tip
                = [R_base_tip    0       ] * [ω_tip]
                  [p̂_base_tip  R_base_tip]   [v_tip]
```

### Force Transform  
```
F_tip_base = Adjoint^T(X_base_tip) * F_base_base
           = [R_base_tip^T  -R_base_tip^T * p̂_base_tip] * [τ_base]
             [0             R_base_tip^T               ]   [f_base]
```

Where p̂ is the skew-symmetric matrix of position vector p.