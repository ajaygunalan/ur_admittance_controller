## Core Notation Principles

Drake uses a systematic notation to prevent coordinate frame errors in multibody mechanics. The key insight is that **all quantities are relative** - positions, orientations, and velocities must specify:
1. **What** is being measured
2. **From where** it's measured 
3. **In which frame** it's expressed

## Key Notation Elements

### Position Vectors
- **Typeset**: $^Ap_B$ = position of point B measured from point A
- **Code**: `p_AB` 
- **Full form**: $[^Ap_B]_C$ = position of B from A, expressed in frame C
- **Code**: `p_AB_C`

**Example**: `p_WG_W` = position of gripper origin from world origin, expressed in world frame

### Rotation Matrices
- **Typeset**: $^GR_F$ = orientation of frame F measured from frame G
- **Code**: `R_GF`
- **Usage**: `r_G = R_GF * r_F` (transforms vector from F to G)

**Example**: `R_WB` = orientation of body B relative to world W

### Rigid Transforms (Poses)
- **Typeset**: $^GX_F$ = pose of frame F in frame G
- **Code**: `X_GF`
- **Combines**: rotation `R_GF` and position `p_GF`
- **Composition**: `X_WC = X_WA * X_AB * X_BC`

**Example**: In pick-and-place, move gripper to object:
```
X_WG = X_WO * X_OG_desired
```
Where O = object, G = gripper, W = world

### Spatial Vectors (6D)
Combine rotational (0-2) and translational (3-5) components:
- **Velocity**: V = [ω, v] (angular, linear)
- **Acceleration**: A = [α, a]
- **Force**: F = [τ, f] (torque, force)

**Full notation**: `V_ABo_W` = spatial velocity of body B in frame A, at point Bo, expressed in W

**Example**: `V_WBcm_B` = velocity of body B's center of mass in world, expressed in B

### Special Frames and Points
- **W**: World frame (ground/inertial)
- **B**: Body frame for body B
- **Bo**: Origin of body B
- **Bcm**: Center of mass of body B
- **Bq**: Offset frame - B's orientation but origin at point q

## Notation Shortcuts
1. If expressed-in frame = measured-from frame, omit subscript:
   - `p_W` instead of `p_W_W`
2. If measured from world, can omit the W:
   - `p_B` = `p_WB`
3. For spatial quantities at body origin in same frame:
   - `V_AB` = `V_ABo_A`

## Practical Example: Pick and Place
```python
# Given: object pose X_WO
# Goal: move object to desired pose X_WD

# 1. Move gripper to grasp pose relative to object
X_GO_grasp = ...  # predefined grasp
X_WG = X_WO * X_OG_grasp.inverse()

# 2. After grasping (object fixed to gripper)
X_GO = X_GO_grasp  # stays constant

# 3. Move gripper+object to desired location  
X_WG_final = X_WD * X_GO
```

This notation ensures every transform composition is verifiable by matching adjacent frame labels: W→O * O→G = W→G.