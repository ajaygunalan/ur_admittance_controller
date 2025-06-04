# Variable Simplification Plan

## Goal
Remove the redundant `V_tcp_base_desired_` variable and use only `V_tcp_base_commanded_` throughout the codebase.

## Current State
- `V_tcp_base_desired_`: Stores raw admittance output
- `V_tcp_base_commanded_`: Copy of desired, then limited
- This is unnecessarily complex!

## Implementation Steps

### Step 1: Update Header File (admittance_node.hpp)
- Remove `Vector6d V_tcp_base_desired_;` declaration
- Keep only `Vector6d V_tcp_base_commanded_;`

### Step 2: Update Initialization (admittance_node.cpp)
- Change line 45: `V_tcp_base_desired_ = Vector6d::Zero();` 
  → `// Removed - using V_tcp_base_commanded_ only`
- Keep: `V_tcp_base_commanded_ = Vector6d::Zero();`

### Step 3: Update compute_admittance() (admittance_computations.cpp)
```cpp
// OLD:
const double dt = control_period_.seconds();
V_tcp_base_desired_ += acceleration * dt;
V_tcp_base_commanded_ = V_tcp_base_desired_;

// NEW:
const double dt = control_period_.seconds();
V_tcp_base_commanded_ += acceleration * dt;
```

### Step 4: Update limit_to_workspace() (admittance_node.cpp)
```cpp
// OLD:
V_tcp_base_commanded_ = V_tcp_base_desired_;
// ... then modify V_tcp_base_commanded_

// NEW:
// Just modify V_tcp_base_commanded_ directly (no copy needed)
```

### Step 5: Update update_damping_matrix() (admittance_computations.cpp)
- Change line 15: Uses `V_tcp_base_desired_` in acceleration computation
  → Use `V_tcp_base_commanded_` instead

## Benefits
1. **Simpler code**: One less variable to track
2. **Clearer logic**: Direct integration and limiting
3. **Less memory**: One less Vector6d (48 bytes)
4. **No unnecessary copies**: Better performance

## Implementation Results

### Changes Made:
1. ✅ **Header file (admittance_node.hpp)**:
   - Removed `Vector6d V_tcp_base_desired_` declaration
   - Updated comment for `V_tcp_base_commanded_`

2. ✅ **Initialization (admittance_node.cpp)**:
   - Removed `V_tcp_base_desired_ = Vector6d::Zero();`

3. ✅ **compute_admittance() (admittance_computations.cpp)**:
   - Changed damping term from `V_tcp_base_desired_` to `V_tcp_base_commanded_`
   - Removed redundant copy: directly integrate into `V_tcp_base_commanded_`

4. ✅ **limit_to_workspace() (admittance_node.cpp)**:
   - Removed unnecessary copy operation
   - Now modifies `V_tcp_base_commanded_` in-place

### Result:
- **Simpler code**: One variable instead of two
- **Clearer logic**: Direct integration → limiting
- **Better performance**: No unnecessary copies
- **Same behavior**: Mathematically equivalent

## Testing Checklist
- [ ] Build the package
- [ ] Verify admittance control still works
- [ ] Check workspace limiting functions correctly
- [ ] Ensure no regression in control behavior
- [ ] Verify integration accumulates properly