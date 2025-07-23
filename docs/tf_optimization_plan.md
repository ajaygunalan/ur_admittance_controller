# TF Lookup Performance Optimization Plan

## Executive Summary

**Issue**: TF lookups performed every callback at 100Hz in wrench_node  
**Impact**: Low (≤1% CPU overhead) but suboptimal  
**Solution**: Share FK results between nodes to eliminate redundant computation  
**Timeline**: 0.5-1 day  
**Risk**: Low - preserves existing interfaces

## Current State

```cpp
// wrench_node.cpp - EVERY callback at 100Hz
void wrench_callback(msg) {
    X_EB_ = tf2::transformToEigen(
        tf_buffer_->lookupTransform("tool0", "base_link", tf2::TimePointZero)
    );
}
```

**Key Facts**:
- Transform changes at 500Hz (UR robot update rate)
- Admittance node already computes FK at 100Hz
- TF lookup cost: ~0.1-1ms per call
- Both nodes need the same transform

## Analysis

### Is This Really a Problem?

**No, it's not critical**:
- TF2 has internal caching - lookups are O(1) for recent transforms
- At 100Hz, overhead is ≤10% of cycle time
- UR drivers publish transforms at 500Hz via TF

**But it's inefficient**:
- Computing FK twice (admittance node + TF tree)
- Unnecessary complexity for data already available
- Violates DRY principle

### Should We Fix It?

**Yes, but with minimal changes**:
- Easy win for code clarity
- Reduces coupling to TF system
- Better for future real-time requirements

## Proposed Solution

### Option 1: Direct Transform Sharing (Recommended)
**Effort**: 2-4 hours

Since admittance_node already computes X_BP (base→payload):

```cpp
// admittance_node.cpp
void control_cycle() {
    // Already computing this
    get_X_BP_current();  // FK computation
    
    // NEW: Publish for wrench_node
    publish_transform(X_BP_current);
}

// wrench_node.cpp  
void setup() {
    // NEW: Subscribe to transform
    transform_sub_ = create_subscription<TransformMsg>(
        "/admittance/tool_transform", ...);
}

void wrench_callback(msg) {
    // Use cached transform instead of TF lookup
    X_EB_ = latest_transform_;  
}
```

### Option 2: Keep TF but Optimize
**Effort**: 1 hour

Add timeout and pre-check:

```cpp
void wrench_callback(msg) {
    // Add 1ms timeout to prevent blocking
    if (tf_buffer_->canTransform("tool0", "base_link", tf2::TimePointZero)) {
        X_EB_ = tf2::transformToEigen(
            tf_buffer_->lookupTransform("tool0", "base_link", 
                                      tf2::TimePointZero,
                                      tf2::durationFromSec(0.001)));  // 1ms timeout
    } else {
        // Use previous transform
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Transform not available, using previous");
    }
}
```

## Implementation Steps

### For Option 1 (Recommended):

1. **Define transform message** (30 min):
   ```cpp
   // Create custom msg or use geometry_msgs/TransformStamped
   ```

2. **Update admittance_node** (1 hour):
   - Add publisher for tool transform
   - Publish after FK computation
   - Ensure 100Hz publishing rate

3. **Update wrench_node** (1 hour):
   - Add subscriber for transform
   - Cache latest transform
   - Fall back to TF if no messages

4. **Test** (30 min):
   - Verify transform freshness
   - Check CPU usage reduction
   - Ensure backward compatibility

## Decision Matrix

| Criteria | Option 1 (Share) | Option 2 (Optimize TF) | Do Nothing |
|----------|-----------------|---------------------|------------|
| Performance | ✅ Best | ✅ Good | ⚠️ OK |
| Complexity | ⚠️ Medium | ✅ Low | ✅ None |
| Maintainability | ✅ Clear data flow | ⚠️ Still coupled | ⚠️ Redundant |
| Real-time ready | ✅ Yes | ⚠️ Maybe | ❌ No |
| Effort | 4 hours | 1 hour | 0 hours |

## Recommendation

**Implement Option 1** because:
- Eliminates redundant FK computation
- Creates explicit data dependency
- Prepares for future real-time requirements
- Minimal code changes with clear benefits

**Skip if**:
- System has no performance issues
- Planning major architecture changes soon
- Team unfamiliar with transform handling

## Code Changes Summary

```diff
// admittance_node.hpp
+ rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;

// admittance_node.cpp
+ transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
+     "/admittance/tool_transform", rclcpp::SystemDefaultsQoS());

  void control_cycle() {
      get_X_BP_current();
+     auto msg = tf2::eigenToTransform(X_BP_current);
+     msg.header.stamp = now();
+     msg.header.frame_id = "base_link";
+     msg.child_frame_id = "tool0";
+     transform_pub_->publish(msg);
  }

// wrench_node.hpp  
+ rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_sub_;
+ std::atomic<bool> has_transform_{false};

// wrench_node.cpp
- X_EB_ = tf2::transformToEigen(
-     tf_buffer_->lookupTransform("tool0", "base_link", tf2::TimePointZero));
+ if (has_transform_) {
+     // Use cached transform
+ } else {
+     // Fallback to TF
+ }
```

## Conclusion

While not critical, this optimization:
- Reduces redundant computation
- Improves code clarity  
- Costs only 4 hours to implement
- Sets foundation for real-time operation

**Verdict**: Worth implementing during next maintenance window.