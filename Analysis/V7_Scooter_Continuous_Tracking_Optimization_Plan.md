# V7 Scooter Continuous Tracking Optimization Plan

**Date**: 2025-10-13  
**Version**: V7 (Based on V6)  
**Objective**: Optimize scooter recognition and continuous tracking, keep pedestrian recognition unchanged

---

## Problem Analysis

### User Feedback
1. **Brief Recognition Issue** - "Can be briefly detected, but cannot be continuously tracked"
2. **High-Speed Loss Issue** - "At high speed, electric scooter passing by pedestrian has no recognition trace"
3. **Requirement** - Keep pedestrian recognition unchanged, only optimize scooter part

### Root Cause Analysis

#### Issue 1: V6 Parameters Too Conservative Leading to Scooter Non-Recognition
**V6 Current State**:
- L0 fast lane: v≥4.5 + cy≥0.90
- L1 high speed: v≥3.0 + cy≥0.90
- L2 medium speed: v≥2.0 + cy≥1.0 + dur≥1.5s
- is_scooter_rider enforces cy≥0.90 (regardless of strict)

**Analysis**:
- V6 designed as extremely conservative strategy to prevent pedestrian false positives
- But this causes real scooters also unable to be recognized
- Especially cy≥0.90 hard requirement may be too high
- Based on actual data, scooter cy range is wide (0.5-1.5m), depends on distance and angle

#### Issue 2: Inertia System Too Strict
**V6 Inertia System**:
```python
INERTIA_ANGLE_THRESHOLD = 0.5  # cos(60°)
INERTIA_ERROR_FACTOR = 2.0
```

**Problem**:
- High-speed scooters easily rejected by inertia system during turns or acceleration
- 60° angle may be too strict for 10fps sampling rate
- 2x prediction error not enough for acceleration/deceleration scenarios

#### Issue 3: MAX_MISS Too Small Causing Tracking Interruption
**V6 State**:
- MAX_MISS_SR = 5 frames (0.5 seconds)

**Problem**:
- High-speed scooter point clouds often sparse (far distance, fast movement)
- 5 consecutive frame misses trigger cleanup, too strict for sparse scenarios
- Original V5's 12 frames actually more reasonable

#### Issue 4: Association Gate Too Small
**V6 State**:
- ASSOC_GATE_BASE_M = 2.0m

**Problem**:
- High-speed scooter 0.1s movement distance = 5-6m/s × 0.1s = 0.5-0.6m
- Plus adaptive expansion 1.8x = 2.0 × 1.8 = 3.6m
- But for extremely high speed (6m/s+) may still not be enough

---

## V7 Solution

### Core Principle
**Dual-Track Strategy**:
- **Pedestrian Recognition**: Maintain V6 strict strategy (prevent false positives)
- **Scooter Recognition**: Relax strategy (priority continuous tracking)

### Solution 1: Lower Scooter cy Threshold (for is_scooter_rider)

#### Modification Strategy
```python
# V6 (too strict):
cy_min = SR_CENTROID_Y_MIN  # Uniform 0.90m

# V7 (distinguish by scenario):
# During conversion check (Track→SR): Strict, cy≥0.85 (prevent pedestrian false positives)
# During association check (SR association): Relaxed, cy≥0.60 (maintain tracking)
```

#### Implementation
```python
def is_scooter_rider(cluster, npts, strict=True, cy_hint=None, for_tracking=False):
    """
    for_tracking=True: For existing SR trajectory association check (relaxed)
    for_tracking=False: For new SR or Track→SR conversion (strict)
    """
    # ... other checks unchanged ...
    
    if for_tracking:
        # Association check: relaxed cy requirement (0.60), maintain tracking
        cy_min = 0.60
        cy_max = SR_CENTROID_Y_MAX * 1.5
    else:
        # Conversion check: strict cy requirement (0.85), prevent false positives
        cy_min = 0.85  # Lowered from 0.90 to 0.85
        cy_max = SR_CENTROID_Y_MAX
    
    if not (cy_min <= cy <= cy_max):
        return False
    
    # ... other checks ...
```

### Solution 2: Increase MAX_MISS_SR, Restore V5 Setting

```python
# V6 (too strict):
MAX_MISS_SR = 5  # 0.5 seconds

# V7 (restore reasonable value):
MAX_MISS_SR = 12  # 1.2 seconds, consistent with V5
```

**Rationale**:
- High-speed scooter sparse point clouds are the norm
- Need longer tolerance time to maintain tracking
- V5's 12 frames tested to be reasonable

### Solution 3: Relax Inertia System (Scooter Only)

#### Modification Strategy
```python
# V6 (uniformly strict):
INERTIA_ANGLE_THRESHOLD = 0.5  # cos(60°)
INERTIA_ERROR_FACTOR = 2.0

# V7 (scooter-specific relaxed):
INERTIA_ANGLE_THRESHOLD_SR = 0.3  # cos(70°), allow larger turns
INERTIA_ERROR_FACTOR_SR = 3.0     # 3x error, adapt to acceleration
```

#### Implementation
```python
def validate_inertia(track, new_cx, new_cz, dt, for_scooter=False):
    """
    for_scooter=True: Use relaxed scooter parameters
    for_scooter=False: Use strict pedestrian parameters
    """
    # ... previous calculations unchanged ...
    
    # Select parameters
    if for_scooter:
        angle_threshold = INERTIA_ANGLE_THRESHOLD_SR  # 0.3
        error_factor = INERTIA_ERROR_FACTOR_SR  # 3.0
    else:
        angle_threshold = INERTIA_ANGLE_THRESHOLD  # 0.5
        error_factor = INERTIA_ERROR_FACTOR  # 2.0
    
    # Angle check
    if cos_angle < angle_threshold:
        inertia_score = max(0.0, cos_angle)
    
    # Prediction error check
    max_error = hist_speed * dt * error_factor
    # ... subsequent logic ...
```

### Solution 4: Increase Scooter Association Gate

```python
# Use larger gate multiplier in ScooterRider association
# V6:
sr_gate = adaptive_association_gate(sr, assoc_gate * 1.8, dt_med)

# V7:
sr_gate = adaptive_association_gate(sr, assoc_gate * 2.5, dt_med)
# 2.0m × 2.5 = 5.0m base gate, can reach 6-7m at high speed
```

### Solution 5: Lower Conversion Thresholds (Balanced Strategy)

```python
# V6 conversion strategy (too conservative):
# L0: v≥4.5 + cy≥0.90
# L1: v≥3.0 + cy≥0.90
# L2: v≥2.0 + cy≥1.0 + dur≥1.5s

# V7 conversion strategy (balanced):
# L0: v≥4.0 + cy≥0.75  # Lower cy requirement
# L1: v≥2.8 + cy≥0.85  # Slightly lower speed and cy
# L2: v≥2.0 + cy≥0.90 + dur≥1.0s  # Lower dur requirement
```

**Rationale**:
- L0 for extremely high speed: cy=0.75 can cover distant fast scooters
- L1 for high speed: cy=0.85 is reasonable boundary between pedestrians (0.3-0.6) and scooters (0.8-1.5)
- L2 medium speed maintains strict (cy=0.90) to avoid pedestrian false positives

---

## Parameter Comparison Table

| Parameter | V6 Value (Conservative) | V7 Value (Balanced) | Change Reason |
|-----------|------------------------|---------------------|---------------|
| **is_scooter_rider** |
| cy_min (conversion) | 0.90m | 0.85m | Slightly relax, cover more scenarios |
| cy_min (association) | 0.90m | 0.60m | Significantly relax, maintain tracking |
| **Conversion Strategy** |
| L0 speed | 4.5m/s | 4.0m/s | Lower threshold |
| L0 cy | 0.90m | 0.75m | Distant scooters |
| L1 speed | 3.0m/s | 2.8m/s | Slightly lower |
| L1 cy | 0.90m | 0.85m | Balance point |
| L2 cy | 1.0m | 0.90m | Unify with L1 |
| L2 duration | 1.5s | 1.0s | Faster conversion |
| **Tracking Parameters** |
| MAX_MISS_SR | 5 frames | 12 frames | Restore V5, more tolerant |
| Association gate multiplier | 1.8 | 2.5 | High speed needs larger |
| MAX_SPEED_JUMP | 6.0m/s | 8.0m/s | Adapt to acceleration |
| **Inertia System** |
| Angle threshold(SR) | 0.5 | 0.3 | Allow larger turns |
| Error factor(SR) | 2.0 | 3.0 | Adapt to acceleration |
| **Display** |
| Point cloud check(SR) | Enabled | Disabled | Reduce flickering |

---

## Summary

### V7 Core Idea
**"Dual-Track" Strategy**:
- Pedestrian part: Maintain V6 strict strategy (prevent false positives priority)
- Scooter part: Relax tracking strategy (continuity priority)

### Key Improvements
1. **Dual-stage cy requirement**: Conversion strict (0.85), association relaxed (0.60)
2. **Restore reasonable tolerance**: MAX_MISS_SR from 5 restored to 12
3. **Relax inertia system**: Scooter-specific relaxed parameters
4. **Increase association gate**: 2.5x base gate
5. **Lower conversion threshold**: L0/L1 easier to trigger

### Balance with V6
- V6 solved pedestrian false positive problem ✓
- V7 maintains V6 advantages while restoring scooter tracking capability
- Achieves "having cake and eating it too" through dual-track system

---

**File**: AAA_SCOOTER_OPTIMIZED.py  
**Based on**: AAA_TEST_IMPROVED_COMPLETE.py (V6)  
**Modified Lines**: Approximately 150 lines (parameters + function modifications)
