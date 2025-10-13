# V6 Improvement Plan: Inertia System and Strict Cleanup Mechanism

## Problem Analysis

### Current Issues (User Feedback)
1. **Old Box Trace Lingering** - Boxes still exist in blank spaces after point clouds disappear
2. **Box Attraction Issue** - Boxes transfer to nearby newly appearing point clouds
3. **Pedestrian False Recognition** - Walking or running pedestrians easily identified as scooters

### Root Causes

#### Issue 1: Box Lingering
- **Current State**: MAX_MISS=8 frames (0.8s), LATCH mechanism additionally extends display
- **Analysis**: For walk.csv pure pedestrian data, boxes shouldn't persist long
- **AAA_Final Advantage**: Faster cleanup mechanism

#### Issue 2: Box Attraction (Most Serious)
- **Current State**: Association gate too large (ASSOC_GATE_BASE_M=3.0), trajectories associate to point clouds 3m away
- **Analysis**: Lacks **inertia constraints** - trajectories should continue along historical motion direction, not jump arbitrarily
- **AAA_Final Advantage**: Smaller association gate (2.4m), stricter association validation

#### Issue 3: Pedestrian False Recognition
- **Current State**: Although SR_CENTROID_Y_MIN=0.90, conversion logic still loose
- **Analysis**:
  - L0 fast lane: v≥4.0 + cy≥0.60, but pedestrians running might reach 4m/s
  - L1 high speed: v≥2.5 + cy≥0.70, but cy<0.90 is reasonable
  - In is_scooter_rider() when strict=False, cy_min drops to 0.72
- **AAA_Final Advantage**: No scooter recognition, only focuses on pedestrians

## Solution

### Core Principle: Borrow AAA_Final's Conservative Strategy

AAA_Final advantages:
1. **Smaller association gate** (2.4m vs 3.0m)
2. **No complex conversion logic** - Only recognizes pedestrians, no false positives
3. **Fast cleanup** - Simple miss mechanism
4. **Strict geometric constraints** - Pedestrians must satisfy specific dimensions and velocity

### Solution A: Add Inertia Constraint System (Recommended)

#### 1. Velocity Inertia Check
```python
def validate_inertia(track, new_cx, new_cz, dt):
    """
    Inertia validation: new position must be consistent with historical motion direction
    
    Returns: (is_valid, inertia_score)
    - is_valid: True=passes inertia check
    - inertia_score: 0-1, larger means more conforming to inertia
    """
    if len(track.centroids) < 3:
        return True, 1.0  # History too short, cannot judge
    
    # Calculate historical velocity vector (last 3 frames)
    recent_centroids = list(track.centroids)[-3:]
    recent_times = list(track.times)[-3:]
    
    # Historical direction vector
    hist_dx = recent_centroids[-1][0] - recent_centroids[0][0]
    hist_dz = recent_centroids[-1][1] - recent_centroids[0][1]
    hist_dt = recent_times[-1] - recent_times[0]
    
    if hist_dt <= 0:
        return True, 1.0
    
    # Historical velocity
    hist_vx = hist_dx / hist_dt
    hist_vz = hist_dz / hist_dt
    hist_speed = math.sqrt(hist_vx**2 + hist_vz**2)
    
    # Predicted position (based on inertia)
    pred_x = recent_centroids[-1][0] + hist_vx * dt
    pred_z = recent_centroids[-1][1] + hist_vz * dt
    
    # New position vector
    new_dx = new_cx - recent_centroids[-1][0]
    new_dz = new_cz - recent_centroids[-1][1]
    new_dist = math.sqrt(new_dx**2 + new_dz**2)
    
    # Prediction error
    pred_error = math.sqrt((new_cx - pred_x)**2 + (new_cz - pred_z)**2)
    
    # If historical speed very small (stationary or slow), relax check
    if hist_speed < 0.5:
        return True, 1.0
    
    # Direction consistency check
    if hist_speed > 0.5 and new_dist > 0.1:
        # Historical direction unit vector
        hist_dir_x = hist_dx / (math.sqrt(hist_dx**2 + hist_dz**2) + 1e-6)
        hist_dir_z = hist_dz / (math.sqrt(hist_dx**2 + hist_dz**2) + 1e-6)
        
        # New direction unit vector
        new_dir_x = new_dx / new_dist
        new_dir_z = new_dz / new_dist
        
        # Direction cosine (dot product)
        cos_angle = hist_dir_x * new_dir_x + hist_dir_z * new_dir_z
        
        # Angle diff > 90°, clearly reverse -> reject
        if cos_angle < -0.1:  # cos(95°) ≈ -0.087
            return False, 0.0
        
        # Angle diff > 60°, suspicious -> lower score
        if cos_angle < 0.5:  # cos(60°) = 0.5
            inertia_score = cos_angle  # 0-0.5 range
        else:
            inertia_score = 1.0
    else:
        inertia_score = 1.0
    
    # Prediction error check
    # Maximum allowed deviation = historical speed × dt × tolerance factor
    max_error = hist_speed * dt * 2.0  # Tolerance factor 2.0
    
    if pred_error > max_error:
        # Too far from prediction
        error_ratio = max_error / (pred_error + 1e-6)
        inertia_score *= error_ratio
        
        if pred_error > max_error * 2.0:
            return False, 0.0
    
    return True, inertia_score
```

#### 2. Modify Association Logic
```python
# Add inertia check in ScooterRider association
if best is not None:
    cx, cz = best["centroid_xz"]
    
    # 【NEW】Inertia validation
    inertia_valid, inertia_score = validate_inertia(sr, cx, cz, dt_med)
    
    if not inertia_valid:
        print(f"  [INERTIA-REJECT] SR#{sr.id} inertia check failed score={inertia_score:.2f}")
        best = None
        best_ci = None
    else:
        # Passed inertia check, continue other validations
        if validate_association(sr, best, abs_t[idx], SR_MAX_SPEED_JUMP):
            used[best_ci] = True
            # ... normal association
        else:
            print(f"  [REJECT-ASSOC] SR#{sr.id} association validation failed")
```

#### 3. Reduce Association Gate
```python
# Current: ASSOC_GATE_BASE_M = 3.0 (too large)
# Change to:
ASSOC_GATE_BASE_M = 2.0  # Consistent with AAA_Final
```

#### 4. Stricter Scooter Judgment
```python
# Modify L0 fast lane, raise cy requirement
if v >= 4.0 and cy >= 0.80:  # Raised from 0.60 to 0.80
    if tr.last_npts >= 5:
        # High-speed lane

# Modify L1, ensure cy>=SR_CENTROID_Y_MIN
if v >= 2.5:
    if is_scooter_rider(...) and cy >= SR_CENTROID_Y_MIN:  # Raised from 0.70 to 0.90
        # Convert

# Modify is_scooter_rider, remove cy_min lowering logic
def is_scooter_rider(cluster, npts, strict=True, cy_hint=None):
    # ...
    cy_min = SR_CENTROID_Y_MIN  # No longer lowered
    cy_max = SR_CENTROID_Y_MAX
    if not (cy_min <= cy <= cy_max):
        return False
```

---

## Recommended Implementation Order

### Step 1 (Immediate Implementation): Inertia System + Reduce Gate
1. Add `validate_inertia()` function
2. Call inertia validation in association logic
3. Change `ASSOC_GATE_BASE_M` from 3.0 to 2.0
4. Add inertia check in SR association

**Expected Effect**: Boxes won't jump to distant point clouds

### Step 2 (Based on Results): Strictify Scooter Judgment
1. Raise L0 fast lane cy threshold (0.60→0.80)
2. Raise L1 conversion cy threshold (0.70→0.90)
3. Remove cy_min lowering logic in is_scooter_rider

**Expected Effect**: Pedestrians won't be falsely identified as scooters

### Step 3 (Optional): More Aggressive Cleanup
1. Reduce MAX_MISS (8→3)
2. Add mandatory point cloud existence check

**Expected Effect**: Boxes disappear quickly

---

## Key Code Locations

### Files to Modify
- `AAA_TEST_IMPROVED_COMPLETE.py`

### Key Functions
1. **NEW**: `validate_inertia()` - Inertia validation
2. **MODIFY**: Association logic in `update()` (~900-950 lines)
3. **MODIFY**: `is_scooter_rider()` - Remove cy_min lowering
4. **MODIFY**: Conversion logic (~975-1010 lines)

### Parameter Adjustments
```python
# Group 1: Association gate
ASSOC_GATE_BASE_M = 2.0  # Changed from 3.0 to 2.0

# Group 2: Inertia parameters (new)
INERTIA_MIN_HISTORY = 3  # Minimum 3 frames history to enable inertia check
INERTIA_ANGLE_THRESHOLD = 0.5  # cos(60°), direction deviation threshold
INERTIA_ERROR_FACTOR = 2.0  # Prediction error tolerance factor

# Group 3: Scooter judgment (strictify)
SR_CENTROID_Y_MIN = 0.90  # Maintain
L0_CY_MIN = 0.80  # L0 fast lane cy threshold (raised from 0.60)
L1_CY_MIN = 0.90  # L1 conversion cy threshold (raised from 0.70)

# Group 4: Cleanup parameters (optional)
MAX_MISS = 3  # Changed from 8 to 3 (aggressive mode)
MAX_MISS_SR = 5  # Changed from 12 to 5 (aggressive mode)
```
