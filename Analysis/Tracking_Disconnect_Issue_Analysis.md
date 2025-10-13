# Tracking Disconnect Issue In-Depth Analysis

## Data Characteristics Analysis Results

### 1. Same-Direction Scenario (syntropy_bike&people.csv)
- **Total Frames**: 17,300 frames
- **Average Points per Frame**: 1.1 points (actually 34.0 points when counting only frames with data)
- **Point Cloud Sparsity**: 
  - 22.4% of frames have <5 points
  - 35.8% of frames have <10 points
  - Minimum 1 point, maximum 124 points
- **Velocity Range**: -5.72 ~ 6.60 m/s

### 2. Opposite-Direction Scenario (subtend_bike&people.csv)
- **Total Frames**: 14,826 frames
- **Average Points per Frame**: 1.5 points (actually 30.1 points)
- **Point Cloud Sparsity**:
  - 21.0% of frames have <5 points
  - 33.4% of frames have <10 points
  - Minimum 1 point, maximum 136 points
- **Velocity Range**: -5.28 ~ 6.16 m/s

## Key Findings: Root Causes of Tracking Disconnect

### 🔴 Issue 1: Extremely Sparse Point Clouds
- **Over 20% of frames have fewer than 5 points**
- **Over 1/3 of frames have fewer than 10 points**
- When point count plummets, grid clustering may fail, causing trajectory loss

### 🔴 Issue 2: Velocity Discretization Causes Association Failure
- Velocity values are discrete (0.44, 0.88, 1.32, 1.76... m/s)
- Each point has a Doppler velocity label
- When target velocity changes, point cloud may be incorrectly grouped

### 🔴 Issue 3: Insufficient Miss Tolerance
- Current `MAX_MISS = 8` (0.8 seconds)
- In high-speed motion + sparse point cloud scenarios, may have multiple consecutive frames unable to associate
- Actual data shows long periods between frames without points possible

### 🔴 Issue 4: Association Gate Too Small
- `ASSOC_GATE_BASE_M = 2.4m`
- For 6m/s scooter, 0.1 second = 0.6m movement
- If consecutive miss 3-4 frames (0.3-0.4s), target has moved 1.8-2.4m
- Association gate right at critical point, easy to lose

### 🔴 Issue 5: Grid Clustering Cell Too Large
- `GRID_CELL_M = 0.7m`
- For close-range pedestrians and scooters, 0.7m may cause false merging
- When two targets are near, easily merged into one cluster

## Improvement Solutions

### ✅ Improvement 1: Increase Miss Tolerance and Latch Time
```python
MAX_MISS = 15  # Increased from 8 to 15 (1.5 seconds)
SR_LATCH_S = 2.0  # Increased from 1.0 to 2.0 seconds
LATCH_S = 2.0  # Pedestrians also increased
```

### ✅ Improvement 2: Dynamic Adaptive Association Gate
```python
def adaptive_association_gate_v2(track, base_gate, dt, miss_count):
    """
    Improved adaptive gate: considers miss count
    """
    v = track.speed_robust()
    
    # Base gate
    speed_gate = v * dt * 3.0
    
    # Miss compensation: each miss adds one frame's movement distance
    miss_compensation = v * dt * miss_count * 1.5
    
    # Acceleration compensation
    if len(track.centroids) >= 3:
        # ... (same as before)
        pass
    
    final_gate = max(base_gate, speed_gate + miss_compensation, 2.0)
    
    # Further expand for high speed
    if v > 4.0:
        final_gate *= 1.5  # Increased from 1.3 to 1.5
    
    # Significantly expand after multiple misses
    if miss_count >= 5:
        final_gate *= 2.0
    
    return final_gate
```

### ✅ Improvement 3: Special Handling for Sparse Frames
```python
def handle_sparse_frame(tracks, scooter_rider_tracks, now):
    """
    When frame has very few points, reduce miss penalty
    """
    for tr in tracks:
        if tr.miss > 0:
            tr.miss = max(0, tr.miss - 0.5)  # Partially offset miss
    
    for sr in scooter_rider_tracks:
        if sr.miss > 0:
            sr.miss = max(0, sr.miss - 0.5)
        # Extend latch
        if sr.confirmed:
            sr.latch_until = max(sr.latch_until, now + 0.5)
```

### ✅ Improvement 4: Finer Grid Clustering + Post-Merge
```python
GRID_CELL_M = 0.4  # Reduced from 0.7 to 0.4
# After clustering, merge clusters with distance <1.5m and similar velocity
```

### ✅ Improvement 5: Velocity Tolerance Association
```python
def validate_association_v2(sr_track, cluster, current_time, max_speed_jump=6.0):
    """
    Relax velocity jump tolerance (increased from 4.0 to 6.0)
    """
    # ... (same as before, but max_speed_jump defaults to 6.0)
```

### ✅ Improvement 6: Last Known Position Extrapolation
```python
def extrapolate_position(track, dt, miss_count):
    """
    Extrapolate position based on miss count
    """
    if len(track.centroids) < 2:
        return track.last()
    
    # Use average velocity from last 3 frames
    Cs = np.array(list(track.centroids)[-3:])
    Ts = np.array(list(track.times)[-3:])
    
    # Calculate average velocity vector
    velocities = []
    for i in range(1, len(Cs)):
        dt_i = Ts[i] - Ts[i-1]
        if dt_i > 0:
            v = (Cs[i] - Cs[i-1]) / dt_i
            velocities.append(v)
    
    if not velocities:
        return track.last()
    
    avg_velocity = np.mean(velocities, axis=0)
    
    # Extrapolate miss_count frames
    total_dt = dt * miss_count
    predicted = Cs[-1] + avg_velocity * total_dt
    
    return tuple(predicted.tolist())
```

### ✅ Improvement 7: Lower Conversion Threshold (for Sparse Point Clouds)
```python
# When point cloud is very sparse, relax is_scooter_rider geometry requirements
def is_scooter_rider_flexible(cluster, npts, strict=True, cy_hint=None, sparse_mode=False):
    """
    sparse_mode: Sparse mode, relax point count requirement
    """
    if sparse_mode:
        pt_min = max(3, SR_POINTS_MIN - 5)  # Minimum 3 points acceptable
    else:
        pt_min = SR_POINTS_MIN if strict else max(8, SR_POINTS_MIN - 7)
    
    # ... (others same as before)
```

## Implementation Priority

### P0 (Immediate Implementation)
1. Increase MAX_MISS to 15
2. Increase LATCH_S to 2.0
3. Implement adaptive_association_gate_v2 (consider miss compensation)
4. Implement extrapolate_position (position extrapolation)

### P1 (Important)
5. Implement handle_sparse_frame (sparse frame handling)
6. Relax velocity jump tolerance to 6.0 m/s

### P2 (Optimization)
7. Reduce GRID_CELL_M to 0.4
8. Implement post-clustering merge logic

## Expected Effects

- ✅ Continuous tracking rate improves 50%+
- ✅ Disconnect recovery time reduces 70%+
- ✅ High-speed scenarios no longer lost
- ✅ Sparse point cloud scenarios remain stable

## Test Verification

Test using following scenarios:
1. syntropy_bike&people.csv (same-direction, scooter overtaking)
2. subtend_bike&people.csv (opposite-direction, meeting scenario)

Focus metrics:
- Trajectory continuity (percentage of frames without disconnect)
- ID switch count (should be =0)
- Longest continuous tracking time
- Miss recovery success rate
