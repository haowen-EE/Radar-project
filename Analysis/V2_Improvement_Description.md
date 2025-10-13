# AAA_TEST V2 Improvement Description

## Focus of This Improvement: Resolve Tracking Disconnection and Choppiness Issues

### Root Cause Analysis
Based on CSV data analysis, found the following key issues:
1. **22-35% of frames have <10 points** - Extreme sparsity causes tracking disconnection
2. **Minimum 1 point/frame** - Almost impossible to cluster
3. **Velocity discretization** - 0.44m/s intervals may cause association failures
4. **High-speed + sparse** - 6m/s × 0.1s × multi-frame miss = easily exceeds association gate

### V2 Version Improvement List

#### 1. Core Parameter Adjustments
```python
GRID_CELL_M = 0.5        # 0.7 → 0.5 finer clustering
ASSOC_GATE_BASE_M = 3.0   # 2.4 → 3.0 tolerate larger movement
MAX_MISS = 15             # 8 → 15 (1.5s tolerance)
LATCH_S = 2.0            # 1.0 → 2.0 extend retention
SR_LATCH_S = 2.0         # 1.0 → 2.0 scooter extend retention
SR_MAX_SPEED_JUMP = 6.0  # 4.0 → 6.0 relax speed jump
```

#### 2. Adaptive Association Gate Enhancement
- ✅ Add **miss compensation**: `v * dt * miss_count * 1.5`
- ✅ High-speed magnification factor: `1.3 → 1.5`
- ✅ When miss≥5: `× 2.0` significant magnification
- ✅ Minimum gate: `1.5m → 2.0m`

```python
final_gate = max(base_gate, speed_gate + miss_compensation, 2.0)
if miss_count >= 5:
    final_gate *= 2.0
```

#### 3. Position Prediction Optimization
- ✅ Use **3-frame average velocity** (more stable)
- ✅ Consider miss count: `total_dt = dt * (1 + miss_count)`
- ✅ Multi-frame extrapolation for tracking recovery

#### 4. Sparse Frame Special Handling (New)
When point count <10:
- ✅ Partial miss offset: `miss = max(0, miss - 0.5)`
- ✅ Extend latch: `+0.5 seconds`
- ✅ Reduce false deletion probability

```python
def handle_sparse_frame(tracks, scooter_rider_tracks, now, total_points):
    if total_points < 10:  # Sparse threshold
        # Reduce miss penalty
        # Extend retention time
```

#### 5. Interface Enhancement
- ✅ Display miss count: `[M:3]`
- ✅ Mark sparse frames: `[SPARSE]`
- ✅ Display current point count: `Pts:7`

### Expected Improvement Effects

| Metric | V1 Version | V2 Version | Improvement |
|--------|-----------|-----------|-------------|
| Continuous tracking rate | ~60% | ~95% | +58% |
| Max disconnection time | 0.8s | 1.5s tolerance | +87% |
| Sparse frame handling | None | Smart penalty reduction | New feature |
| Association gate | Fixed 2.4m | Dynamic 2-12m | Adaptive |
| Miss recovery | Difficult | Prediction + extrapolation | Enhanced |

### Test Scenarios

#### Scenario 1: syntropy_bike&people.csv (Same Direction Overtaking)
- Frames 15831-17300: Extremely sparse point cloud
- Expectation: Scooter tracked throughout without disconnection
- Expectation: Pedestrians correctly recognized, not misjudged

#### Scenario 2: subtend_bike&people.csv (Head-on Encounter)
- Frames 13326-13845: Complex encounter scene
- Expectation: Two targets clearly distinguished
- Expectation: No ID swap during encounter

### Key Log Observations

Focus during runtime:
```
[CONVERT-L1] - High-speed conversion
[CONVERT-L2] - Medium-speed strong feature conversion
[REJECT-ASSOC] - Association validation rejection
[OCCLUDE] - Occlusion detection
[LOCK] - Identity lock
```

Window title displays:
```
Pts:7 [SPARSE] - Sparse frame
Ped:1 SR:1 - Current target count
[M:3] - Miss count
```

### Debugging Recommendations

If still having issues:
1. Check if miss count frequently >5
2. Observe [SPARSE] occurrence frequency
3. Pay attention to [REJECT-ASSOC] reasons
4. Check if association gate is large enough

### Further Optimization Directions

If V2 still not perfect:
- [ ] Implement cluster post-merge (merge close same-speed clusters)
- [ ] Kalman filter prediction (more precise)
- [ ] Multi-hypothesis tracking (handle ambiguity)
- [ ] Point cloud sparsity adaptive parameters
