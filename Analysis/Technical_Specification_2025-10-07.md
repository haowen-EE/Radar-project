# E-scooter Recognition Technical Specification
**Version**: v1.0  
**Scope**: IWR6843 mmWave Radar Point Cloud Processing  
**Audience**: Algorithm Engineers, System Integrators

---

## 1. Solution Overview

### 1.1 Core Approach
Adopts a **geometric feature constraints + dynamic accumulation verification** hybrid strategy, screening candidate clusters through multi-dimensional features combined with temporal consistency for final classification.

```
Input: Raw point cloud cluster (x,y,z coordinate set)
   ↓
Geometric feature extraction (bbox, centroid, area, ratio)
   ↓
Strict geometric filtering (8 constraints)
   ↓
Trajectory consistency accumulation (geom_hits counter)
   ↓
Velocity threshold verification (dual-channel)
   ↓
Output: Track → ScooterRider conversion
```

### 1.2 Technical Advantages
- ✅ **No training data needed**: Pure rule-driven, rapid deployment
- ✅ **Strong real-time performance**: Single-frame judgment + lightweight tracking
- ✅ **High interpretability**: Each parameter corresponds to physical meaning
- ✅ **Good robustness**: Accumulation mechanism suppresses single-frame noise

---

## 2. Parameter Definitions and Physical Meaning

### 2.1 Geometric Parameters (unit: meters)

| Parameter | Value | Meaning | Physical Interpretation |
|-----------|-------|---------|------------------------|
| `SR_WIDTH_MIN` | 0.35 | Minimum width (x-direction) | Scooter deck width ~30-50cm |
| `SR_WIDTH_MAX` | 1.60 | Maximum width | Considers body+vehicle lateral span |
| `SR_DEPTH_MIN` | 0.15 | Minimum depth (z-direction) | Handlebar stem depth |
| `SR_DEPTH_MAX` | 1.10 | Maximum depth | Body front-back span + vehicle length |
| `SR_HEIGHT_MIN` | 0.10 | Minimum height (y-direction) | bbox vertical envelope lower limit |
| `SR_HEIGHT_MAX` | 0.80 | Maximum height | bbox vertical envelope upper limit |
| `SR_CENTROID_Y_MIN` | 1.30 | Minimum centroid height | Cluster center ordinate from radar perspective |
| `SR_CENTROID_Y_MAX` | 3.30 | Maximum centroid height | Avoid abnormally high points |
| `SR_TOP_Y_MIN` | 1.80 | Minimum top height | bbox ymax lower limit |

**Important Note**:  
- All y-coordinates are in radar coordinate system, with radar installation height as zero reference
- Centroid height `centroid_y = (ymin + ymax) / 2`
- bbox height `height = ymax - ymin`, **not** the object's true physical height

### 2.2 Point Count and Area

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SR_POINTS_MIN` | 8 | Minimum point count |
| `SR_POINTS_MAX` | 220 | Maximum point count |
| `SR_BASE_AREA_MIN` | 0.18 | Minimum base area (width × depth) |
| `SR_WIDTH_DEPTH_RATIO_MIN` | 1.05 | Minimum width-depth ratio |

**Design Rationale**:
- Point count: Based on statistical median of 7, raised to 8 for enhanced stability
- Base area: Excludes overly small noise clusters (e.g., single pole)
- Width-depth ratio: Scooters typically have lateral dimension ≥ longitudinal dimension

### 2.3 Velocity Parameters (unit: m/s)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SR_SPEED_MIN` | 1.70 | Main velocity threshold |
| `SR_SPEED_LOW` | 1.30 | Low-speed backup threshold |
| `SR_SPEED_MAX` | 6.50 | Velocity upper limit (anomaly) |

**Dual-Channel Design**:
```python
Main channel: speed ≥ 1.70 m/s
Backup channel: speed ≥ 1.30 m/s AND geom_hits ≥ 3 AND height ≥ 1.8m
```

---

## 3. Algorithm Workflow Details

### 3.1 Geometric Feature Extraction

```python
def _cluster_geometry(cluster):
    """
    Input: cluster = {
        "bbox": (xmin, xmax, ymin, ymax, zmin, zmax),
        "centroid_y": float,  # optional
        "pts": np.array (N, 3)
    }
    Output: geom dict containing:
      - w, h, d: width, height, depth
      - cy: centroid height
      - ymin, ymax: boundaries
      - avg_horizontal: (w+d)/2
      - max_horizontal: max(w, d)
      - min_horizontal: min(w, d)
      - aspect_main: h / max_horizontal
      - aspect_minor: h / min_horizontal
    """
    bbox = cluster["bbox"]
    xmin, xmax, ymin, ymax, zmin, zmax = bbox
    
    w = xmax - xmin
    h = ymax - ymin
    d = zmax - zmin
    
    # Centroid height: prioritize pre-computed value
    cy = cluster.get("centroid_y")
    if cy is None:
        pts = cluster.get("pts")
        cy = float(np.median(pts[:, 1])) if pts else 0.5*(ymin+ymax)
    
    return {
        "w": w, "h": h, "d": d,
        "cy": cy,
        "ymin": ymin, "ymax": ymax,
        "avg_horizontal": 0.5*(w+d),
        "max_horizontal": max(w, d),
        "min_horizontal": min(w, d),
        "aspect_main": h / max(max(w,d), 1e-6),
        "aspect_minor": h / max(min(w,d), 1e-6)
    }
```

### 3.2 Geometric Judgment Logic

```python
def is_scooter_rider(cluster, npts, strict=True):
    """
    Two-level judgment:
    - strict=True: for real-time cluster checking (strict)
    - strict=False: for trajectory conversion judgment (relaxed)
    
    Returns: bool (whether scooter geometry conditions are met)
    """
    geom = _cluster_geometry(cluster)
    
    # 1. Point count check
    pt_min = SR_POINTS_MIN if strict else max(5, int(SR_POINTS_MIN*0.6))
    pt_max = SR_POINTS_MAX if strict else int(SR_POINTS_MAX*1.4)
    if not (pt_min <= npts <= pt_max):
        return False
    
    # 2. Dimension check (relaxed mode expands tolerance)
    tol = 1.0 if strict else 1.25
    if not (SR_WIDTH_MIN/tol <= geom["w"] <= SR_WIDTH_MAX*tol):
        return False
    if not (SR_DEPTH_MIN/tol <= geom["d"] <= SR_DEPTH_MAX*tol):
        return False
    if not (SR_HEIGHT_MIN/1.1 <= geom["h"] <= SR_HEIGHT_MAX*tol):
        return False
    
    # 3. Centroid and top constraints
    cy_min = SR_CENTROID_Y_MIN / (1.05 if strict else 1.4)
    cy_max = SR_CENTROID_Y_MAX * (1.10 if strict else 1.35)
    if not (cy_min <= geom["cy"] <= cy_max):
        return False
    
    top_min = SR_TOP_Y_MIN / (1.05 if strict else 1.25)
    if geom["ymax"] < top_min:
        return False
    
    # 4. Horizontal dimension constraints
    avg_min = SR_AVG_HORIZ_MIN / (1.15 if strict else 1.6)
    avg_max = SR_AVG_HORIZ_MAX * (1.15 if strict else 1.45)
    if not (avg_min <= geom["avg_horizontal"] <= avg_max):
        return False
    
    # 5. Area and width-depth ratio
    area = geom["w"] * geom["d"]
    area_min = SR_BASE_AREA_MIN / (1.1 if strict else 1.5)
    if area < area_min:
        return False
    
    ratio = geom["w"] / max(geom["d"], 1e-6)
    ratio_min = SR_WIDTH_DEPTH_RATIO_MIN / (1.0 if strict else 1.2)
    if ratio < ratio_min:
        return False
    
    return True
```

### 3.3 Trajectory Consistency Accumulation

```python
class SimpleTrack:
    def __init__(self, ...):
        self.geom_hits = 0  # Geometric consistency counter
        self.converted = False  # Prevent duplicate conversion
    
    def update_logic(self, frame):
        """Per-frame update logic"""
        # 1. Preset all tracks as not updated
        for tr in tracks:
            tr.updated = False
        
        # 2. Association update
        for tr in tracks:
            if matched:
                tr.updated = True
                geom_ok = is_scooter_rider(cluster, npts, strict=True)
                if geom_ok:
                    tr.geom_hits = min(tr.geom_hits + 1, 6)  # Accumulate, cap at 6
                else:
                    tr.geom_hits = max(0, tr.geom_hits - 1)  # Decay
        
        # 3. Decay non-updated tracks
        for tr in tracks:
            if not tr.updated:
                tr.geom_hits = max(0, tr.geom_hits - 1)
```

### 3.4 Conversion Judgment

```python
def check_conversion(track):
    """Determine if should convert to ScooterRider"""
    if track.converted:
        return False  # Already converted, skip
    
    if len(track.centroids) < 3:
        return False  # Track too short
    
    # Loose geometric check
    temp_cluster = {
        "bbox": track.last_bbox,
        "centroid_y": track.last_centroid_y
    }
    geom_ok = track.geom_hits >= 1 and \
              is_scooter_rider(temp_cluster, track.last_npts, strict=False)
    
    # Velocity estimation
    speed = track.speed_robust()
    
    # Dual-channel judgment
    speed_ok = speed >= SR_SPEED_MIN
    slow_speed_ok = (speed >= SR_SPEED_LOW and 
                     track.geom_hits >= 3 and 
                     track.last_dims[1] >= 1.8)
    
    if geom_ok and (speed_ok or slow_speed_ok):
        track.converted = True
        return True
    
    return False
```

---

## 4. Parameter Tuning Guide

### 4.1 Tuning Process

```
1. Data Collection
   ├─ Scooter scenarios: At least 3 different speeds/angles
   └─ Pedestrian scenarios: At least 3 different postures/speeds

2. Statistical Analysis
   python -c "
   from test_pedestrian_fix import grid_cluster
   # Calculate centroid height, bbox dimensions, point count distribution
   "

3. Set Initial Values
   - Centroid height: Take scooter 10%-90% percentile range, slightly expand
   - Width/height/depth: Same as above
   - Velocity: Take median-0.3 ~ 90th percentile+0.5

4. Regression Testing
   python test_pedestrian_fix.py

5. Iterative Optimization
   - False positives → Tighten corresponding constraints
   - False negatives → Relax corresponding constraints
   - Prioritize adjusting centroid height and velocity thresholds
```

### 4.2 Common Issue Diagnosis

| Symptom | Possible Cause | Adjustment Suggestion |
|---------|----------------|----------------------|
| Pedestrians recognized as scooters | Centroid height too low | Increase `SR_CENTROID_Y_MIN` |
| Slow scooters missed | Velocity threshold too high | Lower `SR_SPEED_LOW` |
| Distant targets missed | Point count threshold too high | Lower `SR_POINTS_MIN` |
| Noise clusters trigger | Area/width-depth ratio too loose | Increase `SR_BASE_AREA_MIN` |
| Excessive conversions | geom_hits threshold too low | Raise geom_hits requirement in conversion conditions |

### 4.3 Parameter Sensitivity Analysis

**High Sensitivity Parameters** (±10% change has major impact):
- `SR_CENTROID_Y_MIN` / `SR_CENTROID_Y_MAX`
- `SR_SPEED_MIN`
- `SR_HEIGHT_MIN` / `SR_HEIGHT_MAX`

**Medium Sensitivity Parameters**:
- `SR_WIDTH_MIN` / `SR_WIDTH_MAX`
- `SR_BASE_AREA_MIN`

**Low Sensitivity Parameters** (auxiliary constraints):
- `SR_WIDTH_DEPTH_RATIO_MIN`
- `SR_POINTS_MIN` / `SR_POINTS_MAX`

---

## 5. Implementation Recommendations

### 5.1 Code Integration

#### Step 1: Parameter Synchronization
```python
# Replace original parameters in csv_boxs_withthings_V3.py
SR_WIDTH_MIN = 0.35
SR_WIDTH_MAX = 1.60
SR_DEPTH_MIN = 0.15
SR_DEPTH_MAX = 1.10
SR_HEIGHT_MIN = 0.10
SR_HEIGHT_MAX = 0.80
SR_CENTROID_Y_MIN = 1.30
SR_CENTROID_Y_MAX = 3.30
SR_TOP_Y_MIN = 1.80
SR_WIDTH_DEPTH_RATIO_MIN = 1.05
SR_AVG_HORIZ_MIN = 0.28
SR_AVG_HORIZ_MAX = 1.20
SR_BASE_AREA_MIN = 0.18
SR_SPEED_MIN = 1.70
SR_SPEED_LOW = 1.30
SR_SPEED_MAX = 6.50
```

#### Step 2: Function Migration
```python
# Copy _cluster_geometry, is_scooter_rider to main program
# Modify Track class, add geom_hits, converted, updated states
```

#### Step 3: Conversion Logic Update
```python
# In trajectory update loop:
for tr in active_tracks:
    # ... association logic ...
    if matched:
        tr.updated = True
        geom_ok = is_scooter_rider(...)
        if geom_ok:
            tr.geom_hits = min(tr.geom_hits + 1, 6)
        else:
            tr.geom_hits = max(0, tr.geom_hits - 1)

# Check conversion at frame end
for tr in active_tracks:
    if not tr.converted and len(tr.centroids) >= 3:
        if check_conversion(tr):
            convert_to_scooter_rider(tr)
```

### 5.2 Testing Strategy

#### Unit Tests
```python
def test_geometry_extraction():
    cluster = {"bbox": (0,1,2,3,0,0.5), "pts": ...}
    geom = _cluster_geometry(cluster)
    assert 0.9 < geom["w"] < 1.1
    assert geom["cy"] > 0

def test_scooter_classification():
    # Use known scooter cluster
    assert is_scooter_rider(scooter_cluster, 15, strict=True) == True
    # Use known pedestrian cluster
    assert is_scooter_rider(pedestrian_cluster, 10, strict=True) == False
```

#### Integration Tests
```python
# Run test_pedestrian_fix.py
# Check output statistics:
# - Pedestrian false positive rate = 0
# - Scooter recognition count > 0
```

#### Visual Verification
```python
python csv_boxs_withthings_V3.py
# Observe:
# 1. Are scooters correctly labeled orange/red
# 2. Do pedestrians remain green
# 3. Do labels display "ESCOOTER-NORMAL"
```

### 5.3 Deployment Checklist

- [ ] Parameters synchronized to main program
- [ ] Regression tests pass (0 false positives, >80% recall)
- [ ] Visual verification passes (at least 2 scenarios)
- [ ] Code committed to version control
- [ ] Documentation updated (README, parameter guide)
- [ ] Performance tests pass (real-time <100ms/frame)

---

## 6. Limitations and Improvement Directions

### 6.1 Known Limitations
1. **Environmental Dependency**: Radar height changes require re-calibration of centroid height parameters
2. **Occlusion Fragility**: Partial occlusion causing point count/dimension changes may lead to missed detections
3. **Extreme Velocities**: Very slow scooters <1.3m/s need manual annotation
4. **Multi-target Confusion**: Dense scenarios may cause trajectory ID switching

### 6.2 Improvement Directions

#### Short-term (Rule Optimization)
- Introduce temporal smoothing (Kalman filter) to stabilize velocity estimation
- Add occlusion detection and compensation mechanism
- Adaptive threshold adjustment (based on scene point cloud density)

#### Mid-term (Feature Enhancement)
- Extract Doppler velocity features (if available)
- Calculate target RCS (radar cross-section)
- Fuse trajectory historical statistical features

#### Long-term (Machine Learning)
- Collect labeled dataset (>=1000 samples)
- Train SVM/Random Forest classifier
- Explore lightweight CNN (e.g., MobileNet)

---

## 7. FAQ

### Q1: Why is centroid height so high (>1.3m)?
**A**: This is relative height in radar coordinate system, not object's ground clearance. With radar installation height as zero reference, scooter reflection points mainly come from handlebars and rider's upper body, thus higher centroid height.

### Q2: How to handle tilted radar installation?
**A**: Coordinate transformation needed first to correct tilted coordinate system to horizontal before applying parameters. Or re-calibrate centroid height range based on tilt angle.

### Q3: Are parameters universal for other radars?
**A**: Geometric parameters (width/height/depth) have strong universality, but centroid height is strongly correlated with radar installation height/pitch angle and requires re-calibration. Velocity parameters are universal.

### Q4: How to handle new scooter types (e.g., seated)?
**A**: Need to recollect data and expand bbox height range; may need to add "seated-style" subclass judgment.

### Q5: Can stationary scooters be recognized?
**A**: Current approach relies on velocity features and cannot recognize stationary scooters. Consider adding "static scooter" branch (pure geometric judgment, no velocity constraint).

---

## 8. References

### Code Repository
```
test_pedestrian_fix.py - Regression test tool
csv_boxs_withthings_V3.py - Main visualization program
工作报告_2025-10-07.md - Detailed technical report
```

### Data Format
```csv
frameNumber,detIdx,x,y,z,v,snr
1664,0,1.234,2.345,0.567,0.12,15.6
```

### Support Contact
- Technical questions: Refer to code comments
- Parameter tuning: Follow Section 4 process in this document
- Bug reports: Attach test_pedestrian_fix.py output logs

---

**Document Author**: GitHub Copilot  
**Last Updated**: 2025-10-07  
**Version History**:
- v1.0 (2025-10-07): Initial version based on test validation results
