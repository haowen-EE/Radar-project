# Scooter+Rider Unified Target Detection Feature Description

## Feature Overview
Added **Scooter+Rider unified target detection** functionality in `csv_boxs_withthings_V3.py`, identifying and tracking e-scooters with riders as a single entity.

## Implementation Principle

### 1. Data Analysis
Analysis of static scooter+rider point cloud data from `unmove_scooter.csv` extracted the following key features:

| Feature Type | Parameter Value | Description |
|-------------|----------------|-------------|
| **Point Count** | 24-199 points | ~45 points when static, dispersed when moving |
| **Width** | 4.0-7.4 m | Considering 30% motion margin |
| **Height** | 2.5-4.3 m | Vertical dimension of rider + scooter |
| **Depth** | 4.3-8.0 m | Longitudinal dispersion range |
| **Centroid Height** | 1.0-1.9 m | **Key distinguishing feature**, significantly higher than ground objects |
| **Speed Range** | 2.78-6.94 m/s | 10-25 km/h, normal e-scooter speed range |
| **Danger Threshold** | >5.56 m/s | >20 km/h, overspeed displays red warning |
| **Shape Feature** | Height/width ratio > 1.2 | Relatively tall and thin, distinct from vehicles or ground objects |

### 2. Core Components

#### A. `is_scooter_rider(cluster, npts)` Recognition Function
Comprehensively judges whether a cluster is a scooter+rider combination:
```python
✓ Point count between 24-199
✓ Dimensions fit 4-7.4m(width) × 2.5-4.3m(height) × 4.3-8m(depth)
✓ Centroid height 1.0-1.9m (most critical feature)
✓ Height-to-width ratio > 1.2 (tall-thin characteristic)
```

#### B. `ScooterRiderTrack` Tracking Class
Dedicated to scooter+rider state tracking:
- **Speed Range**: 2.78-6.94 m/s (10-25 km/h, normal e-scooter speed)
- **Danger Speed**: >5.56 m/s (>20 km/h, overspeed dangerous riding)
- **Confirmation Strategy**: Sustained 0.3s + speed within range → accumulate 3 points for confirmation
- **Display Color**: 
  - Normal speed (<20km/h): Purple border (RGB: 0.8, 0.2, 0.8)
  - Danger speed (≥20km/h): Red border (RGB: 1.0, 0.0, 0.0)
- **Label Format**: 
  - Normal: `SR{ID} {speed}m/s ({speed}km/h)`
  - Danger: `SR{ID} {speed}m/s ({speed}km/h) ⚠DANGER`

### 3. Detection Workflow
Process in main loop `update()` with the following priority:

```
1. Object Association      ← Confirmed small objects highest priority
2. ScooterRider Association ← Scooter+rider secondary priority
3. Regular Track Association ← Pedestrians and unconfirmed objects
4. New Track Creation:
   - Meets is_scooter_rider() → Create ScooterRiderTrack
   - Otherwise → Create regular Track
```

**Priority Design Purpose**: Prevent scooter+rider from being misidentified as multiple pedestrians or objects

### 4. Association Gate
```python
sr_gate = max(assoc_gate * 1.5, SR_SPEED_MAX * dt_med * 2.0)
```
Uses larger association gate (1.5× regular tracks) because scooters move faster.

## Usage Instructions

### Run Script
```bash
python csv_boxs_withthings_V3.py
```

### Window Display
Title bar shows count of each target type:
```
Pedestrians: X  Objects: Y  Scooters: Z  ScooterRiders: W
```

### Visual Identifiers
- **Purple border**: ScooterRider target
- **Label**: `SR{ID} {speed}m/s` (e.g., SR1 4.52m/s)
- **Red border + ⚠**: When speed ≥20 km/h (danger)

## Parameter Adjustment

### If detection is inaccurate, modify the following parameters:

#### 1. Size Range (Lines 519-528)
```python
SR_WIDTH_MIN = 4.0    # Too strict → decrease
SR_WIDTH_MAX = 7.4    # Too loose → increase
SR_HEIGHT_MIN = 2.5
SR_HEIGHT_MAX = 4.3
SR_DEPTH_MIN = 4.3
SR_DEPTH_MAX = 8.0
```

#### 2. Point Count Range (Lines 529-530)
```python
SR_POINTS_MIN = 24   # Sparse point cloud → decrease
SR_POINTS_MAX = 199  # Dense point cloud → increase
```

#### 3. Centroid Height (Lines 531-532) **Most Critical Parameter**
```python
SR_CENTROID_Y_MIN = 1.0  # Too many ground detections → increase
SR_CENTROID_Y_MAX = 1.9  # Missing detections → increase
```

#### 4. Speed Range (Lines 541-543)
```python
SR_SPEED_MIN = 2.78      # Minimum speed ~10 km/h
SR_SPEED_MAX = 6.94      # Maximum speed ~25 km/h
SR_SPEED_DANGER = 5.56   # Danger threshold ~20 km/h, overspeed displays red
```

#### 5. Confirmation Threshold (Lines 535-538)
```python
SR_MIN_DURATION_S = 0.3  # Fast confirmation → decrease
SR_CONFIRM_SCORE = 3     # Strict confirmation → increase
SR_SCORE_HIT = 2         # Score increase rate
SR_SCORE_MISS = 1        # Score decrease rate
```

## Testing Recommendations

### 1. Use Real Motion Data
Point `MANUAL_CSV_FILE` to CSV file containing moving scooters:
```python
MANUAL_CSV_FILE = 'moving_scooter_rider.csv'
```

### 2. Observe Detection Effects
- ✓ **Correct**: Purple box stably follows scooter+rider
- ✗ **Missed**: No purple box → Relax size/point count/centroid height
- ✗ **False Positive**: Pedestrians identified as scooters → Tighten speed range or raise centroid height minimum

### 3. Speed Verification
Label-displayed speed should be 2.78-6.94 m/s:
- < 2.78 m/s (10 km/h) → May be pushed or slow movement
- 2.78-5.56 m/s (10-20 km/h) → Normal speed, purple border
- 5.56-6.94 m/s (20-25 km/h) → Danger speed, red border + ⚠DANGER
- > 6.94 m/s (25 km/h) → Overspeed or misdetected other vehicle

### 4. Centroid Height Verification
Add debug output in `is_scooter_rider()`:
```python
cy = float(pts[:, 1].mean())
print(f"Cluster {ci}: centroid_y = {cy:.3f}")  # Observe actual centroid height
```

## Technical Details

### Why is Centroid Height a Key Feature?
1. **Ground Objects** (boxes, chairs): Centroid height < 0.5m
2. **Scooter Deck**: Height ~0.1-0.3m
3. **Rider Legs/Waist**: Median ~2.0m
4. **Combined Centroid**: ~1.46m ← **Unique middle height**

This feature effectively distinguishes:
- **Pedestrians**: Higher centroid (>2.0m)
- **Vehicles**: Lower centroid (0.3-0.8m)
- **Objects**: Centroid near ground (<0.5m)

### Why Use Larger Association Gate?
```python
sr_gate = max(assoc_gate * 1.5, SR_SPEED_MAX * dt_med * 2.0)
```
E-scooters can reach 8 m/s, moving ~0.4m in frame interval (~0.05s), so need more lenient association distance to avoid trajectory breaks.

## Relationship with Existing Features

| Feature Module | Priority | Speed Range | Purpose |
|---------------|----------|-------------|---------|
| Object Detection | Highest | 0-0.5 m/s | Static/slow small objects |
| **ScooterRider** | **Secondary** | **2.78-6.94 m/s (10-25 km/h)** | **E-scooter+rider, ≥20km/h red warning** |
| People Detection | Regular | 0.3-3.0 m/s | Walking pedestrians |
| Scooter Plugin | Independent | Unlimited | Separate scooter detection |

## FAQ

**Q: Why not directly use escooter_plugin?**  
A: escooter_plugin detects **scooters alone**, while this feature identifies **scooter+rider combination**, with different features and application scenarios.

**Q: How to disable this feature?**  
A: Comment out ScooterRider association code at lines 838-865, and creation logic at lines 877-879.

**Q: Can display color be adjusted?**  
A: Yes, modify color parameter at line 925:
```python
color=(0.8, 0.2, 0.8, 1.0)  # RGB + Alpha
# For danger: color=(1.0, 0.0, 0.0, 1.0)  # Red
```

---
**Version**: V3_SR1  
**Update Date**: 2024  
**Reference Data**: unmove_scooter.csv (static reference data)
