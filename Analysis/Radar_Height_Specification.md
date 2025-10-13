# Radar Installation Height Specification

## 📏 Installation Details

**Radar Installation Height**: 45cm (0.45m) - Mounted on chair

## 🎯 Coordinate System

### Meaning of Y-axis (Height) Coordinates
- **Point Cloud Y Coordinate**: Height relative to radar position
- **Actual Ground Height**: Y coordinate + 0.45m

### Examples
| Point Cloud Y Value | Actual Ground Height | Corresponding Object |
|-------------------|---------------------|---------------------|
| 0.0m | 0.45m | Radar horizontal plane, chair seat |
| -0.45m | 0.0m | Ground |
| 0.55m | 1.0m | Scooter deck/waist |
| 1.55m | 2.0m | Person's shoulder/head |

## ⚙️ Parameter Adjustments

### 1. ScooterRider Centroid Height (Adjusted)

**Original Analysis Results** (based on unmove_scooter.csv):
- Centroid height: 1.0-1.9m (ground height)

**Adjusted Parameters** (considering radar height):
```python
SR_CENTROID_Y_MIN = 0.55  # 1.0 - 0.45 = 0.55m (relative to radar)
SR_CENTROID_Y_MAX = 1.45  # 1.9 - 0.45 = 1.45m (relative to radar)
```

### 2. Person's Y_EXTENT_MIN (Line 108)
```python
Y_EXTENT_MIN = 0.35  # Person's minimum vertical range
```
- This value is the height span of point cloud, not affected by radar installation height
- Keep original value

### 3. Object Height Limit (Line 120)
```python
OBJ_MAX_Y_EXTENT = 1.00  # Small object height limit (point cloud range)
```
- This value is the object's vertical span in point cloud
- Not absolute height, keep original value

## 📊 Height Range Reference Table

### Scooter+Rider Combination

| Body Part | Actual Ground Height | Point Cloud Y Coordinate | Notes |
|-----------|-------------------|------------------------|-------|
| Deck | 0.1-0.3m | -0.35 ~ -0.15m | Scooter bottom |
| Body | 0.5-1.0m | 0.05 ~ 0.55m | Handlebar/pole |
| Rider legs | 0.8-1.2m | 0.35 ~ 0.75m | Lower body |
| Rider waist | 1.0-1.5m | 0.55 ~ 1.05m | Mid-section |
| Rider torso | 1.5-2.0m | 1.05 ~ 1.55m | Upper body |
| **Centroid** | **1.0-1.9m** | **0.55-1.45m** | **Key recognition feature** |

### Walking Person

| Body Part | Actual Ground Height | Point Cloud Y Coordinate | Notes |
|-----------|-------------------|------------------------|-------|
| Feet | 0.0-0.2m | -0.45 ~ -0.25m | Near ground |
| Legs | 0.5-1.0m | 0.05 ~ 0.55m | Lower body |
| Torso | 1.0-1.5m | 0.55 ~ 1.05m | Mid-section |
| Head | 1.5-1.8m | 1.05 ~ 1.35m | Upper body |
| **Centroid** | **0.8-1.2m** | **0.35-0.75m** | **Typical person centroid** |

### Small Objects (Chairs, Boxes, etc.)

| Type | Actual Ground Height | Point Cloud Y Coordinate | Notes |
|------|-------------------|------------------------|-------|
| Ground objects | 0.0-0.5m | -0.45 ~ 0.05m | Boxes, packages |
| Seats | 0.4-0.8m | -0.05 ~ 0.35m | Chairs, stools |
| **Centroid** | **0.2-0.5m** | **-0.25 ~ 0.05m** | **Lower than people and scooters** |

## 🔍 Centroid Height Distinction Role

Due to radar installation at 45cm height, centroid heights (point cloud Y coordinates) of different target types differ significantly:

```
Ground objects:  -0.25 ~ 0.05m   ← Lowest
   ↓
Walking person:   0.35 ~ 0.75m   ← Medium
   ↓
Scooter+rider:  0.55 ~ 1.45m   ← Higher (key feature!)
```

**This height difference is the key to distinguishing different targets!**

## ⚠️ Important Notes

1. **Negative Y coordinates are normal**
   - Y < 0 means target is below radar
   - Example: Y = -0.3m means actual ground height 0.15m (near ground)

2. **unmove_scooter.csv Data**
   - If this file was also collected at 45cm height, parameters are correctly adjusted
   - If not, need to re-analyze and adjust parameters

3. **Other Height Parameters Unaffected**
   - `Y_EXTENT_MIN`: Vertical span, not absolute height
   - `OBJ_MAX_Y_EXTENT`: Object height range, not ground height
   - Only **centroid Y coordinate** needs to consider radar height compensation

## 📝 Verification Method

Add debug output when running script:
```python
# Add in is_scooter_rider()
cy = float(pts[:, 1].mean())
print(f"Cluster centroid Y: {cy:.3f}m, actual ground: {cy+0.45:.3f}m")
```

Normal cases:
- Scooter+rider: cy ≈ 0.55~1.45m (actual 1.0~1.9m)
- Walking person: cy ≈ 0.35~0.75m (actual 0.8~1.2m)
- Ground objects: cy ≈ -0.25~0.05m (actual 0.2~0.5m)

---
**Update Date**: 2025-10-07  
**Radar Model**: IWR6843  
**Installation Height**: 45cm (chair)
