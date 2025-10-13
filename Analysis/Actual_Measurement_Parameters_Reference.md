# Scooter+Rider Actual Measurement Parameters Reference

## 📐 Actual Measurement Dimensions

### Scooter Components
| Component | Parameter | Actual Size | Notes |
|-----------|-----------|------------|-------|
| Deck length | L_LEN | 1.17 m | Front-back direction |
| Deck width | L_W | 0.20 m | Left-right direction |
| Deck height | DECK_Z | 0.20 m | Ground clearance |
| Pole height | L_H | 1.25 m | From deck to handlebar |
| Pole Y position | PILLAR_Y | 0.44 m | Pole position on Y-axis |

### Human Body (Cylinder Model)
| Parameter | Actual Size | Notes |
|-----------|------------|-------|
| Height | HUM_H = 1.75 m | Total body height |
| Radius | HUM_R = 0.18 m | Cylinder radius |
| Diameter | 2R = 0.36 m | Body width |

### Combined Dimensions
| Parameter | Actual Size | Notes |
|-----------|------------|-------|
| Footprint length | ~1.17 m | Scooter length |
| Footprint width | ~0.36 m | Body diameter |
| Total height | 2.0 m | Deck 0.2m + Person 1.75m |
| Centroid height | ~1.0-1.2 m | Estimated |

## 🎯 Radar Height Compensation

**Radar Installation Height**: 0.45 m (45cm chair)

### Height Conversion
```
Point Cloud Y Coordinate = Actual Ground Height - 0.45m
```

| Actual Ground | Point Cloud Y Coordinate | Meaning |
|--------------|------------------------|---------|
| 0.0 m | -0.45 m | Ground |
| 0.20 m | -0.25 m | Scooter deck |
| 0.45 m | 0.0 m | Radar horizontal plane |
| 1.0 m | 0.55 m | Lower centroid limit |
| 1.2 m | 0.75 m | Upper centroid limit |
| 2.0 m | 1.55 m | Combined total height |

## 📊 Point Cloud Dispersion Analysis

### When Static
- Actual size: 1.17m × 0.36m × 2.0m
- Point cloud range: Close to actual size

### When Moving (Considering 30-50% dispersion)
- Point cloud width: 0.36 × (0.7~1.5) = 0.25~0.54 m
- Point cloud length: 1.17 × (0.7~1.5) = 0.82~1.76 m
- Point cloud height: 2.0 × (0.8~1.5) = 1.6~3.0 m (but limited by radar height)

## ⚙️ Recognition Parameter Settings

### Pre-Adjustment Parameters (Based on unmove_scooter.csv analysis)
```python
SR_WIDTH_MIN = 4.0       # Too large!
SR_WIDTH_MAX = 7.4       # Too large!
SR_HEIGHT_MIN = 2.5      # Too large!
SR_HEIGHT_MAX = 4.3      # Too large!
SR_CENTROID_Y_MIN = 0.55 # Reasonable
SR_CENTROID_Y_MAX = 1.45 # Slightly large
```
**Problem**: unmove_scooter.csv may contain multiple targets or background noise, causing oversized dimensions

### Post-Adjustment Parameters (Based on actual measurements)
```python
# Horizontal dimensions (considering 30-50% point cloud dispersion)
SR_WIDTH_MIN = 0.25      # 0.36 × 0.7 = 0.25m
SR_WIDTH_MAX = 1.80      # 1.17 × 1.5 = 1.75m (rounded to 1.8m)
SR_DEPTH_MIN = 0.25      
SR_DEPTH_MAX = 1.80      

# Vertical dimensions (relative to radar)
SR_HEIGHT_MIN = 1.20     # (2.0-0.45) × 0.8 = 1.24m
SR_HEIGHT_MAX = 2.40     # (2.0-0.45) × 1.5 = 2.33m

# Centroid height (relative to radar)
SR_CENTROID_Y_MIN = 0.40 # (1.0-0.45) × 0.7 ≈ 0.40m
SR_CENTROID_Y_MAX = 1.10 # (1.2-0.45) × 1.4 ≈ 1.10m

# Point count
SR_POINTS_MIN = 15       # May be sparse when moving
SR_POINTS_MAX = 250      # May be dense when moving
```

### Shape Features
```python
# Height-to-width ratio: height / average horizontal dimension > 0.8
avg_horizontal = (w + d) / 2.0
aspect_ratio = h / avg_horizontal

# Actual measured aspect ratio:
# Height: 1.55m (relative to radar)
# Average horizontal: (1.17 + 0.36) / 2 = 0.77m
# Aspect ratio: 1.55 / 0.77 ≈ 2.0
```

## 🔍 Distinction from Other Targets

### Walking Person
| Parameter | Walking Person | Scooter+Rider | Difference |
|-----------|---------------|---------------|-----------|
| Footprint area | 0.4×0.4 = 0.16 m² | 1.17×0.36 = 0.42 m² | Scooter larger |
| Height | 1.5-1.8 m | 2.0 m | Scooter taller |
| Centroid Y (relative to radar) | 0.4-0.7 m | 0.4-1.1 m | Scooter wider range |
| Speed | 0.5-2.5 m/s | 2.78-6.94 m/s | Scooter faster |
| Shape | Nearly cylindrical | L-shaped combination | Different shapes |

### Small Objects
| Parameter | Small Objects | Scooter+Rider |
|-----------|--------------|---------------|
| Height | < 1.0 m | 1.2-2.4 m |
| Centroid Y | < 0.2 m | 0.4-1.1 m |
| Speed | ~0 m/s | 2.78-6.94 m/s |

### Car/Bicycle
| Parameter | Car | Bicycle | Scooter+Rider |
|-----------|-----|---------|---------------|
| Width | 1.5-2.0 m | 0.6-0.8 m | 0.25-1.8 m |
| Height | 1.2-1.6 m | 1.0-1.5 m | 1.2-2.4 m |
| Speed | > 8 m/s | 3-8 m/s | 2.78-6.94 m/s |
| Centroid Y | 0.4-0.8 m | 0.3-0.7 m | 0.4-1.1 m |

## 📋 Verification Checklist

### Size Verification
- [ ] Point cloud width in 0.25-1.8m range
- [ ] Point cloud depth in 0.25-1.8m range
- [ ] Point cloud height in 1.2-2.4m range
- [ ] Centroid Y in 0.4-1.1m range

### Speed Verification
- [ ] Speed in 10-25 km/h (2.78-6.94 m/s)
- [ ] Display red warning when speed ≥20 km/h

### Shape Verification
- [ ] Height/average horizontal dimension > 0.8
- [ ] Point cloud shape relatively elongated

### Continuity Verification
- [ ] Duration ≥ 0.3 seconds
- [ ] Trajectory smooth and continuous

## 🎯 Key Recognition Features Ranking

1. **Speed Range** (Most critical): 2.78-6.94 m/s
   - Exclude static objects and pedestrians
   - Exclude fast vehicles

2. **Centroid Height** (Secondary critical): 0.4-1.1 m (relative to radar)
   - Exclude ground objects (lower centroid)
   - Exclude bicycles alone (slightly lower centroid)

3. **Vertical Dimension**: 1.2-2.4 m
   - Characteristic height of scooter+rider
   - Significantly taller than pedestrians or vehicles

4. **Horizontal Dimension**: 0.25-1.8 m
   - Exclude oversized vehicles
   - Exclude undersized individuals

5. **Shape Feature**: Height/width ratio > 0.8
   - Elongated feature of L-shaped structure
   - Distinguish from wide, low vehicles

## 📝 Testing Recommendations

### Test Data Selection
- ✅ Use actual e-scooter+rider motion data
- ✅ Include different speeds: 10km/h, 15km/h, 20km/h, 25km/h
- ✅ Include different angles: frontal, side, diagonal

### Observation Points
1. Can it stably recognize and track
2. Purple border displayed normally (<20km/h)
3. Red border triggered correctly (≥20km/h)
4. No misdetection of pedestrians or other targets
5. Trajectory smooth and continuous

### Parameter Fine-Tuning
If detection is inaccurate, adjust in the following order:

1. **Speed Range** (Highest priority)
   - Missed detection: Expand SR_SPEED_MIN/MAX
   - False detection: Tighten speed range

2. **Centroid Height** (Secondary priority)
   - Missed detection: Expand SR_CENTROID_Y_MIN/MAX
   - False detection: Tighten range

3. **Size Range** (Third priority)
   - Large point cloud dispersion: Increase SR_WIDTH/HEIGHT_MAX
   - Small point cloud dispersion: Decrease maximum values

4. **Point Count Range** (Last adjustment)
   - Sparse point cloud: Decrease SR_POINTS_MIN
   - Dense point cloud: Increase SR_POINTS_MAX

---
**Update Date**: 2025-10-07  
**Based On**: Actual measured scooter+rider dimensions  
**Radar Height**: 0.45m (45cm chair)
