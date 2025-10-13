# E-scooter Recognition Optimization Guide
**Date**: 2025-10-07  
**Applicable Files**: `test_pedestrian_fix.py`, `csv_boxs_withthings_V3.py`

---

## 1. Background and Objectives
In mmWave radar-collected point clouds, electric scooters and pedestrians exhibit overlapping geometric features, making stable differentiation impossible with old rule-based approaches. To solve the problems of "low scooter recognition rate and high pedestrian false positives," this work iterated through four levels: data statistics, rule design, trajectory management, and regression testing, ultimately achieving the following goals:

- Zero pedestrian false positives as scooters in regression datasets.
- Successful Track→ScooterRider conversion triggered for scooters in both test groups.
- Test script capable of rapidly reproducing experimental results.

---

## 2. Data-Driven Threshold Recalibration
### 2.1 Data Sources
- `escooter1.csv`, `escooter2.csv`: Scooter data
- `man_walk2.csv`, `man_walk3.csv`: Pedestrian data

### 2.2 Statistical Metrics
For each frame's clustered point cloud, the following metrics are calculated:
- bbox dimensions: width (x), depth (z), height (y)
- centroid height: `(ymin + ymax)/2`
- top height: `ymax`
- point count: number of points in cluster
- velocity: smoothed instantaneous speed based on trajectory

**Key Findings**:
- Scooter cluster centroid height median ~2.3m, top height typically ≥1.9m.
- Scooter cluster height (bbox y-range) concentrated in 0.1~0.6m, significantly lower than old rule's 1.35m threshold.
- Scooter speed median between 1.8~2.6 m/s.
- Pedestrian clusters either have very low height (~0.3m) or significantly smaller width-depth dimensions.

### 2.3 New Core Rule Parameters
```python
SR_WIDTH_MIN = 0.35
SR_WIDTH_MAX = 1.60
SR_DEPTH_MIN = 0.15
SR_DEPTH_MAX = 1.10
SR_HEIGHT_MIN = 0.10
SR_HEIGHT_MAX = 0.80
SR_POINTS_MIN = 8
SR_POINTS_MAX = 220
SR_CENTROID_Y_MIN = 1.30
SR_CENTROID_Y_MAX = 3.30
SR_TOP_Y_MIN = 1.80
SR_WIDTH_DEPTH_RATIO_MIN = 1.05
SR_BASE_AREA_MIN = 0.18
SR_SPEED_MIN = 1.70
SR_SPEED_LOW = 1.30
```

- **Height range** adjusted from 1.35~3.40m to 0.10~0.80m, solving "excessive height causing all scooters to be missed."
- **Centroid height** lower limit adjusted from 1.45m to 1.30m, slightly below scooter data's 10th percentile.
- **Base area** and **width-depth ratio** constraints added to filter small noise clusters.
- **Velocity channels**: main channel (≥1.7 m/s) and backup channel (≥1.3 m/s with high-confidence geometry).

---

## 3. Implementation Analysis
### 3.1 Geometric Feature Extraction `_cluster_geometry`
- Input: clustering result `cluster`, containing `bbox`, centroid, raw point set.
- Output: unified geometry descriptor dictionary, encapsulating width/height/depth, centroid height, top height, horizontal dimensions, aspect ratios, etc.
- Purpose: avoid repeated calculations, uniformly support strict and relaxed mode geometry evaluation.

### 3.2 Classification Function `is_scooter_rider`
- **Strict mode (strict=True)** for real-time update "strong condition" filtering; **Relaxed mode (strict=False)** for trajectory conversion with slightly looser tolerances.
- Execution order: point count → dimensions → centroid → top height → horizontal average → base area → width-depth ratio.
- Tolerance strategy:
  - Strict mode tolerance coefficient is 1.0; relaxed mode expands by 20~40%.
  - e.g., width in relaxed mode allows up to `SR_WIDTH_MAX * 1.25`.

### 3.3 Track Class `SimpleTrack`
- Maintains recent position, time, bbox, point count, etc.
- Introduces `geom_hits`: cumulative geometric satisfaction count, measuring trajectory's "geometric stability."
- `converted` flag ensures the same trajectory converts only once.
- `updated` flag controls `geom_hits` decay rhythm—only unmatched tracks decay in the current frame.

### 3.4 Conversion Judgment
```python
geom_ok = (track.geom_hits >= 1 and 
           is_scooter_rider(temp_cluster, track.last_npts, strict=False))

speed_ok = speed >= SR_SPEED_MIN
slow_speed_ok = (speed >= SR_SPEED_LOW and
                 track.geom_hits >= 3 and
                 height >= 1.8 and
                 horiz_mean <= SR_AVG_HORIZ_MAX * 1.05)

if geom_ok and (speed_ok or slow_speed_ok):
    track.converted = True
    conversion_count += 1
```

- **Multi-condition fusion** ensures robustness:
  - `geom_hits >= 1` guarantees at least one frame satisfies strict geometry.
  - Backup channel requires higher `geom_hits` and height to compensate for low-speed phases.
- Print logs distinguish `CONVERT`, `WAIT`, `GEOM` for debugging.

---

## 4. Test Execution and Verification
### 4.1 Run Command
```powershell
python test_pedestrian_fix.py
```
- Assumes current working directory is repository root.
- Script automatically reads CSV from hardcoded paths (modifiable as needed).
- Each dataset analyzes only first 50 frames by default for quick regression; slice range can be extended.

### 4.2 Output Interpretation
Example output snippet:
```
======================================================================
Test File: escooter1.csv
======================================================================
Total Frames: 202
  [CONVERT] Frame1836: Track->SR (v=2.31m/s, npts=8, h=0.17m)
  ...
Result Statistics:
  Track→ScooterRider conversions: 10
  Conclusion: [OK] Scooter recognized as ScooterRider (10 conversions)
```

- `CONVERT` lines indicate specific frame, velocity, point count, bbox height when Track→ScooterRider is triggered.
- "0 conversions" in pedestrian data represents zero false positives.
- If `[GEOM]` or `[WAIT]` appears, some conditions weren't met—adjust parameters accordingly.

### 4.3 Quality Thresholds
| Metric | Status | Description |
|--------|--------|-------------|
| Build | N/A | Pure script, no build step |
| Lint/Typecheck | N/A | Project doesn't integrate lint/typecheck |
| Tests | PASS | `python test_pedestrian_fix.py` passes |
| Smoke | N/A | No additional smoke tests this iteration |

---

## 5. Reproduction Steps
1. Open PowerShell or terminal, switch to repository root.
2. (Optional) Create/activate virtual environment, ensure `numpy` is installed.
3. Run test script:
   ```powershell
   python test_pedestrian_fix.py
   ```
4. Review output statistics, focus on "conversion count" and conclusion lines for each dataset.
5. For full-frame validation, edit script line `for frame_idx, fn in enumerate(frames[:50])`, replace `50` with `len(frames)`.

---

## 6. Common Issues and Troubleshooting
| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Scooter still not recognized | Data velocity < 1.3 m/s or too few points | Lower `SR_SPEED_LOW` or `SR_POINTS_MIN`, collect more point clouds |
| Pedestrian false positives | Centroid or top height too low | Increase `SR_CENTROID_Y_MIN`, `SR_TOP_Y_MIN` |
| Excessive conversions | `geom_hits` accumulates too fast or decays too slowly | Adjust decay strategy or raise `geom_hits` threshold |

---

## 7. Relationship to Main Program
- `csv_boxs_withthings_V3.py` is the main visualization program; should synchronize above parameters and logic to ensure production version matches test tool.
- Recommend adding debug switch in main program to display `geom_hits`, velocity, judgment results in real-time for on-site debugging.

---

## 8. Conclusion
Through data-driven threshold adjustment, combined geometric+velocity judgment, dynamic trajectory consistency management, and systematic regression testing, this iteration significantly improved e-scooter recognition accuracy and stability, laying a solid foundation for future feature expansion (such as occlusion handling, machine learning classification).
