# ⚙️ mmWave Demo Visualizer Configuration Parameters Detailed Guide

---

## 📊 Current Configuration Overview

### Basic Settings
| Parameter | Current Value | Description |
|-----------|--------------|-------------|
| Platform | xWR18xx_AOP | Radar chip platform |
| SDK Version | 3.5 | SDK version |
| Antenna Config | 4Rx,3Tx(30 Azim 38 Elev) | 4 receive, 3 transmit antennas |
| Desirable Configuration | Best Range | Preset configuration mode |
| Frequency Band | 77-81 GHz | Operating frequency |

### Scene Parameters (Scene Selection)
| Parameter | Current Value | Range | Description |
|-----------|--------------|-------|-------------|
| Frame Rate | 10 fps | 1-30 fps | Frame rate, affects temporal resolution |
| Range Resolution | 0.052 m | 0.047-0.195 m | Distance resolution, smaller = more precise |
| Maximum Unambiguous Range | 10 m | 5-50 m | Maximum unambiguous distance |
| Maximum Radial Velocity | 7.23 m/s | 0.42-7.23 m/s | Maximum radial velocity |
| Radial Velocity Resolution | 0.46 m/s | 0.46 m/s | Velocity resolution |

### Real-Time Tuning
| Parameter | Status | Description |
|-----------|--------|-------------|
| Remove Static Clutter | ✅ Enabled | Remove static clutter, recommended on |
| CFAR Range Threshold | ~0-100 dB | Constant false alarm rate distance threshold |
| Doppler Range Threshold | ~0-100 dB | Doppler distance threshold |
| Angle of Arrival (Azimuth) | -90° to 90° | Azimuth field of view |
| Angle of Arrival (Elevation) | -90° to 90° | Elevation field of view |
| Range | 0 to 9.99 m | Detection range |

---

## 🎯 Parameter Adjustment Guide

### 1. Frame Rate

**Current Value**: 10 fps  
**Recommended Values**: 
- **Static/slow scenes**: 5-10 fps (save computation)
- **Regular scenes**: 10-15 fps ✅ (balanced performance)
- **High-speed scenes**: 15-20 fps (improve tracking accuracy)
- **Very high-speed scenes**: 20-30 fps (reduce motion blur)

**Trade-offs**:
- ⬆️ Increase frame rate → Smoother tracking, higher computational load
- ⬇️ Decrease frame rate → Save resources, may miss fast changes

**Influencing Factors**:
```
Velocity sampling = Max velocity / Frame rate
10 fps @ 7.23 m/s → Max 0.72 m per frame
20 fps @ 7.23 m/s → Max 0.36 m per frame
```

---

### 2. Range Resolution

**Current Value**: 0.052 m (5.2 cm)  
**Available Range**: 0.047 - 0.195 m  
**Recommended Values**:
- **High-precision scenes**: 0.047 m (minimum) - for close-range fine recognition
- **Balanced scenes**: 0.050-0.070 m ✅ - general use
- **Long-range scenes**: 0.100-0.195 m - trade for longer detection range

**Physical Meaning**:
```
Range Resolution = c / (2 × Bandwidth)
c = 3×10^8 m/s (speed of light)
Bandwidth ≈ 4 GHz (77-81 GHz)
Theoretical minimum ≈ 0.0375 m
```

**Trade-offs**:
- ⬇️ Smaller resolution → Finer distance distinction, but max range limited
- ⬆️ Larger resolution → Longer detection range, but details blurred

**Application Recommendations**:
| Scene | Recommended Value | Reason |
|-------|------------------|--------|
| Scooter+rider close recognition | 0.047-0.055 m | Need to distinguish deck/pole/rider |
| Medium-range tracking | 0.050-0.070 m | Balance precision and range |
| Long-range warning | 0.100+ m | Prioritize detection distance |

---

### 3. Maximum Unambiguous Range

**Current Value**: 10 m  
**Available Range**: 5 - 50 m  
**Recommended Values**:
- **Indoor/close-range**: 5-10 m ✅ - improve resolution
- **Outdoor medium-range**: 10-20 m - balanced scenario
- **Long-range monitoring**: 30-50 m - sacrifice resolution

**Relationship with Range Resolution**:
```
Max Range ↑ → Range Resolution ↓ (trade-off)
```

**Practical Applications**:
```
10 m setting:
- Covers 0-10 m range
- Suitable for parking lot/sidewalk scenarios
- Resolution 0.052 m relatively good

20 m setting:
- Covers 0-20 m range  
- Suitable for intersection monitoring
- Resolution may drop to 0.1 m
```

**Selection Recommendations**:
- Test area ≤ 8m → Use 10m
- Test area 8-15m → Use 20m
- Test area > 15m → Use 30-50m

---

### 4. Maximum Radial Velocity

**Current Value**: 7.23 m/s (~26 km/h)  
**Available Range**: 0.42 - 7.23 m/s  
**Recommended Values**:
- **Pedestrian scenes**: 3-5 m/s (10-18 km/h)
- **Scooter scenes**: 5-7 m/s (18-25 km/h) ✅
- **Bicycle/motorcycle**: 7-10 m/s (25-36 km/h) - needs special configuration

**Velocity Coverage Range**:
| Target Type | Typical Speed | Recommended Max Velocity Setting |
|------------|--------------|----------------------------------|
| Static objects | 0 m/s | Any |
| Pedestrian walking | 1.4 m/s (5 km/h) | 3 m/s |
| Pedestrian running | 3.5 m/s (12.6 km/h) | 5 m/s |
| Scooter slow | 2.5 m/s (9 km/h) | 5 m/s |
| Scooter normal | 4.5 m/s (16 km/h) | 7 m/s ✅ |
| Scooter fast | 6.5 m/s (23 km/h) | 8-10 m/s |
| Bicycle | 8 m/s (29 km/h) | 10 m/s |

**Radial Velocity vs Actual Speed**:
```
Radial Velocity = Actual Speed × cos(angle)

Head-on approach (0°): Radial velocity = Actual speed
45° angle approach: Radial velocity = Actual speed × 0.707
Side passing (90°): Radial velocity ≈ 0
```

**Note**: 
- Current 7.23 m/s just covers regular scooter speeds
- If testing high-speed scenarios, may need to adjust to higher value
- But larger velocity range may reduce velocity resolution

---

### 5. Radial Velocity Resolution

**Current Value**: 0.46 m/s  
**Description**: Usually determined automatically by other parameters, not independently adjustable  

**Physical Relationship**:
```
Velocity Resolution = λ / (2 × Observation Time)
λ = c / f (wavelength)
f = 79 GHz (center frequency)
λ ≈ 3.8 mm
```

**Practical Meaning**:
```
0.46 m/s resolution means:
- Targets with speed difference < 0.46 m/s may not be distinguishable
- 1.4 m/s (walk) vs 2.5 m/s (run) → Distinguishable ✅
- 3.5 m/s vs 3.8 m/s → Hard to distinguish
```

**Improvement Methods**:
- Increase observation time (frame accumulation)
- Use trajectory smoothing algorithms

---

## 🔧 Recommended Configuration Plans

### Plan A: Current Configuration (General) ✅

```
Frame Rate: 10 fps
Range Resolution: 0.052 m
Max Range: 10 m
Max Velocity: 7.23 m/s
Remove Static Clutter: ON
```

**Suitable Scenarios**:
- ✅ Scooter normal speed recognition
- ✅ Pedestrian walking/running
- ✅ Near to medium range scenes (2-8m)
- ✅ Single target/few targets

**Advantages**: Balanced, suitable for 90% scenarios  
**Disadvantages**: High-speed/long-range limited

---

### Plan B: High-Speed Scene Optimization

```
Frame Rate: 15-20 fps          ⬆️ Increased
Range Resolution: 0.055 m      (Slightly reduced to trade for speed)
Max Range: 10 m                (Keep)
Max Velocity: 10 m/s           ⬆️ Increased (needs manual adjustment)
Remove Static Clutter: ON
```

**Suitable Scenarios**:
- Scooter high-speed testing (>25 km/h)
- Bicycle scenes
- Need to capture fast changes

**Disadvantages**: 
- May need to modify underlying configuration files
- Increased computational load

---

### Plan C: High-Precision Close-Range

```
Frame Rate: 15 fps             ⬆️ Increased
Range Resolution: 0.047 m      ⬇️ Minimum (finest)
Max Range: 5-8 m               ⬇️ Reduced
Max Velocity: 7.23 m/s         (Keep)
Remove Static Clutter: ON
```

**Suitable Scenarios**:
- Scooter detailed feature analysis
- Close-range fine recognition
- Need to distinguish small size differences

**Advantages**: Highest spatial resolution  
**Disadvantages**: Effective range shortened

---

### Plan D: Long-Range Monitoring

```
Frame Rate: 8-10 fps           (Keep or reduce)
Range Resolution: 0.100 m      ⬆️ Increased
Max Range: 20-30 m             ⬆️ Increased
Max Velocity: 7.23 m/s         (Keep)
Remove Static Clutter: ON
```

**Suitable Scenarios**:
- Intersection monitoring
- Long-range early warning
- Large area coverage

**Disadvantages**: Reduced detail recognition capability

---

## 🎛️ Real-Time Tuning Parameters Explanation

### Remove Static Clutter
**Current**: ✅ Enabled  
**Recommended**: Always enabled  
**Function**: 
- Remove reflections from static objects (ground, walls, stationary vehicles)
- Keep only moving targets
- Significantly reduce background noise

**Disable Scenario**: Only for debugging or detecting static objects

---

### CFAR Threshold (Constant False Alarm Rate)

**Range/Doppler Threshold**: Usually keep default  
**Function**: Control detection sensitivity  
**Adjustment Recommendations**:
- Too much noise → Increase threshold
- Missing targets → Decrease threshold
- Generally no adjustment needed

---

### Angle of Arrival

**Azimuth**: -90° to 90°  
**Elevation**: -90° to 90°  

**Current**: Full range open  
**Actual Effective Range**: About ±60° (limited by antenna configuration)  

**Optimization Recommendations**:
```
If test area is fixed:
Azimuth: -45° to 45°   (Limit left-right range)
Elevation: -30° to 30° (Limit up-down range)

Advantages: Reduce edge noise, improve center region quality
```

---

### Range (Distance Range)

**Current**: 0 to 9.99 m  
**Adjustment Recommendations**:
```
If target activity area is 3-8m:
Range: 2 to 10 m

Advantages: 
- Filter close-range clutter (< 2m)
- Filter long-range noise (> 10m)
```

---

## 📐 Parameter Relationship Diagram

```
           ┌─────────────────────────────────┐
           │   Maximum Range                  │
           │         ↑↓ Mutually exclusive    │
           │   Range Resolution               │
           └─────────────────────────────────┘
                        │
                        │ Affects
                        ↓
           ┌─────────────────────────────────┐
           │   Point Cloud Density & Detection Range │
           └─────────────────────────────────┘

           ┌─────────────────────────────────┐
           │   Frame Rate                     │
           │         ↓ Affects                │
           │   Motion Blur & Tracking Continuity │
           └─────────────────────────────────┘

           ┌─────────────────────────────────┐
           │   Maximum Velocity               │
           │         ↓ Affects                │
           │   Velocity Measurement Range     │
           └─────────────────────────────────┘
```

---

## ✅ Configuration Verification Checklist

After adjusting configuration, perform the following verifications:

### 1. Connection Verification
- [ ] "SEND CONFIG TO MMWAVE DEVICE" successful
- [ ] Scatter Plot displays point cloud
- [ ] No error messages

### 2. Performance Verification
- [ ] Frame rate stable (no stuttering)
- [ ] CPU/memory usage normal
- [ ] Data recording smooth

### 3. Data Quality Verification
- [ ] Static background filtered (Static Clutter ON)
- [ ] Moving targets clearly visible
- [ ] Point cloud density appropriate (not too sparse/dense)
- [ ] Velocity readings reasonable

### 4. Range Verification
- [ ] Place target at marked distance (e.g. 5m)
- [ ] Check radar display distance error < 10%

### 5. Velocity Verification
- [ ] Known-velocity motion
- [ ] Radial velocity ≈ Actual speed × cos(angle)

---

## 🔄 Configuration Switching Procedure

```
1. Save current configuration
   → "SAVE CONFIG TO PC" → xxx_config_A.cfg

2. Adjust parameters
   → Adjust sliders in Scene Selection

3. Send new configuration
   → "SEND CONFIG TO MMWAVE DEVICE"

4. Verify configuration
   → Observe visualization effects

5. Save new configuration
   → "SAVE CONFIG TO PC" → xxx_config_B.cfg

6. Record configuration plan
   → Mark configuration used in experiment log
```

---

## 📚 Reference Documentation

- TI mmWave SDK User Guide
- xWR18xx Technical Reference Manual
- mmWave Demo Visualizer User Guide

---

**Creation Date**: 2025-10-11  
**Applicable Version**: SDK 3.5  
**Update**: Continuously optimize based on actual test results
