## 1) Objective
Collect roadside mmWave radar data to verify Stable the detection & tracking of pedestrians and e-scooters
## 2) Fixed placement & geometry
Lateral offset d₀ = 3.0 m, mount height h = 1.1 m, tilt = −3° 
## 3) Visualizer unified settings (work for both classes)
Frame Rate: 20 fps
Range Resolution: 0.045–0.06 m
Max Unambiguous Range: 12 m
Max Radial Velocity: ±8 m/s 
Radial Velocity Resolution: 0.40 m/s 
Real-Time Tuning:
o	Group Peaks (Range/Doppler): ON
o	Remove Static Clutter: ON
o	CFAR: Range 17 dB, Doppler 14 dB 
o	FOV: Azimuth ±90°. Elevation −10° to +15°
o	Gates: Range 0.5–12 m; Doppler −8 to +8 m/s
## 4) Recording
File Size Max: 50 MB
Record time: scooter 10–12 s; pedestrian 18–20 s
## 5) Collection procedure
1.	Pedestrians (2 speeds, 3 trials each): slow 1.2–1.3 m/s; fast ~1.6 m/s.
2.	E-scooters (3 speeds, 3 trials each): 3.0 m/s (legal), 4.0 m/s (overspeed), 5.0–6.0 m/s (dangerous).
3.	Naming: type_speed_d3_trialXX.csv.
4.	Optional: synchronized phone video at 60 fps; clap at start for time alignment.
## 6) Outputs & organizing： Save each pass: CSV.
