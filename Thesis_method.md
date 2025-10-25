	Methodology
4.1 System Architecture and Data Collection
The project developed a stationary, roadside mmWave sensing platform using TI AWR1843AOP hardware configured in the 77–81 GHz band. Unless otherwise stated, experiments ran at 10 fps, ∆R≈0.052m, max unambiguous range 10 m, max radial velocity ±7.23 m/s, and velocity resolution ≈ 0.46 m/s. Static clutter suppression was enabled. These settings balance centimetre level ranging with reliable short range coverage of 2–10 m.

The radar was mounted 0.45 m above ground on a chair. Because y coordinates in the radar point cloud are measured relative to the radar, the ground appears near y = −0.45 m. Typical centroid bands after height compensation are: ground objects −0.25–0.05 m, walking persons 0.35–0.75 m, e scooter+rider 0.55–1.45 m.

Data were first configured and previewed with TI’s mmWave Demo Visualizer, then recorded as paired .dat/.cfg files. The project converted streams to CSV with a custom parser that reads the .cfg to stay consistent with variable profiles. To segment frames robustly, we used detIdx==0 as a frame boundary and rebuilt a strictly increasing timeline for playback and offline processing. This corrected the common issue where multiple subframes share the same timestamp. And the project followed an internal data collection plan covering e scooters at different speeds and approach angles, pedestrians walking/jogging, two person parallel motion, stationaries, and bicycle passes, all at 10 fps with the above radar settings.


4.2 Signal Processing and Detection
Standard FMCW processing (range FFT and Doppler FFT) and CFAR detection run on the device. We compared sliding window and simple statistical background models, keeping Remove Static Clutter (RSC) on during field tests. The visualizer outputs point detections with (x, y, z) and ancillary fields (e.g., Doppler/RCS when available).


4.3 Clustering and Feature Extraction

Earlier project used DBSCAN; however, for real time robustness and predictable compute. The project adopts a grid based, ground plane clustering with 0.5 m cells and 8 connectivity. Clusters with ≥3 points are kept. This achieves near O(N) behavior and is less brittle under sparse returns than DBSCAN at long range.
Per cluster features. For each cluster we compute an 11 D vector:
	bounding dimensions w,h,d;
	centroid (x ˉ,y ˉ,z ˉ);
	top height y_top;
	point count N;
	base area A_base=w⋅d;
	width to depth ratio ρ;
	height to width ratio γ;
These features support the geometry aware classification and height based rules


4.4 Multi Target Tracking and Association
Project maintain object identities with nearest neighbour association under a constant velocity prediction. The association gate expands with speed (adaptive gating), and we use two parameter regimes: a conservative gate for pedestrians and a wider, speed scaled gate for e scooters to preserve continuity during fast motion and sparse frames.
To reduce erroneous “box jumping,” project add inertia based validation that checks direction consistency via the cosine of the angle between historical and proposed motion vectors, and displacement consistency against a speed scaled prediction error. The project uses stricter thresholds for pedestrians and more tolerant thresholds for e scooters, reflecting their higher accelerations and intermittently sparse point clouds. After conversion to an e scooter track, we also apply an identity lock so that associations within a reasonable gate are trusted even when instantaneous features are noisy.
Track management. Unmatched clusters start new tracks; matched ones are smoothed with EWMA. We found that fast e scooters often suffer short dropouts, so project set 12 frames for e scooter tracks, while keeping pedestrians at 3 frames to limit false persistence. This change was key to reducing fragmentation in high speed scenes.

4.5 E Scooter Classification: Dual Track Rules and Three Level Conversion
Classifier is state dependent. I used strict rules to convert a generic track into an “e scooter+rider” class, and relaxed rules to keep tracking once converted. This dual track design controls false positives at the entry stage while maintaining continuity afterwards.
	Geometry and centroid height.
In the radar relative frame, I require size ranges consistent with the measured scooter+rider footprint and dispersion. Typical working limits are: horizontal 0.25–1.80 m, vertical extent ≥0.50 m, and point count N≥3. The centroid height y ˉis the most informative: for conversion we use y ˉ≥0.85" m" , and for ongoing association we relax to y ˉ≥0.60" m" . These thresholds exploit the fact that scooter+rider centroids sit higher than pedestrians in our mounting geometry.
	Speed based, three level conversion.
The project set class conversion using a stratified policy:
{█(L_0 (very high speed): v≥4.0 m/sand y ˉ≥0.75 m                                             @L_1 (high speed): v≥2.8 m/sand y ˉ≥0.85 mwith relaxed geometry         @L_2 (medium speed): v≥2.0 m/s,strict geometry,and duration ≥1.0 s)┤
This extends coverage to slow to moderate riders without increasing false positives from joggers. The box will turn red and report when v≥5.56" m/s" (20 km/h).
	Pedestrian confirmation.
When the track duration, vertical span, and speed fall in walking/jogging envelopes, the system will confirm pedestrian with the score and latch scheme.

4.6 Implementation and Runtime
The full pipeline is implemented in Python with PySerial, NumPy and PyQt/OpenGL for real time visualization. The application boots into a pre configured Python 3.11 venv, fixes Qt plugin paths automatically, and reads CSVs or live streams. The 3 D viewer shows raw points, class coloured bounding boxes, and per ID speed labels. On my setup the per frame processing meets real time at 10 Hz; GUI rendering is the main overhead.
4.7 Parameterisation and Height Compensation
All y related thresholds (e.g., centroid height) are specified relative to the radar and can be translated to true height by adding 0.45 m. For example, a scooter+rider centroid of 0.55–1.45 m in the point cloud corresponds to ~1.0–1.9 m above ground. The analysis notes provide quick tables to verify this mapping and to debug unusual cases (e.g., negative y near the ground).
To cope with sparse frames, I allow mild miss forgiveness and wider gates for already converted e scooter tracks. For denser frames, we tighten point count and size checks. 

