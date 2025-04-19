# Campus Safety Radar Project

Using the TI AWR1843 77 GHz FMCW millimeter‑wave radar combined with GPU acceleration, this project develops an intelligent real‑time monitoring system for detecting and classifying pedestrians and electric scooters (e‑scooters), aiming to enhance campus safety and situational awareness.

---

## I. Project Overview
- **Privacy‑Friendly**: Only senses distance and velocity; does not record images or personal identities.  
- **All‑Weather Operation**: Unaffected by lighting, rain, or fog.  
- **High‑Precision Classification**: Distinguishes between pedestrians and e‑scooters and recognizes behaviors (e.g., normal walking/riding, running, falls).  
- **Multi‑Target Tracking**: Maintains individual target identities even in high‑density scenarios.

> **Note**: This README is based on the project proposal by Haowen Jiang (16 Apr 2025). For full details, see `doc/Proposal.pdf`. :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}

---

## II. Research Objectives
1. **System Deployment**  
   Install the TI AWR1843 radar to capture pedestrian and e‑scooter traffic data on campus.  
2. **Algorithm Development**  
   - Implement target detection using micro‑Doppler feature extraction and CFAR.  
   - Combine rule‑based and machine‑learning methods (SVM, Random Forest, lightweight neural nets) to classify pedestrians vs. e‑scooters.  
3. **Multi‑Target Tracking**  
   Employ Kalman filtering and data‑association techniques (GNN/JPDA) to maintain consistent target IDs.  
4. **Performance Evaluation**  
   Validate detection range (~30 m), classification accuracy (≥ 70%), tracking stability, and system latency (≤ 100 ms) in real‑world scenarios.

---

## III. Technical Background
- **Millimeter‑Wave Radar (77 GHz FMCW)**  
  - 3 × TX & 4 × RX, 4 GHz bandwidth, enabling centimeter‑level range resolution and degree‑level angular resolution.  
  - Unaffected by ambient light or weather; provides only distance/velocity data, ensuring privacy.  
- **Micro‑Doppler Effects**  
  Limb and wheel motions generate distinctive frequency signatures for behavior classification.  
- **GPU Acceleration**  
  CUDA‑based implementation of range/Doppler FFTs, CFAR detection, clutter removal, and clustering—significantly reducing end‑to‑end latency.

---

## IV. Methodology Overview
1. **Sensor Deployment**  
   - Mount the AWR1843 at campus pathways to cover a ~30 m radius.  
   - Synchronously record contextual ground‑truth data for annotation.  
2. **Signal Processing**  
