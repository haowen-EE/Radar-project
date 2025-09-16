"""E-scooter detection and tracking plugin."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Centralised configuration for scooter-specific thresholds/parameters.
SCOOTER_CFG: Dict[str, float] = {
    "DBSCAN_EPS": 0.50,
    "DBSCAN_MIN_SAMPLES": 20,
    "LEN_MIN": 1.0,
    "LEN_MAX": 1.6,
    "WID_MIN": 0.20,
    "WID_MAX": 0.80,
    "AXIS_RATIO_MIN": 1.8,
    "D_NEAR_MIN": 0.16,
    "D_NEAR_MAX": 0.56,
    "TRACK_ASSOC_GATE": 1.2,
    "GATE_SPEED_FACTOR": 1.2,
    "TRACK_MAX_MISS": 8,
    "SPEED_WINDOW": 15,
    "VP90_MIN_COUNT": 10,
    "SPEED_ALPHA": 0.35,
    "SPEED_SANITY_MAX": 15.0,
    "VP90_PEDESTRIAN_MAX": 3.5,
    "VP90_NORMAL_MIN": 3.5,
    "VP90_DANGER_MIN": 5.5,
    "DISPLAY_HEIGHT_MIN": 0.0,
    "DISPLAY_HEIGHT_MAX": 0.18,
    "LABEL_HEIGHT_OFFSET": 0.25,
    "HIST_MIN_BINS": 6,
    "HIST_MAX_BINS": 24
}


@dataclass
class ScooterCandidate:
    indices: np.ndarray
    center: np.ndarray  # (x, z)
    length: float
    width: float
    axis_ratio: float
    d_near: float
    u_axis: np.ndarray
    v_axis: np.ndarray
    u_span: Tuple[float, float]
    v_span: Tuple[float, float]
    u_peak: float


class ScooterTrack:
    _next_id = 1

    def __init__(self, candidate: ScooterCandidate, timestamp: float, cfg: Dict[str, float]):
        self.id = ScooterTrack._next_id
        ScooterTrack._next_id += 1
        self.last_candidate = candidate
        self.last_center = candidate.center.copy()
        self.last_time = float(timestamp)
        self.velocity = np.zeros(2, dtype=float)
        self.speed_smooth = 0.0
        self.speeds = deque(maxlen=int(cfg["SPEED_WINDOW"]))
        self.miss = 0

    @classmethod
    def reset_ids(cls) -> None:
        cls._next_id = 1

    def predict(self, dt: float) -> np.ndarray:
        if not np.isfinite(dt) or dt <= 0.0:
            return self.last_center.copy()
        return self.last_center + self.velocity * dt

    def update(self, candidate: ScooterCandidate, timestamp: float, cfg: Dict[str, float]) -> None:
        dt = float(timestamp) - self.last_time if self.last_time is not None else 0.0
        center = candidate.center
        inst_speed = None
        if dt > 1e-3:
            disp = center - self.last_center
            dist = float(np.linalg.norm(disp))
            if dist > 0.0:
                inst_speed = min(dist / dt, cfg["SPEED_SANITY_MAX"])
                self.velocity = disp / dt
        if inst_speed is not None:
            self.speeds.append(inst_speed)
            if self.speed_smooth <= 0.0:
                self.speed_smooth = inst_speed
            else:
                alpha = cfg["SPEED_ALPHA"]
                self.speed_smooth = (1.0 - alpha) * self.speed_smooth + alpha * inst_speed
        self.last_candidate = candidate
        self.last_center = center.copy()
        self.last_time = float(timestamp)
        self.miss = 0

    def increment_miss(self) -> None:
        self.miss += 1

    def vp90(self, cfg: Dict[str, float]) -> float:
        if not self.speeds:
            return 0.0
        arr = np.clip(np.array(self.speeds, dtype=float), 0.0, cfg["SPEED_SANITY_MAX"])
        return float(np.percentile(arr, 90))

    def label(self, cfg: Dict[str, float]) -> Tuple[Optional[str], float]:
        if self.last_candidate is None:
            return None, 0.0
        if len(self.speeds) < int(cfg["VP90_MIN_COUNT"]):
            return None, self.vp90(cfg)
        vp90 = self.vp90(cfg)
        if vp90 >= cfg["VP90_DANGER_MIN"]:
            return "ESCOOTER-DANGER", vp90
        if vp90 > cfg["VP90_NORMAL_MIN"]:
            return "ESCOOTER-NORMAL", vp90
        return None, vp90


def _dbscan_clusters(points: np.ndarray, eps: float, min_samples: int) -> List[np.ndarray]:
    if points.size == 0:
        return []
    n = points.shape[0]
    eps_sq = float(eps) ** 2
    visited = np.zeros(n, dtype=bool)
    labels = np.full(n, -1, dtype=int)
    cluster_id = 0

    def region_query(i: int) -> np.ndarray:
        diff = points - points[i]
        dist_sq = diff[:, 0] ** 2 + diff[:, 1] ** 2
        return np.flatnonzero(dist_sq <= eps_sq)

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        neighbours = region_query(i)
        if neighbours.size < min_samples:
            continue
        labels[i] = cluster_id
        seeds = deque(neighbours.tolist())
        while seeds:
            j = seeds.popleft()
            if not visited[j]:
                visited[j] = True
                nbh = region_query(j)
                if nbh.size >= min_samples:
                    for nb in nbh:
                        if nb not in seeds:
                            seeds.append(nb)
            if labels[j] == -1:
                labels[j] = cluster_id
        cluster_id += 1

    clusters: List[np.ndarray] = []
    for cid in range(cluster_id):
        idxs = np.flatnonzero(labels == cid)
        if idxs.size:
            clusters.append(idxs)
    return clusters


def _dominant_peak(u_values: np.ndarray, cfg: Dict[str, float]) -> float:
    if u_values.size == 0:
        return 0.0
    bins = int(np.ceil(np.sqrt(u_values.size)))
    bins = max(int(cfg["HIST_MIN_BINS"]), min(int(cfg["HIST_MAX_BINS"]), bins))
    if bins <= 1:
        return float(np.mean(u_values))
    counts, edges = np.histogram(u_values, bins=bins)
    if not np.any(counts):
        return float(np.mean(u_values))
    peak_idx = int(np.argmax(counts))
    return float(0.5 * (edges[peak_idx] + edges[peak_idx + 1]))


def oriented_box_segments(center: Sequence[float], u_axis: Sequence[float], v_axis: Sequence[float],
                          length: float, width: float,
                          height: Tuple[float, float]) -> np.ndarray:
    cx, cz = float(center[0]), float(center[1])
    h0, h1 = float(height[0]), float(height[1])
    u = np.array(u_axis, dtype=float)
    v = np.array(v_axis, dtype=float)
    if np.linalg.norm(u) < 1e-6 or np.linalg.norm(v) < 1e-6:
        raise ValueError("Axis vectors must be non-zero")
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    hl = 0.5 * float(length)
    hw = 0.5 * float(width)
    base = np.array([cx, h0, cz], dtype=float)
    u3 = np.array([u[0], 0.0, u[1]], dtype=float)
    v3 = np.array([v[0], 0.0, v[1]], dtype=float)

    base_pts = [
        base + u3 * hl + v3 * hw,
        base + u3 * hl - v3 * hw,
        base - u3 * hl - v3 * hw,
        base - u3 * hl + v3 * hw,
    ]
    top_pts = [pt + np.array([0.0, h1 - h0, 0.0], dtype=float) for pt in base_pts]

    edges: List[Tuple[np.ndarray, np.ndarray]] = []
    for i in range(4):
        j = (i + 1) % 4
        edges.append((base_pts[i], base_pts[j]))
    for i in range(4):
        j = (i + 1) % 4
        edges.append((top_pts[i], top_pts[j]))
    for i in range(4):
        edges.append((base_pts[i], top_pts[i]))

    segs = [p for edge in edges for p in edge]
    return np.array(segs, dtype=float)


def _analyse_cluster(points: np.ndarray, idxs: np.ndarray, cfg: Dict[str, float]) -> Optional[ScooterCandidate]:
    xz = points[:, [0, 2]]
    center = xz.mean(axis=0)
    demean = xz - center
    cov = np.cov(demean.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, order]
    u_axis = eigvecs[:, 0]
    v_axis = eigvecs[:, 1]
    u_coords = demean @ u_axis
    v_coords = demean @ v_axis
    u_min, u_max = float(u_coords.min()), float(u_coords.max())
    v_min, v_max = float(v_coords.min()), float(v_coords.max())
    length = u_max - u_min
    width = v_max - v_min
    if width <= 1e-3 or length <= 1e-3:
        return None
    axis_ratio = length / width
    u_peak = _dominant_peak(u_coords, cfg)
    d_near = min(abs(u_max - u_peak), abs(u_peak - u_min))

    if not (cfg["LEN_MIN"] <= length <= cfg["LEN_MAX"]):
        return None
    if not (cfg["WID_MIN"] <= width <= cfg["WID_MAX"]):
        return None
    if axis_ratio < cfg["AXIS_RATIO_MIN"]:
        return None
    if not (cfg["D_NEAR_MIN"] <= d_near <= cfg["D_NEAR_MAX"]):
        return None

    return ScooterCandidate(
        indices=idxs.copy(),
        center=center.astype(float),
        length=float(length),
        width=float(width),
        axis_ratio=float(axis_ratio),
        d_near=float(d_near),
        u_axis=u_axis.astype(float),
        v_axis=v_axis.astype(float),
        u_span=(u_min, u_max),
        v_span=(v_min, v_max),
        u_peak=float(u_peak)
    )


class EscooterPlugin:
    """DBSCAN-based detector plus lightweight tracker for e-scooters."""

    def __init__(self, cfg: Optional[Dict[str, float]] = None):
        self.cfg = SCOOTER_CFG.copy()
        if cfg:
            self.cfg.update(cfg)
        self.tracks: List[ScooterTrack] = []

    def reset(self) -> None:
        self.tracks.clear()
        ScooterTrack.reset_ids()

    def _detect_candidates(self, pts: np.ndarray) -> List[ScooterCandidate]:
        if pts.size == 0:
            return []
        eps = float(self.cfg["DBSCAN_EPS"])
        min_samples = int(self.cfg["DBSCAN_MIN_SAMPLES"])
        xz = pts[:, [0, 2]]
        clusters = _dbscan_clusters(xz, eps=eps, min_samples=min_samples)
        candidates: List[ScooterCandidate] = []
        for idxs in clusters:
            cand = _analyse_cluster(pts[idxs], idxs, self.cfg)
            if cand is not None:
                candidates.append(cand)
        return candidates

    def _associate(self, candidates: List[ScooterCandidate], timestamp: float) -> None:
        existing_tracks = list(self.tracks)
        used_track_idx: set[int] = set()
        assignments = [-1] * len(candidates)
        for ci, cand in enumerate(candidates):
            best_idx = -1
            best_dist = None
            for ti, track in enumerate(existing_tracks):
                if ti in used_track_idx:
                    continue
                dt = float(timestamp) - track.last_time if track.last_time is not None else 0.0
                pred = track.predict(dt)
                gate = float(self.cfg["TRACK_ASSOC_GATE"])
                if track.speed_smooth > 0.0:
                    gate += track.speed_smooth * dt * float(self.cfg["GATE_SPEED_FACTOR"])
                dist = float(np.linalg.norm(cand.center - pred))
                if dist <= gate and (best_dist is None or dist < best_dist):
                    best_dist = dist
                    best_idx = ti
            if best_idx >= 0:
                assignments[ci] = best_idx
                used_track_idx.add(best_idx)

        for ti, track in enumerate(existing_tracks):
            if ti not in used_track_idx:
                track.increment_miss()

        for ci, cand in enumerate(candidates):
            ti = assignments[ci]
            if ti >= 0:
                existing_tracks[ti].update(cand, timestamp, self.cfg)
            else:
                new_track = ScooterTrack(cand, timestamp, self.cfg)
                self.tracks.append(new_track)

        max_miss = int(self.cfg["TRACK_MAX_MISS"])
        self.tracks = [tr for tr in self.tracks if tr.miss <= max_miss]

    def process_frame(self, pts: np.ndarray, timestamp: float, frame_idx: Optional[int] = None) -> List[Dict[str, object]]:
        candidates = self._detect_candidates(pts)
        if candidates:
            self._associate(candidates, timestamp)
        else:
            for tr in self.tracks:
                tr.increment_miss()
            max_miss = int(self.cfg["TRACK_MAX_MISS"])
            self.tracks = [tr for tr in self.tracks if tr.miss <= max_miss]

        results: List[Dict[str, object]] = []
        h = (self.cfg["DISPLAY_HEIGHT_MIN"], self.cfg["DISPLAY_HEIGHT_MAX"])
        offset = float(self.cfg["LABEL_HEIGHT_OFFSET"])
        for tr in sorted(self.tracks, key=lambda t: t.id):
            label, vp90 = tr.label(self.cfg)
            if label and tr.last_candidate is not None:
                cand = tr.last_candidate
                results.append({
                    "id": tr.id,
                    "label": label,
                    "vp90": vp90,
                    "center": cand.center.copy(),
                    "length": cand.length,
                    "width": cand.width,
                    "u_axis": cand.u_axis.copy(),
                    "v_axis": cand.v_axis.copy(),
                    "height": h,
                    "label_pos": np.array([
                        cand.center[0],
                        h[1] + offset,
                        cand.center[1]
                    ], dtype=float),
                    "d_near": cand.d_near,
                    "axis_ratio": cand.axis_ratio
                })
        return results
