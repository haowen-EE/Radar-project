from collections import deque
from typing import Dict, List, Optional

import numpy as np


SCOOTER_CFG: Dict[str, float] = {
    "dbscan_eps": 0.50,                    # 聚类半径 (m)
    "dbscan_min_samples": 20,              # 最小点数
    "assoc_distance": 1.4,                 # 轨迹关联门限 (m)
    "max_miss": 6,                         # 最大未关联帧
    "history_size": 80,                    # 轨迹位置窗口
    "speed_history_size": 60,              # 速度窗口
    "min_speed_samples": 10,               # 速度分位数最少样本
    "speed_sanity_max": 15.0,              # 速度上限 (m/s)
    "speed_disp_factor": 1.5,              # 位移合理倍数
    "dt_floor": 1e-3,                      # dt 下限
    "length_range": (1.0, 1.6),            # OBB 长度范围
    "width_range": (0.20, 0.80),           # OBB 宽度范围
    "axis_ratio_min": 1.8,                 # 长宽轴比
    "d_near_range": (0.16, 0.56),          # 乘员近端距离范围
    "peak_hist_bins": 24,                  # u 轴主峰直方图 bins
    "normal_speed_min": 3.5,               # vp90 正常阈值 (m/s)
    "danger_speed_min": 5.5,               # vp90 危险阈值 (m/s)
    "line_width": 3.0,                     # 绘制线宽
    "color_pending": (1.0, 1.0, 0.0, 1.0), # 未达到速度阈值颜色
    "color_normal": (1.0, 0.55, 0.0, 1.0), # 正常颜色（橙色）
    "color_danger": (1.0, 0.1, 0.1, 1.0),  # 危险颜色（红）
    "label_height_offset": 0.15,           # 文字高度偏移
    "display_miss_tolerance": 1,           # 允许未关联的显示帧数
}


def _pairwise_neighbors(points: np.ndarray, eps: float) -> List[np.ndarray]:
    """Naive 邻域查询缓存."""

    if points.size == 0:
        return []
    eps2 = float(eps) ** 2
    neighbors: List[np.ndarray] = []
    for i in range(points.shape[0]):
        diff = points - points[i]
        dist2 = np.sum(diff * diff, axis=1)
        neighbors.append(np.where(dist2 <= eps2)[0])
    return neighbors


def dbscan(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """简易 DBSCAN（2D/3D 均可）。"""

    n = int(points.shape[0])
    if n == 0:
        return np.empty(0, dtype=int)

    UNCLASSIFIED = -2
    NOISE = -1
    labels = np.full(n, UNCLASSIFIED, dtype=int)
    neighbor_cache = _pairwise_neighbors(points, eps)
    cluster_id = 0

    for i in range(n):
        if labels[i] != UNCLASSIFIED:
            continue
        neigh = neighbor_cache[i]
        if neigh.size < min_samples:
            labels[i] = NOISE
            continue

        cluster_id += 1
        labels[i] = cluster_id
        seeds = set(int(idx) for idx in neigh.tolist())
        seeds.discard(i)
        queue = deque(seeds)
        while queue:
            j = queue.popleft()
            if labels[j] == NOISE:
                labels[j] = cluster_id
            if labels[j] != UNCLASSIFIED:
                continue
            labels[j] = cluster_id
            neigh_j = neighbor_cache[j]
            if neigh_j.size >= min_samples:
                for idx in neigh_j:
                    if idx not in seeds:
                        seeds.add(int(idx))
                        queue.append(int(idx))

    labels[labels > 0] -= 1  # 归零起始
    return labels


def _obb_lines(bottom: np.ndarray, top: np.ndarray) -> np.ndarray:
    """生成 OBB 线段 (24,3)."""

    idx_pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
    segs: List[np.ndarray] = []
    for a, b in idx_pairs:
        segs.extend([bottom[a], bottom[b]])
    for a, b in idx_pairs:
        segs.extend([top[a], top[b]])
    for i in range(4):
        segs.extend([bottom[i], top[i]])
    return np.array(segs, dtype=float)


class ScooterTrack:
    _next_id = 1

    def __init__(self, center: np.ndarray, timestamp: float, feature: Dict, cfg: Dict, dt_hint: float):
        self.id = ScooterTrack._next_id
        ScooterTrack._next_id += 1
        self.cfg = cfg
        self.dt_hint = float(dt_hint)
        self.history = deque(maxlen=int(cfg["history_size"]))
        self.time_history = deque(maxlen=int(cfg["history_size"]))
        self.speed_samples = deque(maxlen=int(cfg["speed_history_size"]))
        self.last_feature: Optional[Dict] = None
        self.vp90: Optional[float] = None
        self.status: Optional[str] = None
        self.miss = 0
        self.update(center, timestamp, feature)

    @classmethod
    def reset_ids(cls) -> None:
        cls._next_id = 1

    def distance_to(self, center: np.ndarray) -> float:
        if not self.history:
            return float("inf")
        last = self.history[-1]
        return float(np.hypot(last[0] - center[0], last[1] - center[1]))

    def update(self, center: np.ndarray, timestamp: float, feature: Dict) -> None:
        center = np.asarray(center, dtype=float)
        timestamp = float(timestamp)
        self.history.append(center)
        self.time_history.append(timestamp)
        if len(self.history) >= 2:
            prev = self.history[-2]
            prev_t = self.time_history[-2]
            dt = timestamp - prev_t
            if dt <= self.cfg["dt_floor"]:
                dt = max(self.dt_hint, self.cfg["dt_floor"])
            disp = float(np.hypot(center[0] - prev[0], center[1] - prev[1]))
            if dt > 0:
                inst = disp / dt
                if inst <= self.cfg["speed_sanity_max"] and disp <= self.cfg["speed_sanity_max"] * dt * self.cfg["speed_disp_factor"]:
                    self.speed_samples.append(inst)
        self.last_feature = feature
        self.miss = 0
        self._update_status()

    def _update_status(self) -> None:
        if len(self.speed_samples) >= self.cfg["min_speed_samples"]:
            arr = np.array(self.speed_samples, dtype=float)
            self.vp90 = float(np.percentile(arr, 90))
            if self.vp90 > self.cfg["danger_speed_min"]:
                self.status = "ESCOOTER-DANGER"
            elif self.vp90 > self.cfg["normal_speed_min"]:
                self.status = "ESCOOTER-NORMAL"
            else:
                self.status = None
        else:
            self.vp90 = None
            self.status = None

    def should_display(self) -> bool:
        if self.last_feature is None:
            return False
        if self.status is None:
            return False
        if self.miss > self.cfg["display_miss_tolerance"]:
            return False
        return True

    def render_payload(self) -> Optional[Dict]:
        if not self.should_display():
            return None
        feature = self.last_feature
        if feature is None:
            return None
        color = self.cfg["color_normal"] if self.status == "ESCOOTER-NORMAL" else self.cfg["color_danger"]
        text = f"{self.status} #{self.id} {self.vp90:.2f} m/s" if self.vp90 is not None else f"{self.status} #{self.id}"
        return {
            "lines": feature["lines"],
            "color": color,
            "line_width": self.cfg["line_width"],
            "label_pos": feature["label_pos"],
            "label_text": text,
        }


class EscooterPlugin:
    def __init__(self, cfg: Optional[Dict] = None, dt_hint: float = 0.05):
        self.cfg = dict(SCOOTER_CFG)
        if cfg:
            self.cfg.update(cfg)
        self.dt_hint = float(dt_hint)
        self.tracks: List[ScooterTrack] = []

    def reset(self) -> None:
        self.tracks.clear()
        ScooterTrack.reset_ids()

    def process_frame(self, points: np.ndarray, timestamp: float, frame_idx: Optional[int] = None) -> List[Dict]:
        timestamp = float(timestamp)
        pts = np.asarray(points, dtype=float)
        results: List[Dict] = []

        candidates = self._extract_candidates(pts)
        for tr in self.tracks:
            tr.miss += 1

        used_tracks: set = set()
        for feat in candidates:
            center = feat["center2d"]
            best_track = None
            best_dist = self.cfg["assoc_distance"]
            for tr in self.tracks:
                if tr in used_tracks:
                    continue
                d = tr.distance_to(center)
                if d <= best_dist:
                    best_dist = d
                    best_track = tr
            if best_track is not None:
                used_tracks.add(best_track)
                best_track.update(center, timestamp, feat)
            else:
                track = ScooterTrack(center, timestamp, feat, self.cfg, self.dt_hint)
                self.tracks.append(track)

        survivors: List[ScooterTrack] = []
        for tr in self.tracks:
            if tr.miss > self.cfg["max_miss"]:
                continue
            survivors.append(tr)
            payload = tr.render_payload()
            if payload:
                results.append(payload)
        self.tracks = survivors
        return results

    # === feature 计算 ===
    def _extract_candidates(self, pts3d: np.ndarray) -> List[Dict]:
        if pts3d.size == 0:
            return []
        planar = pts3d[:, [0, 2]]
        labels = dbscan(planar, self.cfg["dbscan_eps"], int(self.cfg["dbscan_min_samples"]))
        clusters: List[Dict] = []
        for cid in np.unique(labels):
            if cid < 0:
                continue
            idxs = np.where(labels == cid)[0]
            clusters.append(self._analyze_cluster(pts3d[idxs]))
        return [c for c in clusters if c is not None]

    def _analyze_cluster(self, pts: np.ndarray) -> Optional[Dict]:
        if pts.shape[0] < int(self.cfg["dbscan_min_samples"]):
            return None
        planar = pts[:, [0, 2]]
        mean = planar.mean(axis=0)
        centered = planar - mean
        cov = np.cov(centered, rowvar=False)
        if not np.all(np.isfinite(cov)):
            return None
        vals, vecs = np.linalg.eigh(cov)
        order = np.argsort(vals)[::-1]
        vals = vals[order]
        vecs = vecs[:, order]
        if vecs.shape[1] < 2:
            return None
        u = vecs[:, 0]
        v = vecs[:, 1]
        proj_u = centered @ u
        proj_v = centered @ v
        u_min = float(np.min(proj_u))
        u_max = float(np.max(proj_u))
        v_min = float(np.min(proj_v))
        v_max = float(np.max(proj_v))
        length = u_max - u_min
        width = v_max - v_min
        if length <= 0 or width <= 0:
            return None
        axis_ratio = length / max(width, 1e-6)
        L0, L1 = self.cfg["length_range"]
        W0, W1 = self.cfg["width_range"]
        if not (L0 <= length <= L1 and W0 <= width <= W1 and axis_ratio >= self.cfg["axis_ratio_min"]):
            return None

        bins = max(int(self.cfg["peak_hist_bins"]), 4)
        hist, edges = np.histogram(proj_u, bins=bins)
        if hist.size == 0:
            return None
        peak_idx = int(np.argmax(hist))
        u_peak = 0.5 * (edges[peak_idx] + edges[peak_idx + 1])
        d_near = min(abs(u_max - u_peak), abs(u_peak - u_min))
        d0, d1 = self.cfg["d_near_range"]
        if not (d0 <= d_near <= d1):
            return None

        center_local = np.array([(u_min + u_max) * 0.5, (v_min + v_max) * 0.5], dtype=float)
        center_world = mean + u * center_local[0] + v * center_local[1]
        ymin = float(np.min(pts[:, 1]))
        ymax = float(np.max(pts[:, 1]))
        label_height = ymax + self.cfg["label_height_offset"]

        corners_local = np.array([
            [u_min, v_min],
            [u_max, v_min],
            [u_max, v_max],
            [u_min, v_max],
        ], dtype=float)
        corners_world = (mean + corners_local @ np.stack([u, v], axis=1).T).reshape(4, 2)
        bottom = np.column_stack((corners_world[:, 0], np.full(4, ymin), corners_world[:, 1]))
        top = np.column_stack((corners_world[:, 0], np.full(4, ymax), corners_world[:, 1]))

        feature = {
            "center2d": center_world,
            "length": length,
            "width": width,
            "axis_ratio": axis_ratio,
            "d_near": d_near,
            "lines": _obb_lines(bottom, top),
            "label_pos": np.array([center_world[0], label_height, center_world[1]], dtype=float),
        }
        return feature


__all__ = ["EscooterPlugin", "SCOOTER_CFG"]
