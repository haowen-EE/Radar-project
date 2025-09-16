import csv, math, os
from collections import defaultdict, deque
from datetime import datetime
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl

from escooter_plugin import EscooterPlugin, oriented_box_segments


# ================= 配置 =================

CSV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'man_run_without_RSC.csv')

# 坐标方向：绕 y 轴 +90° (True) 或无旋转 (False)
ROTATE_Y_PLUS_90_FOR_X_ALIGNED_WALK = False

# 聚类 & 关联
GRID_CELL_M = 0.7
MIN_POINTS_IN_CLUSTER = 3
ASSOC_GATE_BASE_M = 2.4
MAX_MISS = 8

# 速度与稳定性
ROLL_WIN = 40               # 轨迹窗口帧
EWMA_ALPHA = 0.35           # 质心指数平滑
SPEED_WIN_PAIR = 10         # 最近多少对相邻帧用于速度估计
SPEED_SANITY_MAX = 9.0      # m/s：瞬时速度合理上限
DISP_FACTOR = 1.25          # 位移合理上限：disp <= SPEED_SANITY_MAX * dt * DISP_FACTOR

# 判定阈值（步行）
WALK_SPEED_LO = 0.3
WALK_SPEED_HI = 7.0
MIN_DURATION_S = 0.5
Y_EXTENT_MIN = 0.35

# 抗抖显示（得分 & 锁存）
CONFIRM_SCORE = 3
SCORE_HIT = 2
SCORE_MISS = 1
LATCH_S = 1.0

# === Object（静小物体）识别参数 ===
OBJ_SPEED_MAX = 0.20          # m/s：近乎静止的上限
OBJ_MIN_DURATION_S = 0.5      # s：最短持续时间
OBJ_MAX_POINTS = 15           # 小体积点数上限（可调）
OBJ_MAX_VOL = 0.50            # m^3：bbox 体积上限
OBJ_MAX_Y_EXTENT = 1.00       # m：高度上限
OBJ_ASSOC_GATE = 0.55         # m：Object 轨迹再关联门限（再小一点，增强抗吸附）
OBJ_CONFIRM_SCORE = 3
OBJ_SCORE_HIT = 2
OBJ_SCORE_MISS = 1
OBJ_LATCH_S = 1.2             # s：锁存显示防抖（略增）

# === 近邻分裂 + 保护圈 ===
OBJ_EXCLUDE_R = 0.50          # m：Object 保护圈半径（圈内点优先归小物体）
OBJ_OCCLU_HOLD_S = 0.6        # s：疑似遮挡时额外延长 Object 锁存
OBJ_OCCLU_MISS_RELIEF = 1     # 遮挡时，对 miss 的减免（最多减到不增加）

# 绘制
POINT_SIZE = 3
PT_COLOR = (1, 0, 0, 1)
BOX_COLOR = (0, 1, 0, 1)
BOX_WIDTH = 2
LABEL_SPEED = True

# Object 绘制
OBJ_BOX_COLOR = (0, 0.6, 1, 1)  # 青蓝色
OBJ_BOX_WIDTH = 2
LABEL_OBJECT = True

# Scooter 绘制
SCOOTER_BOX_WIDTH = 2.5
SCOOTER_COLOR_NORMAL = (1.0, 0.7, 0.0, 1.0)
SCOOTER_COLOR_DANGER = (1.0, 0.0, 0.0, 1.0)
SCOOTER_LABEL_COLOR = {
    "ESCOOTER-NORMAL": QtGui.QColor(255, 170, 0),
    "ESCOOTER-DANGER": QtGui.QColor(255, 64, 64),
}
# =======================================

try:
    from pyqtgraph.opengl import GLTextItem
    HAS_GLTEXT = True
except Exception:
    HAS_GLTEXT = False

# === 方框稳定化（固定/缓变） ===
BOX_SIZE_MODE = 'fixed'                 # 'fixed' | 'smooth' | 'raw'
BOX_FIXED_WHD = (0.6, 1.7, 0.6)        # (x 宽, y 高, z 深)
BOX_SMOOTH_ALPHA = 0.25
BOX_DELTA_CLAMP = 0.12
BOX_SIZE_MIN = (0.4, 1.4, 0.4)
BOX_SIZE_MAX = (0.9, 2.0, 0.9)
BOX_ANCHOR = 'center'                   # 'center' | 'ground'
BOX_PAD = 0.02
BOX_SIZE_MIN_ARR = np.array(BOX_SIZE_MIN, float)
BOX_SIZE_MAX_ARR = np.array(BOX_SIZE_MAX, float)


def transform_xyz(x, y, z):
    return (z, y, -x) if ROTATE_Y_PLUS_90_FOR_X_ALIGNED_WALK else (x, y, z)


def load_frames(csv_file):
    """按 detIdx==0 分帧，返回 data, frames, rel_t, abs_t, dt_med"""
    data, time_map, fid = {}, {}, -1
    with open(csv_file, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            det_idx = int(row.get('detIdx', 0) or 0)
            if det_idx == 0:
                fid += 1
                data[fid] = []
                ts = row.get('timeStamp')
                if ts:
                    try:
                        time_map[fid] = datetime.fromisoformat(ts)
                    except Exception:
                        pass
            try:
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                if fid >= 0:
                    data[fid].append(transform_xyz(x, y, z))
            except Exception:
                continue
    frames = sorted(data.keys())
    if not frames:
        raise RuntimeError('No frames parsed from CSV')

    # 用帧起始时间戳算中位帧间隔
    times = [time_map.get(fid) for fid in frames]
    ts_valid = [t for t in times if isinstance(t, datetime)]
    if len(ts_valid) >= 3:
        t0 = ts_valid[0]
        secs = np.array([(t - t0).total_seconds() for t in ts_valid], float)
        dts = np.diff(secs); dts = dts[dts > 0]
        dt_med = float(np.median(dts)) if dts.size else 0.05
    else:
        dt_med = 0.05

    rel_t = np.cumsum([0.0] + [dt_med]*(len(frames)-1)).astype(float)

    if ts_valid:
        base = ts_valid[0]
        abs_list = []
        for i, fid in enumerate(frames):
            t = time_map.get(fid)
            abs_list.append((t - base).total_seconds() if isinstance(t, datetime) else float(rel_t[i]))
        abs_t = np.array(abs_list, float)
    else:
        abs_t = rel_t.copy()

    return data, frames, rel_t, abs_t, dt_med


def grid_cluster(pts, cell=GRID_CELL_M, min_points=MIN_POINTS_IN_CLUSTER):
    if len(pts) == 0:
        return []
    P = np.asarray(pts)
    x = P[:, 0]; y = P[:, 1]; z = P[:, 2]
    off = 1000.0 * cell
    ix = np.floor((x + off) / cell).astype(np.int64)
    iz = np.floor((z + off) / cell).astype(np.int64)
    cell_map = defaultdict(list)
    for i, (cx, cz) in enumerate(zip(ix, iz)):
        cell_map[(cx, cz)].append(i)

    vis, clusters = set(), []
    from collections import deque as dq
    for key in list(cell_map.keys()):
        if key in vis:
            continue
        q = dq([key]); vis.add(key); comp = []
        while q:
            c = q.popleft(); comp.append(c); cx, cz = c
            for dx in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dz == 0:
                        continue
                    n = (cx + dx, cz + dz)
                    if n in cell_map and n not in vis:
                        vis.add(n); q.append(n)
        idxs = []
        for c in comp:
            idxs.extend(cell_map[c])
        if len(idxs) >= min_points:
            cpts = P[idxs]
            xmin = float(cpts[:, 0].min()); xmax = float(cpts[:, 0].max())
            ymin = float(cpts[:, 1].min()); ymax = float(cpts[:, 1].max())
            zmin = float(cpts[:, 2].min()); zmax = float(cpts[:, 2].max())
            cxz = cpts[:, [0, 2]].mean(axis=0)
            clusters.append({
                "idxs": np.array(idxs, int),
                "pts": cpts,
                "centroid_xz": cxz,
                "bbox": (xmin, xmax, ymin, ymax, zmin, zmax)
            })
    return clusters


def make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax):
    c000 = np.array([xmin, ymin, zmin]); c100 = np.array([xmax, ymin, zmin])
    c010 = np.array([xmin, ymax, zmin]); c110 = np.array([xmax, ymax, zmin])
    c001 = np.array([xmin, ymin, zmax]); c101 = np.array([xmax, ymin, zmax])
    c011 = np.array([xmin, ymax, zmax]); c111 = np.array([xmax, ymax, zmax])
    edges = [
        (c000, c100), (c100, c110), (c110, c010), (c010, c000),
        (c001, c101), (c101, c111), (c111, c011), (c011, c001),
        (c000, c001), (c100, c101), (c110, c111), (c010, c011)
    ]
    return np.vstack([np.vstack(e) for e in edges])


# === Object：尺寸判定 ===
def bbox_dims(bbox):
    xmin, xmax, ymin, ymax, zmin, zmax = bbox
    w = max(0.0, xmax - xmin); h = max(0.0, ymax - ymin); d = max(0.0, zmax - zmin)
    return w, h, d


def is_small_object(bbox, npts):
    w, h, d = bbox_dims(bbox)
    vol = w * h * d
    small_by_geom = (vol <= OBJ_MAX_VOL and h <= OBJ_MAX_Y_EXTENT) or (max(w, d) <= 0.80 and h <= 1.0)
    small_by_pts = (npts <= OBJ_MAX_POINTS)
    return small_by_geom and small_by_pts


# === Object 容差判断：给已确认 Object 的再关联留余量 ===
def small_object_with_margin(bbox, npts, k_size=1.6, k_pts=1.8):
    w, h, d = bbox_dims(bbox)
    vol = w * h * d
    small_by_geom = (vol <= OBJ_MAX_VOL * k_size and h <= OBJ_MAX_Y_EXTENT * k_size)
    small_by_pts = (npts <= int(OBJ_MAX_POINTS * k_pts))
    return small_by_geom and small_by_pts


# === 近邻分裂：围绕 Object 保护圈把“大簇”一分为二（圈内→小物体；圈外→行人/其他） ===
def split_cluster_near_object(cluster, obj_cx, obj_cz, radius=OBJ_EXCLUDE_R):
    """
    返回 (success, small_subcluster, big_subcluster)
    - success=False 表示无需/无法切分
    - small_subcluster/ big_subcluster 与原 cluster 同样的数据结构
    """
    pts = cluster["pts"]
    if pts.shape[0] < 6:  # 太少没必要分
        return False, None, None
    xz = pts[:, [0, 2]]
    d = np.hypot(xz[:, 0] - obj_cx, xz[:, 1] - obj_cz)
    mask_in = d <= radius
    n_in = int(mask_in.sum()); n_out = int((~mask_in).sum())
    if n_in < 3 or n_out < 3:
        return False, None, None

    def mk(cpts, idxs):
        xmin = float(cpts[:, 0].min()); xmax = float(cpts[:, 0].max())
        ymin = float(cpts[:, 1].min()); ymax = float(cpts[:, 1].max())
        zmin = float(cpts[:, 2].min()); zmax = float(cpts[:, 2].max())
        cxz = cpts[:, [0, 2]].mean(axis=0)
        return {
            "idxs": idxs.copy(),
            "pts": cpts.copy(),
            "centroid_xz": cxz,
            "bbox": (xmin, xmax, ymin, ymax, zmin, zmax)
        }

    sub_in = mk(pts[mask_in], cluster["idxs"][mask_in])
    sub_out = mk(pts[~mask_in], cluster["idxs"][~mask_in])
    # 约束：圈内应当是“小物体”，否则认为切分无意义
    if not is_small_object(sub_in["bbox"], sub_in["pts"].shape[0]):
        return False, None, None
    return True, sub_in, sub_out


class Track:
    _next = 1

    def __init__(self, cx, cz, t, yext, bbox=None, frame_idx=0, npts=0):
        self.id = Track._next; Track._next += 1
        self.c_smooth = np.array([cx, cz], float)   # 平滑质心
        self.centroids = deque(maxlen=ROLL_WIN)     # 原始 (x,z)
        self.times = deque(maxlen=ROLL_WIN)         # 绝对秒
        self.frames = deque(maxlen=ROLL_WIN)        # 帧号
        self.y_exts = deque(maxlen=ROLL_WIN)
        self.miss = 0
        self.score = 0
        self.confirmed = False
        self.latch_until = 0.0
        self.last_bbox = bbox
        # 垂直位置与盒子尺寸的平滑状态
        self.yc_smooth = None
        self.y_base_smooth = None
        self.size = None

        # === Object 状态 ===
        self.last_npts = int(npts) if npts else 0
        self.obj_score = 0
        self.obj_confirmed = False
        self.obj_latch_until = 0.0

        # === Object 锁 ===
        self.lock_type = None   # None | 'object'

        self.add(cx, cz, t, yext, bbox=bbox, frame_idx=frame_idx, npts=npts)

    def add(self, cx, cz, t, yext, bbox=None, frame_idx=None, npts=None):
        self.c_smooth = (1 - EWMA_ALPHA) * self.c_smooth + EWMA_ALPHA * np.array([cx, cz], float)
        self.centroids.append((cx, cz))
        self.times.append(float(t))
        if frame_idx is not None:
            self.frames.append(int(frame_idx))
        self.y_exts.append(float(yext))
        if bbox is not None:
            self.last_bbox = bbox
            xmin, xmax, ymin, ymax, zmin, zmax = bbox
            yc = 0.5 * (ymin + ymax)
            self.yc_smooth = yc if self.yc_smooth is None else (1 - BOX_SMOOTH_ALPHA) * self.yc_smooth + BOX_SMOOTH_ALPHA * yc
            self.y_base_smooth = ymin if self.y_base_smooth is None else (1 - BOX_SMOOTH_ALPHA) * self.y_base_smooth + BOX_SMOOTH_ALPHA * ymin
            obs_size = np.array([xmax - xmin, ymax - ymin, zmax - zmin], float)
            self._update_size(obs_size)
        if npts is not None:
            self.last_npts = int(npts)
        self.miss = 0

    def last(self):
        return tuple(self.c_smooth.tolist())

    def last_frame(self):
        return self.frames[-1] if len(self.frames) > 0 else -1

    def duration(self):
        if len(self.times) < 2:
            return 0.0
        return self.times[-1] - self.times[0]

    def y_med(self):
        return float(np.median(self.y_exts)) if self.y_exts else 0.0

    def speed_robust(self):
        n = len(self.times)
        if n < 2:
            return 0.0
        k = min(n - 1, SPEED_WIN_PAIR)
        if k <= 0:
            return 0.0

        Cs = np.array(list(self.centroids)[-k - 1:], float)
        for i in range(1, Cs.shape[0]):
            Cs[i] = (1 - EWMA_ALPHA) * Cs[i - 1] + EWMA_ALPHA * Cs[i]
        Ts = np.array(list(self.times)[-k - 1:], float)

        dx = np.diff(Cs[:, 0]); dz = np.diff(Cs[:, 1])
        dt = np.diff(Ts)
        mask = dt > 0
        if not np.any(mask):
            return 0.0
        dx = dx[mask]; dz = dz[mask]; dt = dt[mask]

        disp = np.hypot(dx, dz)
        inst = disp / dt

        ok = (inst <= SPEED_SANITY_MAX) & (disp <= SPEED_SANITY_MAX * dt * DISP_FACTOR)
        good = inst[ok]
        if good.size == 0:
            return float(np.median(inst)) if inst.size else 0.0
        return float(np.median(good))

    def update_score_and_state(self, now):
        # 如果这条轨迹已被锁定为 Object，就不参与“行人”识别
        if self.lock_type == 'object':
            return False, self.speed_robust()

        dur = self.duration()
        v = self.speed_robust()
        ymed = self.y_med()
        ok = (dur >= MIN_DURATION_S) and (WALK_SPEED_LO <= v <= WALK_SPEED_HI) and (ymed >= Y_EXTENT_MIN)
        self.score = min(self.score + SCORE_HIT, 10) if ok else max(self.score - SCORE_MISS, 0)
        if (not self.confirmed) and self.score >= CONFIRM_SCORE:
            self.confirmed = True
            self.latch_until = now + LATCH_S
        if self.confirmed and self.score > 0:
            self.latch_until = max(self.latch_until, now + 0.2)
        show = self.confirmed and (now <= self.latch_until)
        return show, v

    # === 尺寸更新与用于显示的稳定方框 ===
    def _update_size(self, obs_size: np.ndarray):
        mode = BOX_SIZE_MODE
        if mode == 'fixed' or obs_size is None:
            self.size = np.array(BOX_FIXED_WHD, float); return
        if mode == 'raw':
            self.size = np.clip(obs_size, BOX_SIZE_MIN_ARR, BOX_SIZE_MAX_ARR); return
        if self.size is None:
            seed = np.clip(obs_size, BOX_SIZE_MIN_ARR, BOX_SIZE_MAX_ARR)
            base = np.array(BOX_FIXED_WHD, float)
            self.size = 0.5 * base + 0.5 * seed
        else:
            prev = self.size
            target = np.clip(obs_size, BOX_SIZE_MIN_ARR, BOX_SIZE_MAX_ARR)
            up = prev * (1.0 + BOX_DELTA_CLAMP)
            dn = prev * (1.0 - BOX_DELTA_CLAMP)
            target = np.minimum(np.maximum(target, dn), up)
            self.size = (1.0 - BOX_SMOOTH_ALPHA) * prev + BOX_SMOOTH_ALPHA * target

    def display_bbox(self):
        if self.size is None:
            self.size = np.array(BOX_FIXED_WHD, float)
        w, h, d = self.size
        cx, cz = self.c_smooth
        pad = float(BOX_PAD)
        if BOX_ANCHOR == 'ground' and self.y_base_smooth is not None:
            ymin = self.y_base_smooth - pad; ymax = ymin + h + 2 * pad
        else:
            yc = self.yc_smooth if self.yc_smooth is not None else 0.5 * h
            ymin = (yc - 0.5 * h) - pad; ymax = (yc + 0.5 * h) + pad
        xmin = (cx - 0.5 * w) - pad; xmax = (cx + 0.5 * w) + pad
        zmin = (cz - 0.5 * d) - pad; zmax = (cz + 0.5 * d) + pad
        return xmin, xmax, ymin, ymax, zmin, zmax

    # === Object 功能 ===
    def last_dims(self):
        if self.last_bbox is None:
            return 0.0, 0.0, 0.0
        return bbox_dims(self.last_bbox)

    def small_like(self):
        if self.last_bbox is None:
            return False
        return is_small_object(self.last_bbox, self.last_npts)

    def is_objectish(self):
        v = self.speed_robust()
        small = self.small_like()
        short_hist = (len(self.times) < 6)
        return (small and (v <= OBJ_SPEED_MAX * 1.5 or short_hist))

    def update_object_state(self, now):
        dur = self.duration()
        v = self.speed_robust()
        cond = (dur >= OBJ_MIN_DURATION_S) and (v <= OBJ_SPEED_MAX) and self.small_like()
        self.obj_score = min(self.obj_score + OBJ_SCORE_HIT, 10) if cond else max(self.obj_score - OBJ_SCORE_MISS, 0)
        if (not self.obj_confirmed) and self.obj_score >= OBJ_CONFIRM_SCORE:
            self.obj_confirmed = True
            self.obj_latch_until = now + OBJ_LATCH_S
            # === Object 锁 ===
            self.lock_type = 'object'
        if self.obj_confirmed and self.obj_score > 0:
            self.obj_latch_until = max(self.obj_latch_until, now + 0.2)
        return self.obj_confirmed and (now <= self.obj_latch_until)


def main():
    data, frames, rel_t, abs_t, dt_med = load_frames(CSV_FILE)
    total_time = rel_t[-1] if len(rel_t) else 0.0

    assoc_gate = max(ASSOC_GATE_BASE_M, SPEED_SANITY_MAX * dt_med * 2.0)

    app = QtWidgets.QApplication([])
    view = gl.GLViewWidget()
    view.opts['distance'] = 20
    view.setCameraPosition(azimuth=45, elevation=20, distance=20)
    view.setWindowTitle(f'Radar 3D Pedestrian (robust): 0.000 s [{os.path.basename(CSV_FILE)}]')
    view.show()
    axis = gl.GLAxisItem(); axis.setSize(x=10, y=10, z=10); view.addItem(axis)
    grid = gl.GLGridItem(); grid.setSize(10, 10); grid.setSpacing(1, 1); view.addItem(grid)
    scatter = gl.GLScatterPlotItem(size=POINT_SIZE, color=PT_COLOR); view.addItem(scatter)

    box_items, text_items = [], []
    tracks = []
    scooter_plugin = EscooterPlugin()

    timer = QtCore.QTimer()
    timer.setInterval(max(int(dt_med * 1000), 20))  # 兜底 20ms
    elapsed = QtCore.QElapsedTimer(); elapsed.start()
    idx = 0

    def clear_tracks():
        nonlocal tracks
        tracks.clear()
        Track._next = 1
        scooter_plugin.reset()

    def update():
        nonlocal idx, box_items, text_items, tracks
        sec = elapsed.elapsed() / 1000.0
        # 回放到头：重置并清空轨迹
        if total_time > 0 and sec > total_time:
            elapsed.restart(); idx = 0; sec = 0.0
            clear_tracks()

        while idx < len(frames) - 1 and sec >= rel_t[idx + 1]:
            idx += 1

        pts = np.array(data[frames[idx]], float) if data[frames[idx]] else np.zeros((0, 3))
        scatter.setData(pos=pts)

        # 清除上一帧图元
        for it in box_items: view.removeItem(it)
        box_items = []
        for it in text_items: view.removeItem(it)
        text_items = []

        # 聚类
        clusters = grid_cluster(pts)

        # 所有轨迹 miss+1
        for tr in tracks:
            tr.miss += 1

        # 先把“可能被合并的大簇”在 Object 周围切开
        if clusters and any(tr.obj_confirmed for tr in tracks):
            new_clusters = []
            used_flags = [False]*len(clusters)
            # 逐个 object 试图切分附近簇
            for tr in tracks:
                if not tr.obj_confirmed:
                    continue
                ocx, ocz = tr.last()
                for ci, c in enumerate(clusters):
                    if used_flags[ci]:
                        continue
                    # 只有“看起来不是小物体”的簇才有切分必要
                    if is_small_object(c["bbox"], c["pts"].shape[0]):
                        continue
                    # 与 Object 距离足够近才尝试切分
                    cx, cz = c["centroid_xz"]
                    if math.hypot(cx - ocx, cz - ocz) > max(OBJ_EXCLUDE_R*1.2, OBJ_ASSOC_GATE*1.5):
                        continue
                    ok, sub_in, sub_out = split_cluster_near_object(c, ocx, ocz, radius=OBJ_EXCLUDE_R)
                    if ok:
                        # 内圈（小物体）与外圈（行人）都保留
                        new_clusters.append(sub_in)
                        new_clusters.append(sub_out)
                        used_flags[ci] = True
            # 把未被切分的原簇加回来
            for ci, c in enumerate(clusters):
                if not used_flags[ci]:
                    new_clusters.append(c)
            clusters = new_clusters

        # 关联（两阶段：先 Object，再其他）
        used = [False] * len(clusters)

        # ---------- 阶段 A：只给“已确认的 Object”找“小物体型”簇 ----------
        obj_tracks = [tr for tr in tracks if tr.obj_confirmed]
        for tr in obj_tracks:
            best = None; bestd = None; best_ci = None
            px, pz = tr.last()
            for ci, c in enumerate(clusters):
                if used[ci]:
                    continue
                npts = len(c["idxs"])
                # 只考虑“小物体型 + 有余量”的簇，并使用更小的门限
                if not small_object_with_margin(c["bbox"], npts):
                    continue
                cx, cz = c["centroid_xz"]
                d = math.hypot(cx - px, cz - pz)
                if d <= OBJ_ASSOC_GATE and (bestd is None or d < bestd):
                    bestd = d; best = c; best_ci = ci
            if best is not None:
                used[best_ci] = True
                cx, cz = best["centroid_xz"]; xmin, xmax, ymin, ymax, zmin, zmax = best["bbox"]
                yext = ymax - ymin; npts = len(best["idxs"])
                tr.add(cx, cz, abs_t[idx], yext, bbox=best["bbox"], frame_idx=idx, npts=npts)
            else:
                # 疑似遮挡：附近若存在“大且快速”的簇，则延长锁存并减轻 miss
                near_big_fast = False
                for c in clusters:
                    cx, cz = c["centroid_xz"]
                    if math.hypot(cx - px, cz - pz) <= OBJ_EXCLUDE_R*1.5:
                        # 用 bbox 高/体积粗判“大”
                        w,h,d = bbox_dims(c["bbox"]); vol = w*h*d
                        if (h > OBJ_MAX_Y_EXTENT*1.2) or (vol > OBJ_MAX_VOL*2.0) or (len(c["idxs"]) > OBJ_MAX_POINTS*2):
                            near_big_fast = True; break
                if near_big_fast:
                    tr.obj_latch_until = max(tr.obj_latch_until, abs_t[idx] + OBJ_OCCLU_HOLD_S)
                    # 减轻 miss：相当于抵消本帧 miss+1
                    tr.miss = max(0, tr.miss - OBJ_OCCLU_MISS_RELIEF)

        # ---------- 阶段 B：其余轨迹做常规关联 ----------
        for ci, c in enumerate(clusters):
            if used[ci]:
                continue
            cx, cz = c["centroid_xz"]; xmin, xmax, ymin, ymax, zmin, zmax = c["bbox"]
            yext = ymax - ymin
            npts = len(c["idxs"])
            best = None; bestd = None
            for tr in tracks:
                if tr.last_frame() >= idx:
                    continue

                if tr.obj_confirmed:
                    gate = OBJ_ASSOC_GATE
                    if not small_object_with_margin(c["bbox"], npts):
                        continue
                else:
                    gate = assoc_gate
                    if tr.is_objectish() and (not is_small_object(c["bbox"], npts)):
                        continue

                px, pz = tr.last()
                d = math.hypot(cx - px, cz - pz)
                if d <= gate and (bestd is None or d < bestd):
                    bestd = d; best = tr
            if best is not None:
                used[ci] = True
                best.add(cx, cz, abs_t[idx], yext, bbox=c["bbox"], frame_idx=idx, npts=npts)

        # 新建轨迹
        for ci, c in enumerate(clusters):
            if used[ci]:
                continue
            cx, cz = c["centroid_xz"]; xmin, xmax, ymin, ymax, zmin, zmax = c["bbox"]
            npts = len(c["idxs"])
            tr = Track(cx, cz, abs_t[idx], ymax - ymin, bbox=c["bbox"], frame_idx=idx, npts=npts)
            tracks.append(tr)

        # 清理超时轨迹
        tracks = [tr for tr in tracks if tr.miss <= MAX_MISS]

        # 识别 + 锁存显示：先 Object 后 People
        ped_count = 0
        obj_count = 0
        for tr in tracks:
            # 先判 Object
            obj_show = tr.update_object_state(abs_t[idx])
            if obj_show and tr.last_bbox is not None:
                xmin, xmax, ymin, ymax, zmin, zmax = tr.last_bbox  # Object 用原始 bbox
                segs = make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax)
                box = gl.GLLinePlotItem(pos=segs, mode='lines', color=OBJ_BOX_COLOR, width=OBJ_BOX_WIDTH, antialias=True)
                view.addItem(box); box_items.append(box)

                if HAS_GLTEXT and LABEL_OBJECT:
                    cx3 = (xmin + xmax) / 2; cy3 = ymax + 0.1; cz3 = (zmin + zmax) / 2
                    txt = GLTextItem(pos=(cx3, cy3, cz3), text="Object",
                                     color=QtGui.QColor(0, 153, 255), font=QtGui.QFont("Microsoft YaHei", 14))
                    view.addItem(txt); text_items.append(txt)
                obj_count += 1
                continue  # 已作为 Object 展示，则不再按“行人”绘制

            # 再判行人
            show, v = tr.update_score_and_state(abs_t[idx])
            if not show:
                continue
            ped_count += 1
            xmin, xmax, ymin, ymax, zmin, zmax = tr.display_bbox()
            segs = make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax)
            box = gl.GLLinePlotItem(pos=segs, mode='lines', color=BOX_COLOR, width=BOX_WIDTH, antialias=True)
            view.addItem(box); box_items.append(box)

            if HAS_GLTEXT and LABEL_SPEED:
                cx3 = (xmin + xmax) / 2; cy3 = ymax + 0.1; cz3 = (zmin + zmax) / 2
                txt = GLTextItem(pos=(cx3, cy3, cz3), text=f"People {v:.2f} m/s",
                                 color=QtGui.QColor(0, 255, 0), font=QtGui.QFont("Microsoft YaHei", 14))
                view.addItem(txt); text_items.append(txt)

        scooter_states = scooter_plugin.process_frame(pts, abs_t[idx], frame_idx=idx)
        scooter_count = len(scooter_states)
        for st in scooter_states:
            segs = oriented_box_segments(st["center"], st["u_axis"], st["v_axis"],
                                         st["length"], st["width"], st["height"])
            color = SCOOTER_COLOR_DANGER if st["label"] == "ESCOOTER-DANGER" else SCOOTER_COLOR_NORMAL
            box = gl.GLLinePlotItem(pos=segs, mode='lines', color=color,
                                    width=SCOOTER_BOX_WIDTH, antialias=True)
            view.addItem(box); box_items.append(box)
            if HAS_GLTEXT:
                qcolor = SCOOTER_LABEL_COLOR.get(st["label"], QtGui.QColor(255, 170, 0))
                label = f"{st['label']} #{st['id']} {st['vp90']:.2f} m/s"
                txt = GLTextItem(pos=tuple(st["label_pos"].tolist()), text=label,
                                 color=qcolor, font=QtGui.QFont("Microsoft YaHei", 14))
                view.addItem(txt); text_items.append(txt)

        view.setWindowTitle(
            f'Radar 3D Pedestrian (robust): {rel_t[idx]:.3f} s  行人: {ped_count}  物体: {obj_count}  滑板车: {scooter_count}  gate={assoc_gate:.2f} m'
        )

    timer.timeout.connect(update)
    timer.start()
    QtWidgets.QApplication.instance().exec_()


if __name__ == '__main__':
    main()
