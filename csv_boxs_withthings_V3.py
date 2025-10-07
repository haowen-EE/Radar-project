import os
import sys

# ================= 运行时环境自动配置 =================
def _ensure_runtime():
    """自动切换至 .venv311 环境并设置 Qt 插件路径。"""
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    if os.name == 'nt':
        venv_python = os.path.join(project_root, '.venv311', 'Scripts', 'python.exe')
        plugin_dir = os.path.join(project_root, '.venv311', 'Lib', 'site-packages', 'PyQt5', 'Qt5', 'plugins')
    else:
        venv_python = os.path.join(project_root, '.venv311', 'bin', 'python')
        plugin_dir = os.path.join(project_root, '.venv311', 'lib', 
                                   f'python{sys.version_info.major}.{sys.version_info.minor}', 
                                   'site-packages', 'PyQt5', 'Qt5', 'plugins')
    
    already_in_venv = (not os.path.exists(venv_python) or 
                       os.path.abspath(sys.executable) == os.path.abspath(venv_python))
    
    if already_in_venv:
        if os.path.isdir(plugin_dir) and 'QT_QPA_PLATFORM_PLUGIN_PATH' not in os.environ:
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_dir
        return
    
    if os.path.isdir(plugin_dir):
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = plugin_dir
    
    os.execv(venv_python, [venv_python, os.path.abspath(__file__)] + sys.argv[1:])


def _ensure_qt_plugin_path():
    """兜底：确保 Qt 插件路径已设置。"""
    if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
        return
    
    candidates = []
    if os.name == 'nt':
        scripts_dir = os.path.dirname(sys.executable)
        candidates.append(os.path.abspath(os.path.join(scripts_dir, '..', 'Lib', 'site-packages', 
                                                        'PyQt5', 'Qt5', 'plugins')))
        project_root = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.abspath(os.path.join(project_root, '.venv311', 'Lib', 'site-packages', 
                                                        'PyQt5', 'Qt5', 'plugins')))
    else:
        lib_dir = os.path.abspath(os.path.join(os.path.dirname(sys.executable), '..', 'lib'))
        candidates.append(os.path.join(lib_dir, f'python{sys.version_info.major}.{sys.version_info.minor}', 
                                        'site-packages', 'PyQt5', 'Qt5', 'plugins'))
    
    for path_candidate in candidates:
        if os.path.isdir(path_candidate):
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = path_candidate
            break


# 执行环境配置（必须在 import PyQt5 之前）
_ensure_runtime()
_ensure_qt_plugin_path()

# ================= 正常导入 =================
import csv
import math
from collections import defaultdict, deque
from datetime import datetime
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl

from escooter_plugin import EscooterPlugin, oriented_box_segments


# ================= 配置 =================

# ====== 重要: 雷达安装高度 ======
# 雷达安装在 45cm 高的椅子上
# 所有点云Y坐标(高度)是相对于雷达位置的
# 实际物体离地高度 = 点云Y坐标 + 0.45m
# ================================

# ====== 数据文件配置(绝对路径) ======
# 直接填写CSV文件的完整绝对路径
# 推荐使用 r'...' 原始字符串避免转义问题
# 
# 示例:
# CSV_FILE = r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\unmove_scooter.csv'
# CSV_FILE = r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\escooter_noRSC.csv'
# CSV_FILE = r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\escooter2.csv'
# CSV_FILE = r'D:\雷达数据\my_test_data.csv'

CSV_FILE = r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\escooter1.csv'

print(f"[INFO] 正在加载数据文件: {CSV_FILE}")
# ====================================

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
# 注意: 行人速度范围 步行1.3-1.5m/s, 快走1.34-1.79m/s, 慢跑1.8-2.7m/s
# 设置为2.5m/s避免将电动滑板车(5m/s)误判为行人
WALK_SPEED_LO = 0.3
WALK_SPEED_HI = 2.5         # m/s: 行人速度上限(慢跑),避免误判滑板车
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


# ==========================================================================
# ScooterRider 滑板车+人 识别与跟踪
# ==========================================================================
# 雷达安装高度补偿
RADAR_HEIGHT_M = 0.45    # 雷达安装在45cm高的椅子上

# 基于实际测量的滑板车+人尺寸 (已考虑雷达高度补偿和点云扩散)
# 实际测量: 滑板车长1.17m×宽0.36m, 总高2.0m (踏板0.2m+人1.75m)
# 点云特性: 运动时会扩散,考虑30-50%余量
# 注意: 实际物体高度 = 点云Y坐标 + 雷达高度(0.45m)

# 水平尺寸 (考虑点云扩散30-50%)
SR_WIDTH_MIN = 0.25      # 宽度下限 (m): 0.36 × 0.7 = 0.25m
SR_WIDTH_MAX = 1.80      # 宽度上限 (m): 1.17 × 1.5 = 1.75m (取整1.8m)
SR_DEPTH_MIN = 0.25      # 深度下限 (m): 与宽度同
SR_DEPTH_MAX = 1.80      # 深度上限 (m): 与宽度同

# 垂直尺寸 - 根据escooter1/2实际数据调整
# 实际数据: 近距离高度中位数0.95m(１)和1.20m(２), 远距离0.5m左右
SR_HEIGHT_MIN = 0.30     # 高度下限 (m): 大幅降低以覆盖远距离/点云稀疏场景
SR_HEIGHT_MAX = 4.00     # 高度上限 (m): 扩大以容忍噪声点

# 点数 - 根据实际数据调整
# 近距离中位数:32-52点, 但远距离只有3-14点
SR_POINTS_MIN = 3        # 点数下限: 降至3以识别远距离目标
SR_POINTS_MAX = 300      # 点数上限: 略微增加

# 质心高度 - 根据escooter1/2实际数据调整
# 实际数据显示: escooter1质心Y中位数2.63m, escooter2为2.46m
# 范围: 0.2-5.7m, 大部分集中在1-4m
SR_CENTROID_Y_MIN = 0.20  # 质心下限 (m): 根据实际数据调整
SR_CENTROID_Y_MAX = 5.00  # 质心上限 (m): 根据实际数据调整
SR_SPEED_MIN = 1.5       # 速度下限 (m/s): 降低以覆盖慢速滑板车
SR_SPEED_MAX = 6.94      # 速度上限 (m/s) ~25 km/h
SR_SPEED_DANGER = 5.56   # 危险速度阈值 (m/s) ~20 km/h,超过此速度显示红色
SR_MIN_DURATION_S = 0.1  # 最小持续时间(进一步降低)
SR_CONFIRM_SCORE = 1     # 确认分数阈值(降低至1以更快确认)
SR_SCORE_HIT = 2
SR_SCORE_MISS = 1
SR_LATCH_S = 1.0         # 确认后保持显示时间


def is_scooter_rider(cluster, npts, strict=True, cy_hint=None):
    """
    判断一个聚类是否为滑板车+人的组合目标
    
    基于实际测量尺寸:
    - 滑板车: 长1.17m × 宽0.36m × 踏板高0.20m + 立柱高1.25m
    - 人体: 直径0.36m × 高1.75m
    - 总高: 2.0m (实际), 1.55m (相对雷达)
    
    参数:
    - strict: True=严格模式(新建轨迹时),False=宽松模式(检测已有轨迹)
    
    识别逻辑:
    1. 点数: 15-250 (考虑运动扩散)
    2. 水平尺寸: 0.25-1.8m (宽/深,考虑点云扩散)
    3. 垂直尺寸: 1.2-2.4m (高度,相对雷达)
    4. 质心高度: 0.40-1.10m (相对雷达,关键区分特征)
    5. 形状特征: 高度 > 水平尺寸 (较为高瘦)
    """
    # 宽松模式下点数范围更大
    pt_min = SR_POINTS_MIN if strict else max(8, SR_POINTS_MIN - 7)
    pt_max = SR_POINTS_MAX if strict else SR_POINTS_MAX + 50
    
    if npts < pt_min or npts > pt_max:
        return False
    
    bbox = cluster["bbox"]
    xmin, xmax, ymin, ymax, zmin, zmax = bbox
    w = xmax - xmin
    h = ymax - ymin
    d = zmax - zmin
    
    # 宽松模式下尺寸容差增加20%
    tolerance = 1.0 if strict else 1.2
    
    # 尺寸检查
    if not (SR_WIDTH_MIN / tolerance <= w <= SR_WIDTH_MAX * tolerance):
        return False
    if not (SR_HEIGHT_MIN * 0.8 <= h <= SR_HEIGHT_MAX * tolerance):  # 高度下限放宽更多
        return False
    if not (SR_DEPTH_MIN / tolerance <= d <= SR_DEPTH_MAX * tolerance):
        return False
    
    # 质心高度检查 (最关键特征,区分人、车、物体)
    pts = cluster.get("pts") if isinstance(cluster, dict) else None
    if pts is not None and len(pts) > 0:
        cy = float(np.asarray(pts)[:, 1].mean())
    elif cy_hint is not None:
        cy = float(cy_hint)
    else:
        cy = 0.5 * (ymin + ymax)
    cy_min = SR_CENTROID_Y_MIN * 0.7 if not strict else SR_CENTROID_Y_MIN
    cy_max = SR_CENTROID_Y_MAX * 1.3 if not strict else SR_CENTROID_Y_MAX
    if not (cy_min <= cy <= cy_max):
        return False
    
    # 特殊优化: 如果质心高度非常符合滑板车特征(0.6-0.9m),即使点数很少也认可
    # 这是典型滑板车+人的质心高度,优先级高于点数要求
    if 0.6 <= cy <= 0.9 and npts >= 5:
        return True  # 强烈滑板车特征,直接通过
    
    # 形状特征: 滑板车+人应该比较高瘦 (高度 > 平均水平尺寸)
    # 但当点数很少(<15)时,跳过高宽比检查,避免稀疏点云导致的误判
    avg_horizontal = (w + d) / 2.0
    if npts >= 15:  # 只在点数足够多时检查高宽比
        ratio_threshold = 0.6 if not strict else 0.8  # 宽松模式降低高宽比要求
        if avg_horizontal > 0 and h / avg_horizontal < ratio_threshold:
            return False
    # 点数<15时,主要依赖速度和质心高度判断,跳过形状检查
    
    return True


class ScooterRiderTrack:
    """
    滑板车+人统一目标的追踪类
    类似 Track 但使用不同的识别参数和速度范围
    """
    _next_sr = 1

    def __init__(self, cx, cz, t, bbox, frame_idx, npts):
        self.id = ScooterRiderTrack._next_sr
        ScooterRiderTrack._next_sr += 1
        self.c_smooth = np.array([cx, cz], float)
        self.centroids = deque(maxlen=ROLL_WIN)
        self.times = deque(maxlen=ROLL_WIN)
        self.frames = deque(maxlen=ROLL_WIN)
        self.miss = 0
        self.score = 0
        self.confirmed = False
        self.latch_until = 0.0
        self.last_bbox = bbox
        self.last_npts = int(npts)
        
        # 平滑的质心高度和盒子尺寸
        xmin, xmax, ymin, ymax, zmin, zmax = bbox
        self.cy_smooth = 0.5 * (ymin + ymax)
        self.size = np.array([xmax - xmin, ymax - ymin, zmax - zmin], float)
        
        self.add(cx, cz, t, bbox, frame_idx, npts)

    def add(self, cx, cz, t, bbox, frame_idx, npts):
        self.c_smooth = (1 - EWMA_ALPHA) * self.c_smooth + EWMA_ALPHA * np.array([cx, cz], float)
        self.centroids.append((cx, cz))
        self.times.append(float(t))
        self.frames.append(int(frame_idx))
        self.last_bbox = bbox
        self.last_npts = int(npts)
        
        # 更新质心高度和尺寸
        xmin, xmax, ymin, ymax, zmin, zmax = bbox
        cy = 0.5 * (ymin + ymax)
        self.cy_smooth = (1 - BOX_SMOOTH_ALPHA) * self.cy_smooth + BOX_SMOOTH_ALPHA * cy
        obs_size = np.array([xmax - xmin, ymax - ymin, zmax - zmin], float)
        self.size = (1 - BOX_SMOOTH_ALPHA) * self.size + BOX_SMOOTH_ALPHA * obs_size
        
        self.miss = 0

    def last(self):
        return tuple(self.c_smooth.tolist())

    def last_frame(self):
        return self.frames[-1] if len(self.frames) > 0 else -1

    def duration(self):
        if len(self.times) < 2:
            return 0.0
        return self.times[-1] - self.times[0]

    def speed_robust(self):
        """与 Track 相同的速度计算方法"""
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

        dx = np.diff(Cs[:, 0])
        dz = np.diff(Cs[:, 1])
        dt = np.diff(Ts)
        mask = dt > 0
        if not np.any(mask):
            return 0.0
        dx = dx[mask]
        dz = dz[mask]
        dt = dt[mask]

        disp = np.hypot(dx, dz)
        inst = disp / dt

        ok = (inst <= SPEED_SANITY_MAX) & (disp <= SPEED_SANITY_MAX * dt * DISP_FACTOR)
        good = inst[ok]
        if good.size == 0:
            return float(np.median(inst)) if inst.size else 0.0
        return float(np.median(good))

    def update_state(self, now):
        """更新滑板车+人的状态"""
        dur = self.duration()
        v = self.speed_robust()
        
        # 条件: 持续时间足够 + 速度在滑板车范围内
        cond = (dur >= SR_MIN_DURATION_S) and (SR_SPEED_MIN <= v <= SR_SPEED_MAX)
        
        self.score = min(self.score + SR_SCORE_HIT, 10) if cond else max(self.score - SR_SCORE_MISS, 0)
        
        if (not self.confirmed) and self.score >= SR_CONFIRM_SCORE:
            self.confirmed = True
            self.latch_until = now + SR_LATCH_S
        
        if self.confirmed and self.score > 0:
            self.latch_until = max(self.latch_until, now + 0.2)
        
        show = self.confirmed and (now <= self.latch_until)
        return show, v

    def display_bbox(self):
        """返回显示用的边界框"""
        w, h, d = self.size
        cx, cz = self.c_smooth
        cy = self.cy_smooth
        pad = 0.1
        
        xmin = (cx - 0.5 * w) - pad
        xmax = (cx + 0.5 * w) + pad
        ymin = (cy - 0.5 * h) - pad
        ymax = (cy + 0.5 * h) + pad
        zmin = (cz - 0.5 * d) - pad
        zmax = (cz + 0.5 * d) + pad
        
        return xmin, xmax, ymin, ymax, zmin, zmax


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
    scooter_rider_tracks = []  # 滑板车+人的追踪列表
    scooter_plugin = EscooterPlugin()

    timer = QtCore.QTimer()
    timer.setInterval(max(int(dt_med * 1000), 20))  # 兜底 20ms
    elapsed = QtCore.QElapsedTimer(); elapsed.start()
    idx = 0

    def clear_tracks():
        nonlocal tracks, scooter_rider_tracks
        tracks.clear()
        scooter_rider_tracks.clear()
        Track._next = 1
        ScooterRiderTrack._next_sr = 1
        scooter_plugin.reset()

    def update():
        nonlocal idx, box_items, text_items, tracks, scooter_rider_tracks
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
        for sr in scooter_rider_tracks:
            sr.miss += 1

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

        # ---------- 阶段 B1：ScooterRider 关联（优先级高于普通轨迹） ----------
        sr_gate = max(assoc_gate * 1.8, SR_SPEED_MAX * dt_med * 2.5)  # 更大的关联门限
        for sr in scooter_rider_tracks:
            best = None; bestd = None; best_ci = None
            px, pz = sr.last()
            for ci, c in enumerate(clusters):
                if used[ci]:
                    continue
                # 已确认的ScooterRider轨迹,放宽关联条件(不强制检查is_scooter_rider)
                # 只需要距离足够近即可,避免因点云变化导致跟踪丢失
                cx, cz = c["centroid_xz"]
                d = math.hypot(cx - px, cz - pz)
                if d <= sr_gate and (bestd is None or d < bestd):
                    bestd = d; best = c; best_ci = ci
            if best is not None:
                used[best_ci] = True
                cx, cz = best["centroid_xz"]
                npts = len(best["idxs"])
                sr.add(cx, cz, abs_t[idx], best["bbox"], idx, npts)

        # ---------- 阶段 B2：其余轨迹做常规关联 ----------
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
        
        # === 动态转换: Track → ScooterRider (多级速度阈值策略) ===
        # 检查已有的Track是否应该转为ScooterRider
        # 注意: CSV中的v是径向速度,不能用于判断!必须用轨迹计算的速度
        # 优化策略: 偏重滑板车识别,使用三级速度阈值覆盖所有速度分布
        tracks_to_convert = []
        for i, tr in enumerate(tracks):
            # 跳过已锁定为Object的轨迹
            if tr.lock_type == 'object':
                continue
            
            # 最少2帧数据即可判断(0.2s响应,vs原来3帧0.3s)
            if tr.last_bbox is not None and len(tr.centroids) >= 2:
                # 构造临时cluster用于检查
                temp_cluster = {"bbox": tr.last_bbox}
                v = tr.speed_robust()  # 从轨迹位置变化计算的真实速度
                
                # 获取质心高度(用于辅助判断) - 使用yc_smooth平滑后的质心Y坐标
                cy = tr.yc_smooth if tr.yc_smooth is not None else 0
                
                # === 第一级: 高速滑板车 (v>2.5m/s) ===
                # 接近真实滑板车速度,立即识别,优先级最高
                if is_scooter_rider(temp_cluster, tr.last_npts, strict=False, cy_hint=cy) and v > 2.5:
                    tracks_to_convert.append((i, tr, 'high_speed'))
                    continue
                
                # === 第二级: 中速疑似滑板车 (v>1.2m/s) ===
                # 考虑径向速度+横向运动,58.6%的escooter1点云v<1.8m/s
                # 通过几何特征辅助判断: 远距离(点数少)或质心较高
                elif is_scooter_rider(temp_cluster, tr.last_npts, strict=False, cy_hint=cy) and v > 1.2:
                    # 额外检查: 点数<30(远距离稀疏)或质心Y>1.5m(明显高于行人)
                    if tr.last_npts < 30 or cy > 1.5:
                        tracks_to_convert.append((i, tr, 'medium_speed'))
                        continue
                
                # === 第三级: 低速但几何特征强 (v>0.8m/s) ===
                # 可能是慢速滑板车或速度计算误差,使用严格几何检查+持续时间+质心高度
                elif is_scooter_rider(temp_cluster, tr.last_npts, strict=True, cy_hint=cy) and v > 0.8:
                    # 要求: 严格几何+至少5帧持续(0.5s)+质心较高(>1.0m,区分行人)
                    # 行人质心通常<1.0m,滑板车+人质心>1.0m
                    if len(tr.centroids) >= 5 and cy > 1.0:
                        tracks_to_convert.append((i, tr, 'low_speed'))
        
        # 执行转换
        for item in reversed(tracks_to_convert):  # 反向遍历避免索引问题
            i, old_tr = item[0], item[1]
            speed_level = item[2] if len(item) > 2 else 'unknown'
            # 创建新的ScooterRiderTrack,继承历史数据
            cx, cz = old_tr.last()
            sr = ScooterRiderTrack(cx, cz, abs_t[idx], old_tr.last_bbox, idx, old_tr.last_npts)
            # 继承部分历史(最近的数据)
            n_keep = min(10, len(old_tr.centroids))
            if n_keep > 0:
                for j in range(-n_keep, 0):
                    sr.centroids.append(old_tr.centroids[j])
                    sr.times.append(old_tr.times[j])
                    sr.frames.append(old_tr.frames[j])
            scooter_rider_tracks.append(sr)
            del tracks[i]
            v_actual = old_tr.speed_robust()
            print(f"[CONVERT] Track#{old_tr.id} → ScooterRider#{sr.id} (v={v_actual:.2f}m/s, level={speed_level}, npts={old_tr.last_npts})")

        # 新建轨迹(只创建普通Track,ScooterRider通过动态转换机制识别)
        # 原因:初始帧没有速度信息,无法区分行人和滑板车
        for ci, c in enumerate(clusters):
            if used[ci]:
                continue
            cx, cz = c["centroid_xz"]; xmin, xmax, ymin, ymax, zmin, zmax = c["bbox"]
            npts = len(c["idxs"])
            
            # 统一创建普通Track,等待动态转换机制判断是否为ScooterRider
            # 这样可以利用多帧速度信息进行更准确的判断
            tr = Track(cx, cz, abs_t[idx], ymax - ymin, bbox=c["bbox"], frame_idx=idx, npts=npts)
            tracks.append(tr)

        # 清理超时轨迹
        tracks = [tr for tr in tracks if tr.miss <= MAX_MISS]
        scooter_rider_tracks = [sr for sr in scooter_rider_tracks if sr.miss <= MAX_MISS]

        # 识别 + 锁存显示：ScooterRider → Object → People
        ped_count = 0
        obj_count = 0
        sr_count = 0
        
        # 1. 先显示 ScooterRider
        for sr in scooter_rider_tracks:
            show, v = sr.update_state(abs_t[idx])
            if show:
                sr_count += 1
                bbox = sr.display_bbox()
                xmin, xmax, ymin, ymax, zmin, zmax = bbox
                lines = make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax)
                
                # 根据速度选择颜色: 超过20km/h(5.56m/s)显示红色,否则紫色
                v_kmh = v * 3.6  # 转换为km/h
                if v >= SR_SPEED_DANGER:  # 危险速度
                    box_color = (1.0, 0.0, 0.0, 1.0)  # 红色
                    label_color = (255, 0, 0, 255)    # 红色
                    label = f"SR{sr.id} {v:.2f}m/s ({v_kmh:.1f}km/h) ⚠危险"
                else:  # 正常速度
                    box_color = (0.8, 0.2, 0.8, 1.0)  # 紫色
                    label_color = (200, 100, 200, 255)  # 紫色
                    label = f"SR{sr.id} {v:.2f}m/s ({v_kmh:.1f}km/h)"
                
                line_item = gl.GLLinePlotItem(pos=lines, color=box_color, width=2.5, mode='lines')
                view.addItem(line_item)
                box_items.append(line_item)
                cx, cz = sr.last()
                txt = gl.GLTextItem(pos=(cx, ymax + 0.3, cz), text=label, color=label_color)
                view.addItem(txt)
                text_items.append(txt)
        
        # 2. 显示 Object 和 People
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
            f'Radar 3D Pedestrian (robust): {rel_t[idx]:.3f} s  行人: {ped_count}  物体: {obj_count}  滑板车: {scooter_count}  滑板车+人: {sr_count}  gate={assoc_gate:.2f} m'
        )

    timer.timeout.connect(update)
    timer.start()
    QtWidgets.QApplication.instance().exec_()


if __name__ == '__main__':
    main()
