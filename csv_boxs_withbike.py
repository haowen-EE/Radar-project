# -*- coding: utf-8 -*-
"""
Radar 3D viewer with People / Object / E‑Scooter+Rider
- DBSCAN clustering (distance-adaptive)
- E‑Scooter distance-adaptive gates + cold-start fast confirm (+ speed & d_near relax at high speed)
- Predictive association + keep-alive
Design refs: escooter_rider_config.json / readme
"""

import argparse
import csv, math, os
from collections import defaultdict, deque
from datetime import datetime
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl

# ================= 基本配置 =================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, 'Data')


def _default_csv_file():
    candidates = []
    if os.path.isdir(DATA_DIR):
        for name in sorted(os.listdir(DATA_DIR)):
            if name.lower().endswith('.csv') and 'scooter' in name.lower():
                path = os.path.join(DATA_DIR, name)
                if os.path.isfile(path):
                    candidates.append(path)
    if candidates:
        return candidates[0]
    fallback = os.path.join(SCRIPT_DIR, 'escooter_fast_without_RSC.csv')
    return fallback if os.path.isfile(fallback) else None


CSV_FILE = _default_csv_file()


def list_scooter_csvs(data_dir=DATA_DIR):
    files = []
    if os.path.isdir(data_dir):
        for name in sorted(os.listdir(data_dir)):
            if name.lower().endswith('.csv') and 'scooter' in name.lower():
                path = os.path.join(data_dir, name)
                if os.path.isfile(path):
                    files.append(path)
    return files


def resolve_csv_path(csv_path=None, data_dir=DATA_DIR):
    if csv_path:
        candidate = csv_path
        if not os.path.isabs(candidate):
            candidate = os.path.join(SCRIPT_DIR, candidate)
            if not os.path.isfile(candidate):
                candidate = os.path.join(data_dir, csv_path)
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate
        raise FileNotFoundError(f'CSV 文件未找到: {csv_path}')

    default = _default_csv_file()
    if default is None:
        raise FileNotFoundError('未在 Data 目录或脚本根目录找到默认的 e-scooter CSV 文件，请使用 --csv 指定。')
    return default

ROTATE_Y_PLUS_90_FOR_X_ALIGNED_WALK = False

# ---- 聚类：默认使用 DBSCAN（更稳），可切回栅格 ----
USE_DBSCAN = True
DBSCAN_EPS_BASE = 0.50             # 近距基线 eps
DBSCAN_EPS_MIN  = 0.35
DBSCAN_EPS_MAX  = 0.70
DBSCAN_EPS_SLOPE_PER_M = 0.02      # 距离每 +1m，eps +0.02
DBSCAN_MIN_SAMPLES = 3             # RSC=ON：3 起
# 栅格聚类（备用）
GRID_CELL_M = 0.6
MIN_POINTS_IN_CLUSTER = 2          # RSC 下有时只打到 2~3 个点
ASSOC_GATE_BASE_M = 2.4
MAX_MISS = 8

# ---- 速度与稳定性 ----
ROLL_WIN = 40
EWMA_ALPHA = 0.35
SPEED_WIN_PAIR = 10
SPEED_SANITY_MAX = 9.0
DISP_FACTOR = 1.25

# ---- 行人判定（保持原始，不做抑制，确保能识别） ----
WALK_SPEED_LO = 0.3
WALK_SPEED_HI = 3.5
MIN_DURATION_S = 0.5
Y_EXTENT_MIN = 0.35
CONFIRM_SCORE = 3
SCORE_HIT = 2
SCORE_MISS = 1
LATCH_S = 1.0

# === Object（静小物体） ===
OBJ_SPEED_MAX = 0.20
OBJ_MIN_DURATION_S = 0.5
OBJ_MAX_POINTS = 15
OBJ_MAX_VOL = 0.50
OBJ_MAX_Y_EXTENT = 1.00
OBJ_ASSOC_GATE = 0.55
OBJ_CONFIRM_SCORE = 3
OBJ_SCORE_HIT = 2
OBJ_SCORE_MISS = 1
OBJ_LATCH_S = 1.2

# 近邻分裂 + 保护圈
OBJ_EXCLUDE_R = 0.50
OBJ_OCCLU_HOLD_S = 0.6
OBJ_OCCLU_MISS_RELIEF = 1

# 绘制
POINT_SIZE = 3
PT_COLOR = (1, 0, 0, 1)
BOX_COLOR = (0, 1, 0, 1)
BOX_WIDTH = 2
LABEL_SPEED = True
OBJ_BOX_COLOR = (0, 0.6, 1, 1)
OBJ_BOX_WIDTH = 2
LABEL_OBJECT = True

# === 传感器与滑板车几何信息（用于精准识别） ===
RADAR_LATERAL_OFFSET_M = 3.0               # 雷达距离道路中线 3m
RADAR_MOUNT_HEIGHT_RANGE_M = (0.50, 0.70)  # 雷达安装高度范围（地面以上）
RADAR_MOUNT_HEIGHT_M = float(np.mean(RADAR_MOUNT_HEIGHT_RANGE_M))

SCOOTER_DECK_LENGTH_M = 1.17
SCOOTER_STAND_LENGTH_M = 0.73
SCOOTER_DECK_WIDTH_M = 0.23
SCOOTER_DECK_THICKNESS_M = 0.20
SCOOTER_POLE_HEIGHT_M = 1.05
SCOOTER_TOTAL_HEIGHT_M = 1.25

SCOOTER_LENGTH_RANGE = (SCOOTER_DECK_LENGTH_M * 0.6,
                        SCOOTER_DECK_LENGTH_M * 1.4)
SCOOTER_WIDTH_RANGE = (SCOOTER_DECK_WIDTH_M * 0.7,
                       max(SCOOTER_DECK_WIDTH_M * 3.0, 0.8))
SCOOTER_HEIGHT_RANGE = (SCOOTER_TOTAL_HEIGHT_M * 0.7,
                        SCOOTER_TOTAL_HEIGHT_M * 1.35)
SCOOTER_BASE_HEIGHT_RANGE = (-0.2, 1.6)

# === E‑Scooter + Rider（核心） ===
ESCOOTER_CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'escooter_rider_config.json')

# 缺省门限（无 JSON 时使用；与文档一致）
ESCOOTER_SHAPE_GATE = dict(
    L_min=SCOOTER_LENGTH_RANGE[0],
    L_max=SCOOTER_LENGTH_RANGE[1],
    W_min=SCOOTER_WIDTH_RANGE[0],
    W_max=SCOOTER_WIDTH_RANGE[1],
    axis_ratio_min=1.6
)
ESCOOTER_DNEAR_RANGE = (0.16, 0.56)     # 典型
ESCOOTER_SPEED_MEAN_MIN = 1.5
ESCOOTER_VP90_MIN = 3.0
ESCOOTER_NPTS_RANGE_RSC = (2, 18)       # 放宽 RSC 下点数

# 距离自适应（>3m：W_max 放宽、L_min/AR_min 下调；设封顶/地板）
ESCOOTER_RANGE_REF_M   = 3.0
ESCOOTER_WMAX_SLOPE    = 0.08           # 每米 +0.08
ESCOOTER_LMIN_SLOPE    = 0.10           # 每米 −0.10
ESCOOTER_ARMIN_SLOPE   = 0.18           # 每米 −0.18
ESCOOTER_WMAX_ABS      = 1.00
ESCOOTER_LMIN_FLOOR    = 0.75
ESCOOTER_ARMIN_FLOOR   = 1.50

# 高速补偿（放宽 d_near）
ESCOOTER_FAST_SPEED        = 5.0
ESCOOTER_DNEAR_PAD_FAST_LO = 0.10
ESCOOTER_DNEAR_PAD_FAST_HI = 0.18

# 冷启动与快速确认（首 0.8s：形状+点数+速度≥4.0 即可先确认；无需 d_near）
SCOOTER_SPEED_SKIP_FRAMES      = 3
SCOOTER_FAST_CONFIRM_SPEED     = 4.0
SCOOTER_FAST_CONFIRM_MAX_AGE_S = 0.8

# 预测关联
PRED_GATE_BASE             = 0.4
PRED_GATE_EXTRA_PER_MS     = 0.0025
PRED_GATE_MAX_EXTRA        = 5.0

# E‑Scooter 显示 & 保活
ESCOOTER_KEEPALIVE_S       = 0.5
ESCOOTER_SCORE_HIT = 2
ESCOOTER_SCORE_MISS = 1
ESCOOTER_CONFIRM_SCORE = 3
ESCOOTER_LATCH_S = 1.0
ESCOOTER_COLOR = (1.0, 0.8, 0.2, 1.0)
ESCOOTER_BOX_WIDTH = 2
ESCOOTER_LABEL = True

SCOOTER_DANGEROUS_SPEED = 5.5

def load_escooter_cfg(path=ESCOOTER_CFG_PATH):
    import json
    cfg = dict(shape=ESCOOTER_SHAPE_GATE.copy(),
               dnear=ESCOOTER_DNEAR_RANGE,
               speed=(ESCOOTER_SPEED_MEAN_MIN, ESCOOTER_VP90_MIN),
               stand=dict(center_from_pole=0.365,
                          length=SCOOTER_STAND_LENGTH_M,
                          width=SCOOTER_DECK_WIDTH_M))
    try:
        with open(path, 'r', encoding='utf-8') as f:
            j = json.load(f)
        g = j.get('features_and_gates', {})
        s = g.get('obb_shape_gate', {})
        cfg['shape'].update(dict(
            L_min=s.get('L_min', cfg['shape']['L_min']),
            L_max=s.get('L_max', cfg['shape']['L_max']),
            W_min=s.get('W_min', cfg['shape']['W_min']),
            W_max=s.get('W_max', cfg['shape']['W_max']),
            axis_ratio_min=s.get('axis_ratio_min', cfg['shape']['axis_ratio_min'])
        ))
        d = g.get('human_position_gate', {})
        rng = d.get('d_near_range', list(cfg['dnear']))
        cfg['dnear'] = (float(rng[0]), float(rng[1]))
        sp = g.get('speed_gate', {})
        cfg['speed'] = (sp.get('mean_speed_min', cfg['speed'][0]),
                        sp.get('vp90_min', cfg['speed'][1]))
        geom = j.get('geometry', {})
        stand = geom.get('stand_zone', {})
        if stand:
            cfg['stand'] = dict(
                center_from_pole=stand.get('center_from_pole', cfg['stand']['center_from_pole']),
                length=stand.get('length', cfg['stand']['length']),
                width=stand.get('width', cfg['stand']['width'])
            )
    except Exception:
        pass
    return cfg

ESCOOTER_CFG = load_escooter_cfg()   # 取自你的 JSON/README（门限/几何/流程）  # ← 参考文档
# （参考：DBSCAN 参数、OBB 门限、d_near 解释与调参要点）  # see project docs

try:
    from pyqtgraph.opengl import GLTextItem
    HAS_GLTEXT = True
except Exception:
    HAS_GLTEXT = False

# === 方框稳定化 ===
BOX_SIZE_MODE = 'fixed'
BOX_FIXED_WHD = (0.6, 1.7, 0.6)
BOX_SMOOTH_ALPHA = 0.25
BOX_DELTA_CLAMP = 0.12
BOX_SIZE_MIN = (0.4, 1.4, 0.4)
BOX_SIZE_MAX = (0.9, 2.0, 0.9)
BOX_ANCHOR = 'center'
BOX_PAD = 0.02
BOX_SIZE_MIN_ARR = np.array(BOX_SIZE_MIN, float)
BOX_SIZE_MAX_ARR = np.array(BOX_SIZE_MAX, float)

# ================= 工具函数 =================

def transform_xyz(x, y, z):
    return (z, y, -x) if ROTATE_Y_PLUS_90_FOR_X_ALIGNED_WALK else (x, y, z)


RADAR_WORLD_OFFSET = np.array([RADAR_LATERAL_OFFSET_M,
                               RADAR_MOUNT_HEIGHT_M,
                               0.0], float)


def sensor_to_world(pt3):
    arr = np.array(pt3, float)
    return tuple((arr + RADAR_WORLD_OFFSET).tolist())


def centroid_sensor_to_world(cx, cz, bbox):
    if bbox is not None:
        ymin, ymax = bbox[2], bbox[3]
        cy = 0.5 * (ymin + ymax)
    else:
        cy = 0.0
    return sensor_to_world((cx, cy, cz))


def _parse_time(ts):
    if not ts: return None
    try:
        return datetime.fromisoformat(ts)
    except Exception:
        try:
            if ts.endswith('Z'):  # 兼容 ISOZ
                return datetime.fromisoformat(ts.replace('Z', '+00:00'))
        except Exception:
            return None
    return None

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
                ts = _parse_time(row.get('timeStamp'))
                if ts: time_map[fid] = ts
            try:
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                if fid >= 0:
                    data[fid].append(transform_xyz(x, y, z))
            except Exception:
                continue
    frames = sorted(data.keys())
    if not frames: raise RuntimeError('No frames parsed from CSV')

    # 帧间隔估计
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

# ---- DBSCAN（自实现，基于 XZ 平面） ----
def _dbscan_region(XZ, i, eps2):
    diff = XZ - XZ[i]
    d2 = (diff[:,0]**2 + diff[:,1]**2)
    return np.flatnonzero(d2 <= eps2)

def dbscan_cluster(pts, eps=0.5, min_samples=3):
    if len(pts) == 0:
        return []
    P = np.asarray(pts, float)
    XZ = P[:, [0, 2]]
    N = XZ.shape[0]
    visited = np.zeros(N, dtype=bool)
    labels = -np.zeros(N, dtype=int) - 1  # -1 未标注
    cid = 0
    eps2 = float(eps*eps)
    for i in range(N):
        if visited[i]: continue
        visited[i] = True
        neigh = _dbscan_region(XZ, i, eps2)
        if neigh.size < min_samples:
            continue
        labels[i] = cid
        seed = list(neigh.tolist())
        k = 0
        while k < len(seed):
            j = seed[k]; k += 1
            if not visited[j]:
                visited[j] = True
                neigh_j = _dbscan_region(XZ, j, eps2)
                if neigh_j.size >= min_samples:
                    seed.extend([int(t) for t in neigh_j.tolist() if t not in seed])
            if labels[j] == -1:
                labels[j] = cid
        cid += 1

    clusters = []
    for lab in range(cid):
        idxs = np.flatnonzero(labels == lab)
        if idxs.size < min_samples:
            continue
        cpts = P[idxs]
        xmin = float(cpts[:, 0].min()); xmax = float(cpts[:, 0].max())
        ymin = float(cpts[:, 1].min()); ymax = float(cpts[:, 1].max())
        zmin = float(cpts[:, 2].min()); zmax = float(cpts[:, 2].max())
        cxz = cpts[:, [0, 2]].mean(axis=0)
        clusters.append({
            "idxs": idxs.astype(int),
            "pts": cpts,
            "centroid_xz": cxz,
            "bbox": (xmin, xmax, ymin, ymax, zmin, zmax)
        })
    return clusters

# ---- 栅格聚类（备用） ----
def grid_cluster(pts, cell=GRID_CELL_M, min_points=MIN_POINTS_IN_CLUSTER):
    if len(pts) == 0:
        return []
    P = np.asarray(pts)
    x = P[:, 0]; z = P[:, 2]
    off = 1000.0 * cell
    ix = np.floor((x + off) / cell).astype(np.int64)
    iz = np.floor((z + off) / cell).astype(np.int64)
    cell_map = defaultdict(list)
    for i, (cx, cz) in enumerate(zip(ix, iz)):
        cell_map[(cx, cz)].append(i)

    vis, clusters = set(), []
    from collections import deque as dq
    for key in list(cell_map.keys()):
        if key in vis: continue
        q = dq([key]); vis.add(key); comp = []
        while q:
            c = q.popleft(); comp.append(c); cx, cz = c
            for dx in (-1,0,1):
                for dz in (-1,0,1):
                    if dx==0 and dz==0: continue
                    n = (cx+dx, cz+dz)
                    if n in cell_map and n not in vis:
                        vis.add(n); q.append(n)
        idxs = []
        for c in comp: idxs.extend(cell_map[c])
        if len(idxs) >= min_points:
            cpts = P[idxs]
            xmin = float(cpts[:, 0].min()); xmax = float(cpts[:, 0].max())
            ymin = float(cpts[:, 1].min()); ymax = float(cpts[:, 1].max())
            zmin = float(cpts[:, 2].min()); zmax = float(cpts[:, 2].max())
            cxz = cpts[:, [0, 2]].mean(axis=0)
            clusters.append({"idxs": np.array(idxs, int),
                             "pts": cpts,
                             "centroid_xz": cxz,
                             "bbox": (xmin, xmax, ymin, ymax, zmin, zmax)})
    return clusters

def cluster_frame(pts):
    if not USE_DBSCAN:
        return grid_cluster(pts)
    P = np.asarray(pts, float)
    if P.shape[0] == 0:
        return []
    # 按距离自适应 eps
    rng = np.hypot(P[:,0], P[:,2])
    r_med = float(np.median(rng))
    eps = DBSCAN_EPS_BASE + DBSCAN_EPS_SLOPE_PER_M * max(0.0, r_med - 3.0)
    eps = float(min(max(eps, DBSCAN_EPS_MIN), DBSCAN_EPS_MAX))
    return dbscan_cluster(P, eps=eps, min_samples=DBSCAN_MIN_SAMPLES)

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

def small_object_with_margin(bbox, npts, k_size=1.6, k_pts=1.8):
    w, h, d = bbox_dims(bbox)
    vol = w * h * d
    small_by_geom = (vol <= OBJ_MAX_VOL * k_size and h <= OBJ_MAX_Y_EXTENT * k_size)
    small_by_pts = (npts <= int(OBJ_MAX_POINTS * k_pts))
    return small_by_geom and small_by_pts

def split_cluster_near_object(cluster, obj_cx, obj_cz, radius=OBJ_EXCLUDE_R):
    pts = cluster["pts"]
    if pts.shape[0] < 6: return False, None, None
    xz = pts[:, [0, 2]]
    d = np.hypot(xz[:, 0] - obj_cx, xz[:, 1] - obj_cz)
    mask_in = d <= radius
    n_in = int(mask_in.sum()); n_out = int((~mask_in).sum())
    if n_in < 3 or n_out < 3: return False, None, None

    def mk(cpts, idxs):
        xmin = float(cpts[:, 0].min()); xmax = float(cpts[:, 0].max())
        ymin = float(cpts[:, 1].min()); ymax = float(cpts[:, 1].max())
        zmin = float(cpts[:, 2].min()); zmax = float(cpts[:, 2].max())
        cxz = cpts[:, [0, 2]].mean(axis=0)
        return {"idxs": idxs.copy(), "pts": cpts.copy(), "centroid_xz": cxz,
                "bbox": (xmin, xmax, ymin, ymax, zmin, zmax)}

    sub_in = mk(pts[mask_in], cluster["idxs"][mask_in])
    sub_out = mk(pts[~mask_in], cluster["idxs"][~mask_in])
    if not is_small_object(sub_in["bbox"], sub_in["pts"].shape[0]): return False, None, None
    return True, sub_in, sub_out

# === E‑Scooter：几何（XZ PCA‑OBB） ===
def pca_obb_xz(pts):
    if pts.shape[0] < 3: return None
    XZ = pts[:, [0, 2]].astype(float)
    ctr = XZ.mean(axis=0)
    X = XZ - ctr
    U, S, Vt = np.linalg.svd(X, full_matrices=False)
    u = Vt[0]; v = Vt[1]
    if np.linalg.det(np.vstack([u, v])) < 0: v = -v
    pu = X @ u; pv = X @ v
    L = float(pu.max() - pu.min())
    W = float(pv.max() - pv.min())
    u_min = float(pu.min()); u_max = float(pu.max())
    return dict(center=ctr, u=u, v=v, L=L, W=W, proj_u=pu, u_rng=(u_min, u_max))

def obb_corners_lines(center, u, v, L, W, y_min, y_max):
    cx, cz = center; ux, uz = u; vx, vz = v
    du = L * 0.5; dv = W * 0.5
    P = []
    for su in (-1, 1):
        for sv in (-1, 1):
            x = cx + su*du*ux + sv*dv*vx
            z = cz + su*du*uz + sv*dv*vz
            P.append([x, y_min, z])
    P += [[p[0], y_max, p[2]] for p in P[:4]]
    p0,p1,p2,p3,p4,p5,p6,p7 = P
    edges = [
        (p0,p1),(p1,p3),(p3,p2),(p2,p0),
        (p4,p5),(p5,p7),(p7,p6),(p6,p4),
        (p0,p4),(p1,p5),(p2,p6),(p3,p7)
    ]
    return np.vstack([np.vstack(e) for e in edges])

def dominant_peak(val, bins=None):
    if val.size < 3: return float(np.median(val))
    if bins is None: bins = max(8, int(np.sqrt(val.size)))
    hist, edges = np.histogram(val, bins=bins)
    k = int(np.argmax(hist))
    return float(0.5*(edges[k] + edges[k+1]))


def scooter_geometry_valid(feat, bbox):
    if feat is None:
        return False
    obb = feat['obb']
    L = float(obb['L'])
    W = float(obb['W'])
    y_min = float(feat['y_min'])
    y_max = float(feat['y_max'])
    height = max(0.0, y_max - y_min)

    length_ok = (SCOOTER_LENGTH_RANGE[0] <= L <= SCOOTER_LENGTH_RANGE[1])
    width_ok = (SCOOTER_WIDTH_RANGE[0] <= W <= SCOOTER_WIDTH_RANGE[1])
    height_ok = (SCOOTER_HEIGHT_RANGE[0] <= height <= SCOOTER_HEIGHT_RANGE[1])

    base_world_height = RADAR_MOUNT_HEIGHT_M + y_min
    base_ok = (SCOOTER_BASE_HEIGHT_RANGE[0] <= base_world_height <= SCOOTER_BASE_HEIGHT_RANGE[1])

    return length_ok and width_ok and height_ok and base_ok


def adapt_scooter_shape_gate_by_range(base_shape, r):
    dr = max(0.0, r - ESCOOTER_RANGE_REF_M)
    W_max = min(base_shape['W_max'] + ESCOOTER_WMAX_SLOPE*dr, ESCOOTER_WMAX_ABS)
    L_min = max(ESCOOTER_LMIN_FLOOR, base_shape['L_min'] - ESCOOTER_LMIN_SLOPE*dr)
    AR_min = max(ESCOOTER_ARMIN_FLOOR, base_shape['axis_ratio_min'] - ESCOOTER_ARMIN_SLOPE*dr)
    return dict(L_min=L_min, L_max=base_shape['L_max'], W_min=base_shape['W_min'],
                W_max=W_max, axis_ratio_min=AR_min)

# ================= 轨迹类 =================

class Track:
    _next = 1

    def __init__(self, cx, cz, t, yext, bbox=None, frame_idx=0, npts=0, pts=None):
        self.id = Track._next; Track._next += 1
        self.c_smooth = np.array([cx, cz], float)
        self.centroids = deque(maxlen=ROLL_WIN)
        self.times = deque(maxlen=ROLL_WIN)
        self.frames = deque(maxlen=ROLL_WIN)
        self.y_exts = deque(maxlen=ROLL_WIN)
        self.miss = 0
        self.score = 0
        self.confirmed = False
        self.latch_until = 0.0
        self.last_bbox = bbox
        self.yc_smooth = None
        self.y_base_smooth = None
        self.size = None

        # Object
        self.last_npts = int(npts) if npts else 0
        self.obj_score = 0
        self.obj_confirmed = False
        self.obj_latch_until = 0.0

        # 速度样本（vp90）
        self.speeds = deque(maxlen=ROLL_WIN)

        # E‑Scooter
        self.last_pts = pts
        self.escooter_score = 0
        self.escooter_confirmed = False
        self.escooter_latch_until = 0.0

        # 类别锁
        self.lock_type = None   # None | 'object' | 'escooter'

        self.add(cx, cz, t, yext, bbox=bbox, frame_idx=frame_idx, npts=npts, pts=pts)

    def add(self, cx, cz, t, yext, bbox=None, frame_idx=None, npts=None, pts=None):
        prev = np.array(self.centroids[-1]) if len(self.centroids)>0 else None
        prev_t = float(self.times[-1]) if len(self.times)>0 else None

        self.c_smooth = (1 - EWMA_ALPHA) * self.c_smooth + EWMA_ALPHA * np.array([cx, cz], float)
        self.centroids.append((cx, cz))
        self.times.append(float(t))
        if frame_idx is not None: self.frames.append(int(frame_idx))
        self.y_exts.append(float(yext))

        # 速度样本
        if (prev is not None) and (prev_t is not None) and (t > prev_t):
            dt = float(t - prev_t)
            v = float(np.hypot(cx - prev[0], cz - prev[1]) / max(1e-6, dt))
            if np.isfinite(v): self.speeds.append(v)

        if bbox is not None:
            self.last_bbox = bbox
            xmin, xmax, ymin, ymax, zmin, zmax = bbox
            yc = 0.5 * (ymin + ymax)
            self.yc_smooth = yc if self.yc_smooth is None else (1 - BOX_SMOOTH_ALPHA) * self.yc_smooth + BOX_SMOOTH_ALPHA * yc
            self.y_base_smooth = ymin if self.y_base_smooth is None else (1 - BOX_SMOOTH_ALPHA) * self.y_base_smooth + BOX_SMOOTH_ALPHA * ymin
            obs_size = np.array([xmax - xmin, ymax - ymin, zmax - zmin], float)
            self._update_size(obs_size)
        if npts is not None: self.last_npts = int(npts)
        if pts is not None and isinstance(pts, np.ndarray) and pts.shape[1] >= 3:
            self.last_pts = pts.copy()

        self.miss = 0

    def last(self): return tuple(self.c_smooth.tolist())
    def last_frame(self): return self.frames[-1] if len(self.frames) > 0 else -1
    def duration(self):
        if len(self.times) < 2: return 0.0
        return self.times[-1] - self.times[0]
    def y_med(self): return float(np.median(self.y_exts)) if self.y_exts else 0.0

    def speed_robust(self):
        n = len(self.times)
        if n < 2: return 0.0
        k = min(n - 1, SPEED_WIN_PAIR)
        if k <= 0: return 0.0
        Cs = np.array(list(self.centroids)[-k - 1:], float)
        for i in range(1, Cs.shape[0]): Cs[i] = (1 - EWMA_ALPHA) * Cs[i - 1] + EWMA_ALPHA * Cs[i]
        Ts = np.array(list(self.times)[-k - 1:], float)
        dx = np.diff(Cs[:, 0]); dz = np.diff(Cs[:, 1]); dt = np.diff(Ts)
        mask = dt > 0
        if not np.any(mask): return 0.0
        dx = dx[mask]; dz = dz[mask]; dt = dt[mask]
        disp = np.hypot(dx, dz); inst = disp / dt
        ok = (inst <= SPEED_SANITY_MAX) & (disp <= SPEED_SANITY_MAX * dt * DISP_FACTOR)
        good = inst[ok]
        if good.size == 0: return float(np.median(inst)) if inst.size else 0.0
        return float(np.median(good))

    def vp90(self):
        if len(self.speeds) < 5: return None
        return float(np.percentile(np.array(self.speeds), 90.0))

    # 常速度外推（关联预测）
    def predict_pos(self, t_now):
        if len(self.centroids) < 2: return self.c_smooth[0], self.c_smooth[1]
        (x2, z2) = self.centroids[-1]; (x1, z1) = self.centroids[-2]
        t2 = float(self.times[-1]); t1 = float(self.times[-2])
        dt = max(1e-6, t2 - t1)
        vx = (x2 - x1) / dt; vz = (z2 - z1) / dt
        dt_pred = float(t_now - t2)
        return x2 + vx * dt_pred, z2 + vz * dt_pred

    # 行人判定：恢复“原生逻辑”，不再做形状/速度抑制，确保行人能出
    def update_score_and_state(self, now):
        if self.lock_type in ('object', 'escooter'):
            return False, self.speed_robust()
        dur = self.duration(); v = self.speed_robust(); ymed = self.y_med()
        ok = (dur >= MIN_DURATION_S) and (WALK_SPEED_LO <= v <= WALK_SPEED_HI) and (ymed >= Y_EXTENT_MIN)
        self.score = min(self.score + SCORE_HIT, 10) if ok else max(self.score - SCORE_MISS, 0)
        if (not self.confirmed) and self.score >= CONFIRM_SCORE:
            self.confirmed = True; self.latch_until = now + LATCH_S
        if self.confirmed and self.score > 0:
            self.latch_until = max(self.latch_until, now + 0.2)
        show = self.confirmed and (now <= self.latch_until)
        return show, v

    # 显示方框
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
            up = prev * (1.0 + BOX_DELTA_CLAMP); dn = prev * (1.0 - BOX_DELTA_CLAMP)
            target = np.minimum(np.maximum(target, dn), up)
            self.size = (1.0 - BOX_SMOOTH_ALPHA) * prev + BOX_SMOOTH_ALPHA * target

    def display_bbox(self):
        if self.size is None: self.size = np.array(BOX_FIXED_WHD, float)
        w, h, d = self.size; cx, cz = self.c_smooth; pad = float(BOX_PAD)
        yc = self.yc_smooth if self.yc_smooth is not None else 0.5 * h
        ymin = (yc - 0.5 * h) - pad; ymax = (yc + 0.5 * h) + pad
        xmin = (cx - 0.5 * w) - pad; xmax = (cx + 0.5 * w) + pad
        zmin = (cz - 0.5 * d) - pad; zmax = (cz + 0.5 * d) + pad
        return xmin, xmax, ymin, ymax, zmin, zmax

    # === Object ===
    def small_like(self):
        if self.last_bbox is None: return False
        return is_small_object(self.last_bbox, self.last_npts)

    def is_objectish(self):
        v = self.speed_robust(); short_hist = (len(self.times) < 6)
        return (self.small_like() and (v <= OBJ_SPEED_MAX * 1.5 or short_hist))

    def update_object_state(self, now):
        dur = self.duration(); v = self.speed_robust()
        cond = (dur >= OBJ_MIN_DURATION_S) and (v <= OBJ_SPEED_MAX) and self.small_like()
        self.obj_score = min(self.obj_score + OBJ_SCORE_HIT, 10) if cond else max(self.obj_score - OBJ_SCORE_MISS, 0)
        if (not self.obj_confirmed) and self.obj_score >= OBJ_CONFIRM_SCORE:
            self.obj_confirmed = True; self.obj_latch_until = now + OBJ_LATCH_S; self.lock_type = 'object'
        if self.obj_confirmed and self.obj_score > 0:
            self.obj_latch_until = max(self.obj_latch_until, now + 0.2)
        return self.obj_confirmed and (now <= self.obj_latch_until)

    # === E‑Scooter 特征/判定 ===
    def escooter_features(self):
        if self.last_pts is None or self.last_bbox is None: return None
        obb = pca_obb_xz(self.last_pts)
        if obb is None: return None
        xmin,xmax,ymin,ymax,zmin,zmax = self.last_bbox
        u_peak = dominant_peak(obb['proj_u'])
        u_min, u_max = obb['u_rng']
        d_near = float(min(abs(u_peak - u_min), abs(u_max - u_peak)))
        return dict(obb=obb, y_min=ymin, y_max=ymax, u_peak=u_peak,
                    d_near=d_near, npts=int(self.last_npts))

    def update_escooter_state(self, now, cfg):
        feat = self.escooter_features()
        if feat is None:
            self.escooter_score = max(self.escooter_score - ESCOOTER_SCORE_MISS, 0)
            return False, None

        geom_ok = scooter_geometry_valid(feat, self.last_bbox)
        feat['geometry_ok'] = geom_ok

        # 距离自适应形状门限
        r = float(np.hypot(self.c_smooth[0], self.c_smooth[1]))
        sh = adapt_scooter_shape_gate_by_range(cfg['shape'], r)

        L,W = feat['obb']['L'], feat['obb']['W']
        AR = float(max(L,W)/max(1e-6, min(L,W)))
        dnear_lo, dnear_hi = cfg['dnear']

        v_mean = self.speed_robust()
        if v_mean >= ESCOOTER_FAST_SPEED:  # 高速补偿：放宽 d_near
            dnear_lo = max(0.0, dnear_lo - ESCOOTER_DNEAR_PAD_FAST_LO)
            dnear_hi =         dnear_hi + ESCOOTER_DNEAR_PAD_FAST_HI

        # 形状/点数门限
        shape_ok = (sh['L_min'] <= L <= sh['L_max']) and (sh['W_min'] <= W <= sh['W_max']) and (AR >= sh['axis_ratio_min'])
        n_ok     = (ESCOOTER_NPTS_RANGE_RSC[0] <= feat['npts'] <= ESCOOTER_NPTS_RANGE_RSC[1])
        # 人位置信号
        near_ok  = (dnear_lo <= feat['d_near'] <= dnear_hi)

        # 冷启动速度豁免
        samples = len(self.speeds)
        if samples < SCOOTER_SPEED_SKIP_FRAMES:
            sp_ok = True
        else:
            v90 = self.vp90()
            sp_ok = (v_mean >= ESCOOTER_CFG['speed'][0]) and (True if v90 is None else (v90 >= ESCOOTER_CFG['speed'][1]))

        ok = shape_ok and n_ok and near_ok and sp_ok and geom_ok

        # —— 首 0.8 s 快速确认：形状+点数+速度≥4.0 即先确认（无需 d_near）——
        if (not self.escooter_confirmed) and (self.duration() <= SCOOTER_FAST_CONFIRM_MAX_AGE_S) \
           and shape_ok and n_ok and geom_ok and (v_mean >= SCOOTER_FAST_CONFIRM_SPEED):
            self.escooter_confirmed = True
            self.lock_type = 'escooter'
            self.escooter_score = max(self.escooter_score, ESCOOTER_CONFIRM_SCORE)
            self.escooter_latch_until = now + ESCOOTER_LATCH_S

        # 常规积分/衰减
        self.escooter_score = min(self.escooter_score + ESCOOTER_SCORE_HIT, 10) if ok \
                              else max(self.escooter_score - ESCOOTER_SCORE_MISS, 0)

        if (not self.escooter_confirmed) and self.escooter_score >= ESCOOTER_CONFIRM_SCORE:
            self.escooter_confirmed = True
            self.escooter_latch_until = now + ESCOOTER_LATCH_S
            self.lock_type = 'escooter'

        # 保活（防“只闪一帧”）
        if self.escooter_confirmed:
            self.escooter_latch_until = max(self.escooter_latch_until, now + ESCOOTER_KEEPALIVE_S)

        show = self.escooter_confirmed and (now <= self.escooter_latch_until)
        return (show, feat) if show else (False, feat)

# ================= 主循环 =================


def tracking_step(tracks, clusters, abs_time, frame_idx, assoc_gate):
    track_list = tracks
    for tr in track_list:
        tr.miss += 1

    if clusters and any(tr.obj_confirmed for tr in track_list):
        new_clusters = []
        used_flags = [False] * len(clusters)
        for tr in track_list:
            if not tr.obj_confirmed:
                continue
            ocx, ocz = tr.last()
            for ci, c in enumerate(clusters):
                if used_flags[ci]:
                    continue
                if is_small_object(c["bbox"], c["pts"].shape[0]):
                    continue
                cx, cz = c["centroid_xz"]
                if math.hypot(cx - ocx, cz - ocz) > max(OBJ_EXCLUDE_R * 1.2, OBJ_ASSOC_GATE * 1.5):
                    continue
                ok, sub_in, sub_out = split_cluster_near_object(c, ocx, ocz, radius=OBJ_EXCLUDE_R)
                if ok:
                    if sub_in is not None:
                        new_clusters.append(sub_in)
                    if sub_out is not None:
                        new_clusters.append(sub_out)
                    used_flags[ci] = True
        for ci, c in enumerate(clusters):
            if not used_flags[ci]:
                new_clusters.append(c)
        clusters = new_clusters

    used = [False] * len(clusters)

    obj_tracks = [tr for tr in track_list if tr.obj_confirmed]
    for tr in obj_tracks:
        best = None
        bestd = None
        best_ci = None
        px_pred, pz_pred = tr.predict_pos(abs_time)
        for ci, c in enumerate(clusters):
            if used[ci]:
                continue
            npts = len(c["idxs"])
            if not small_object_with_margin(c["bbox"], npts):
                continue
            cx, cz = c["centroid_xz"]
            dt_gap_ms = max(0.0, (abs_time - tr.times[-1])) * 1000.0
            gate_extra = min(PRED_GATE_MAX_EXTRA, PRED_GATE_BASE + PRED_GATE_EXTRA_PER_MS * dt_gap_ms)
            d = math.hypot(cx - px_pred, cz - pz_pred)
            if d <= (OBJ_ASSOC_GATE + gate_extra) and (bestd is None or d < bestd):
                bestd = d
                best = c
                best_ci = ci
        if best is not None:
            used[best_ci] = True
            cx, cz = best["centroid_xz"]
            xmin, xmax, ymin, ymax, zmin, zmax = best["bbox"]
            yext = ymax - ymin
            npts = len(best["idxs"])
            tr.add(cx, cz, abs_time, yext, bbox=best["bbox"], frame_idx=frame_idx, npts=npts, pts=best["pts"])
        else:
            tr.obj_latch_until = max(tr.obj_latch_until, abs_time + 0.2)

    for ci, c in enumerate(clusters):
        if used[ci]:
            continue
        cx, cz = c["centroid_xz"]
        xmin, xmax, ymin, ymax, zmin, zmax = c["bbox"]
        yext = ymax - ymin
        npts = len(c["idxs"])
        best = None
        bestd = None
        for tr in track_list:
            if tr.last_frame() >= frame_idx:
                continue
            if tr.obj_confirmed:
                gate = OBJ_ASSOC_GATE
                if not small_object_with_margin(c["bbox"], npts):
                    continue
            else:
                gate = assoc_gate
                if tr.is_objectish() and (not is_small_object(c["bbox"], npts)):
                    continue
            px_pred, pz_pred = tr.predict_pos(abs_time)
            dt_gap_ms = max(0.0, (abs_time - tr.times[-1])) * 1000.0
            gate_extra = min(PRED_GATE_MAX_EXTRA, PRED_GATE_BASE + PRED_GATE_EXTRA_PER_MS * dt_gap_ms)
            d = math.hypot(cx - px_pred, cz - pz_pred)
            if d <= (gate + gate_extra) and (bestd is None or d < bestd):
                bestd = d
                best = tr
        if best is not None:
            used[ci] = True
            best.add(cx, cz, abs_time, yext, bbox=c["bbox"], frame_idx=frame_idx, npts=npts, pts=c["pts"])

    for ci, c in enumerate(clusters):
        if used[ci]:
            continue
        cx, cz = c["centroid_xz"]
        xmin, xmax, ymin, ymax, zmin, zmax = c["bbox"]
        npts = len(c["idxs"])
        track_list.append(Track(cx, cz, abs_time, ymax - ymin, bbox=c["bbox"], frame_idx=frame_idx, npts=npts, pts=c["pts"]))

    track_list = [tr for tr in track_list if tr.miss <= MAX_MISS]

    results = {
        "objects": [],
        "escooters": [],
        "people": [],
        "counts": {"obj": 0, "escooter": 0, "ped": 0},
    }

    for tr in track_list:
        if tr.update_object_state(abs_time) and tr.last_bbox is not None:
            results["objects"].append({
                "track_id": tr.id,
                "bbox": tr.last_bbox,
                "speed": tr.speed_robust(),
            })
            results["counts"]["obj"] += 1
            continue

        show_sc, feat = tr.update_escooter_state(abs_time, ESCOOTER_CFG)
        if show_sc and feat is not None:
            speed_val = tr.speed_robust()
            state = 'dangerous' if speed_val > SCOOTER_DANGEROUS_SPEED else 'normal'
            cx, cz = tr.last()
            world_ctr = centroid_sensor_to_world(cx, cz, tr.last_bbox)
            results["escooters"].append({
                "track_id": tr.id,
                "centroid": (cx, cz),
                "world_centroid": world_ctr,
                "speed": speed_val,
                "state": state,
                "bbox": tr.last_bbox,
                "features": feat,
                "geometry_ok": feat.get('geometry_ok', False),
            })
            results["counts"]["escooter"] += 1
            continue

        show, v = tr.update_score_and_state(abs_time)
        if show:
            results["people"].append({
                "track_id": tr.id,
                "bbox": tr.display_bbox(),
                "speed": v,
            })
            results["counts"]["ped"] += 1

    return track_list, results


def summarize_escooter_tracks(detections):
    if not detections:
        return []

    by_track = {}
    for det in detections:
        tid = det["track_id"]
        info = by_track.setdefault(tid, {
            "frames": [],
            "rel_times": [],
            "abs_times": [],
            "speeds": [],
            "positions": [],
            "world_positions": [],
            "states": [],
            "geometry_flags": [],
        })
        info["frames"].append(det["frame_index"])
        info["rel_times"].append(det["rel_time"])
        info["abs_times"].append(det["abs_time"])
        info["speeds"].append(det["speed"])
        info["positions"].append(det["centroid"])
        info["world_positions"].append(det.get("world_centroid"))
        info["states"].append(det.get("state", "unknown"))
        info["geometry_flags"].append(bool(det.get("geometry_ok")))

    summary = []
    for tid, info in by_track.items():
        frames = info["frames"]
        rel_times = info["rel_times"]
        abs_times = info["abs_times"]
        speeds = info["speeds"]
        pos = info["positions"]

        path_len = 0.0
        for (x1, z1), (x2, z2) in zip(pos, pos[1:]):
            path_len += math.hypot(x2 - x1, z2 - z1)

        summary.append({
            "track_id": tid,
            "start_frame": frames[0],
            "end_frame": frames[-1],
            "num_detections": len(frames),
            "rel_start": rel_times[0],
            "rel_end": rel_times[-1],
            "abs_start": abs_times[0],
            "abs_end": abs_times[-1],
            "duration": rel_times[-1] - rel_times[0] if len(rel_times) > 1 else 0.0,
            "median_speed": float(np.median(speeds)) if speeds else 0.0,
            "max_speed": float(np.max(speeds)) if speeds else 0.0,
            "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
            "path_length": path_len,
            "start_position": pos[0],
            "end_position": pos[-1],
            "world_start": info["world_positions"][0] if info["world_positions"] else None,
            "world_end": info["world_positions"][-1] if info["world_positions"] else None,
            "state": 'dangerous' if any(s == 'dangerous' for s in info["states"]) else 'normal',
            "geometry_consistent": all(info["geometry_flags"]) if info["geometry_flags"] else False,
        })

    summary.sort(key=lambda item: item["rel_start"])
    return summary


def analyze_scooter_csv(csv_file, verbose=True):
    data, frames, rel_t, abs_t, dt_med = load_frames(csv_file)
    assoc_gate = max(ASSOC_GATE_BASE_M, SPEED_SANITY_MAX * dt_med * 2.0)
    tracks = []
    Track._next = 1
    detections = []

    for idx, fid in enumerate(frames):
        pts = np.array(data[fid], float) if data[fid] else np.zeros((0, 3))
        clusters = cluster_frame(pts)
        tracks, results = tracking_step(tracks, clusters, abs_t[idx], idx, assoc_gate)
        for es in results["escooters"]:
            detections.append({
                "track_id": es["track_id"],
                "frame": fid,
                "frame_index": idx,
                "rel_time": rel_t[idx],
                "abs_time": abs_t[idx],
                "speed": es["speed"],
                "state": es["state"],
                "centroid": es["centroid"],
                "world_centroid": es["world_centroid"],
                "bbox": es["bbox"],
                "features": es["features"],
                "geometry_ok": es.get("geometry_ok", False),
            })

    summary = summarize_escooter_tracks(detections)
    if verbose:
        name = os.path.basename(csv_file)
        if not summary:
            print(f"{name}: 未检测到 e-scooter 行驶轨迹。")
        else:
            print(f"{name}: 检测到 {len(summary)} 段 e-scooter 行驶。")
            for item in summary:
                state_text = '危险驾驶' if item['state'] == 'dangerous' else '正常行驶'
                ws = item.get('world_start')
                we = item.get('world_end')
                ws_txt = f"起点({ws[0]:.2f},{ws[1]:.2f},{ws[2]:.2f})" if ws else "起点未知"
                we_txt = f"终点({we[0]:.2f},{we[1]:.2f},{we[2]:.2f})" if we else "终点未知"
                geom_txt = "几何匹配" if item.get('geometry_consistent') else "几何需复核"
                print(
                    f"  轨迹 #{item['track_id']:02d}: {item['rel_start']:.2f}s → {item['rel_end']:.2f}s "
                    f"(持续 {item['duration']:.2f}s, 样本 {item['num_detections']})，速度中位 {item['median_speed']:.2f} m/s，"
                    f"距离约 {item['path_length']:.2f} m，{state_text}，{geom_txt}，{ws_txt}，{we_txt}"
                )

    return summary, detections

def run_viewer(csv_file):
    csv_file = os.path.abspath(csv_file)
    data, frames, rel_t, abs_t, dt_med = load_frames(csv_file)
    total_time = rel_t[-1] if len(rel_t) else 0.0
    assoc_gate = max(ASSOC_GATE_BASE_M, SPEED_SANITY_MAX * dt_med * 2.0)

    app = QtWidgets.QApplication([])
    view = gl.GLViewWidget()
    view.opts['distance'] = 20
    view.setCameraPosition(azimuth=45, elevation=20, distance=20)
    view.setWindowTitle(f'Radar 3D (Scooter+Rider & People): 0.000 s [{os.path.basename(csv_file)}]')
    view.show()
    axis = gl.GLAxisItem(); axis.setSize(x=10, y=10, z=10); view.addItem(axis)
    grid = gl.GLGridItem(); grid.setSize(10, 10); grid.setSpacing(1, 1); view.addItem(grid)
    scatter = gl.GLScatterPlotItem(size=POINT_SIZE, color=PT_COLOR); view.addItem(scatter)

    box_items, text_items = [], []
    tracks = []
    Track._next = 1

    timer = QtCore.QTimer()
    timer.setInterval(max(int(dt_med * 1000), 20))
    elapsed = QtCore.QElapsedTimer(); elapsed.start()
    idx = 0

    def clear_tracks():
        nonlocal tracks
        tracks.clear()
        Track._next = 1

    def update():
        nonlocal idx, box_items, text_items, tracks
        sec = elapsed.elapsed() / 1000.0
        if total_time > 0 and sec > total_time:
            elapsed.restart(); idx = 0; sec = 0.0; clear_tracks()

        while idx < len(frames) - 1 and sec >= rel_t[idx + 1]:
            idx += 1

        pts = np.array(data[frames[idx]], float) if data[frames[idx]] else np.zeros((0, 3))
        scatter.setData(pos=pts)

        for it in box_items: view.removeItem(it)
        box_items = []
        for it in text_items: view.removeItem(it)
        text_items = []

        clusters = cluster_frame(pts)
        tracks, results = tracking_step(tracks, clusters, abs_t[idx], idx, assoc_gate)

        counts = results["counts"]
        danger_cnt = sum(1 for es in results["escooters"] if es.get("state") == 'dangerous')

        for obj in results["objects"]:
            bbox = obj["bbox"]
            xmin, xmax, ymin, ymax, zmin, zmax = bbox
            segs = make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax)
            box = gl.GLLinePlotItem(pos=segs, mode='lines', color=OBJ_BOX_COLOR, width=OBJ_BOX_WIDTH, antialias=True)
            view.addItem(box); box_items.append(box)
            if HAS_GLTEXT and LABEL_OBJECT:
                cx3 = (xmin + xmax)/2; cy3 = ymax + 0.1; cz3 = (zmin + zmax)/2
                txt = GLTextItem(pos=(cx3, cy3, cz3), text="Object",
                                 color=QtGui.QColor(0,153,255), font=QtGui.QFont("Microsoft YaHei", 14))
                view.addItem(txt); text_items.append(txt)

        for es in results["escooters"]:
            feat = es["features"]
            obb = feat['obb']
            is_danger = es.get("state") == 'dangerous'
            box_color = (1.0, 0.2, 0.2, 1.0) if is_danger else ESCOOTER_COLOR
            segs = obb_corners_lines(obb['center'], obb['u'], obb['v'], obb['L'], obb['W'],
                                     feat['y_min'], feat['y_max'])
            box = gl.GLLinePlotItem(pos=segs, mode='lines', color=box_color,
                                    width=ESCOOTER_BOX_WIDTH, antialias=True)
            view.addItem(box); box_items.append(box)

            u = obb['u']; v = obb['v']; center = obb['center']; L = obb['L']
            u_min, u_max = obb['u_rng']; u_peak = feat['u_peak']
            near_is_min = (abs(u_peak - u_min) <= abs(u_max - u_peak))
            sign = -1.0 if near_is_min else 1.0
            near_x = center[0] + sign*(L*0.5)*u[0]; near_z = center[1] + sign*(L*0.5)*u[1]
            pole_len = max(0.1, 0.05*L)
            pole_a = np.array([near_x, feat['y_min'], near_z])
            pole_b = np.array([near_x + pole_len*u[0], feat['y_min'], near_z + pole_len*u[1]])
            pole_item = gl.GLLinePlotItem(pos=np.vstack([pole_a, pole_b]), mode='lines', color=box_color,
                                          width=ESCOOTER_BOX_WIDTH, antialias=True)
            view.addItem(pole_item); box_items.append(pole_item)
            sz = ESCOOTER_CFG['stand']; du = sz['length']*0.5; dv = sz['width']*0.5
            c_u = sz['center_from_pole'] if near_is_min else (-sz['center_from_pole'])
            czx = np.array([near_x + c_u*u[0], near_z + c_u*u[1]])
            vx, vz = v; y0 = feat['y_min'] + 0.02
            rect = []
            for su in (-1,1):
                for sv in (-1,1):
                    x = czx[0] + su*du*u[0] + sv*dv*vx
                    z = czx[1] + su*du*u[1] + sv*dv*vz
                    rect.append([x,y0,z])
            r0,r1,r2,r3 = rect
            rect_edges = np.vstack([r0,r1, r1,r3, r3,r2, r2,r0])
            rect_item = gl.GLLinePlotItem(pos=rect_edges, mode='lines', color=box_color, width=1, antialias=True)
            view.addItem(rect_item); box_items.append(rect_item)
            if HAS_GLTEXT and ESCOOTER_LABEL:
                vtxt = es["speed"]
                state_lbl = '危险驾驶' if es.get("state") == 'dangerous' else '正常行驶'
                color = QtGui.QColor(255,64,64) if es.get("state") == 'dangerous' else QtGui.QColor(255,204,102)
                txtpos = (near_x, feat['y_max']+0.1, near_z)
                txt = GLTextItem(pos=txtpos, text=f"E‑Scooter {state_lbl} {vtxt:.2f} m/s",
                                 color=color, font=QtGui.QFont("Microsoft YaHei", 14))
                view.addItem(txt); text_items.append(txt)

        for ped in results["people"]:
            xmin,xmax,ymin,ymax,zmin,zmax = ped["bbox"]
            segs = make_bbox_lines(xmin, xmax, ymin, ymax, zmin, zmax)
            box = gl.GLLinePlotItem(pos=segs, mode='lines', color=BOX_COLOR, width=BOX_WIDTH, antialias=True)
            view.addItem(box); box_items.append(box)
            if HAS_GLTEXT and LABEL_SPEED:
                cx3 = (xmin + xmax)/2; cy3 = ymax + 0.1; cz3 = (zmin + zmax)/2
                txt = GLTextItem(pos=(cx3, cy3, cz3), text=f"People {ped['speed']:.2f} m/s",
                                 color=QtGui.QColor(0,255,0), font=QtGui.QFont("Microsoft YaHei", 14))
                view.addItem(txt); text_items.append(txt)

        view.setWindowTitle(
            f'Radar 3D: {rel_t[idx]:.3f} s  行人:{counts["ped"]}  物体:{counts["obj"]}  滑板车+人:{counts["escooter"]} '
            f'(危险:{danger_cnt})  gate={assoc_gate:.2f} m'
        )

    timer.timeout.connect(update)
    timer.start()
    QtWidgets.QApplication.instance().exec_()

def main(argv=None):
    parser = argparse.ArgumentParser(description='Radar 3D viewer & analyzer for e-scooter detection.')
    parser.add_argument('--csv', help='指定需要加载的 CSV 文件（可为相对路径或文件名）。')
    parser.add_argument('--data-dir', default=DATA_DIR, help='数据目录（默认: 脚本目录下的 Data）。')
    parser.add_argument('--list', action='store_true', help='列出 data 目录下所有包含 scooter 的 CSV 文件后退出。')
    parser.add_argument('--analyze', action='store_true', help='不启用 3D 可视化，直接离线分析 e-scooter 行驶经过。')
    parser.add_argument('--all', action='store_true', help='与 --analyze 一起使用，批量分析 data 目录下的所有 scooter CSV。')
    args = parser.parse_args(argv)

    data_dir = os.path.abspath(args.data_dir)

    if args.list:
        files = list_scooter_csvs(data_dir)
        if files:
            print('可用的 e-scooter CSV 文件：')
            for path in files:
                rel = os.path.relpath(path, SCRIPT_DIR)
                print(f'  {rel}')
        else:
            print('未在指定目录找到包含 scooter 的 CSV 文件。')
        return

    if args.all and not args.analyze:
        parser.error('--all 需要与 --analyze 同时使用。')

    try:
        if args.all:
            csvs = list_scooter_csvs(data_dir)
            if args.csv:
                chosen = resolve_csv_path(args.csv, data_dir)
                csvs = [chosen] + [p for p in csvs if os.path.abspath(p) != os.path.abspath(chosen)]
            if not csvs:
                raise FileNotFoundError('未找到任何包含 scooter 的 CSV 文件。')
            targets = csvs
        else:
            targets = [resolve_csv_path(args.csv, data_dir)]
    except FileNotFoundError as exc:
        raise SystemExit(str(exc))

    if args.analyze:
        for path in targets:
            analyze_scooter_csv(path, verbose=True)
    else:
        run_viewer(targets[0])


if __name__ == '__main__':
    main()
