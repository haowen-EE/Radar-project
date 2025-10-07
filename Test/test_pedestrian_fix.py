#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试行人数据是否被误识别为滑板车
"""
import sys
import os
import csv
import numpy as np
from collections import deque
import math

# 导入主程序的参数和函数
sys.path.insert(0, os.path.dirname(__file__))

# 复制必要的参数
GRID_SIZE = 0.7
MIN_CLUSTER_POINTS = 3
DBSCAN_EPS = 0.8
DBSCAN_MIN_SAMPLES = 3

# ScooterRider参数（根据统计数据重新设定阈值）
#
# 经验依据：
#   - 滑板车簇的质心高度通常在 1.8~2.8m（雷达高度为0参考）
#   - 轴向宽/深范围约 0.4~1.3m，垂直高度 0.1~0.6m
#   - 点数一般 ≥8，面积 w*d 约 0.2 m² 以上
#   - 行人簇质心大多低于 1.0m，或在宽深面积上显著更小
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
SR_AVG_HORIZ_MIN = 0.28
SR_AVG_HORIZ_MAX = 1.20
SR_BASE_AREA_MIN = 0.18
SR_SPEED_MIN = 1.70
SR_SPEED_LOW = 1.30
SR_SPEED_MAX = 6.50

EWMA_ALPHA = 0.35
ROLL_WIN = 40
SPEED_WIN_PAIR = 10
SPEED_SANITY_MAX = 20.0
DISP_FACTOR = 1.5

def _cluster_geometry(cluster):
    bbox = cluster.get("bbox")
    if bbox is None:
        return None
    xmin, xmax, ymin, ymax, zmin, zmax = bbox
    w = max(0.0, xmax - xmin)
    h = max(0.0, ymax - ymin)
    d = max(0.0, zmax - zmin)
    cy = cluster.get("centroid_y")
    if cy is None:
        pts = cluster.get("pts")
        if pts is not None and len(pts):
            cy = float(np.median(pts[:, 1]))
        else:
            cy = 0.5 * (ymin + ymax)
    geom = {
        "w": float(w),
        "h": float(h),
        "d": float(d),
        "cy": float(cy),
        "ymin": float(ymin),
        "ymax": float(ymax)
    }
    geom["avg_horizontal"] = 0.5 * (geom["w"] + geom["d"])
    geom["max_horizontal"] = max(geom["w"], geom["d"])
    geom["min_horizontal"] = min(geom["w"], geom["d"])
    geom["aspect_main"] = geom["h"] / max(geom["max_horizontal"], 1e-6)
    geom["aspect_minor"] = geom["h"] / max(geom["min_horizontal"], 1e-6)
    return geom


def is_scooter_rider(cluster, npts, strict=True):
    """判断是否为滑板车+人"""
    geom = _cluster_geometry(cluster)
    if geom is None:
        return False

    pt_min = SR_POINTS_MIN if strict else max(5, int(SR_POINTS_MIN * 0.6))
    pt_max = SR_POINTS_MAX if strict else int(SR_POINTS_MAX * 1.4)
    if npts < pt_min or npts > pt_max:
        return False

    tol = 1.0 if strict else 1.25
    width_min = SR_WIDTH_MIN / tol
    width_max = SR_WIDTH_MAX * tol
    depth_min = SR_DEPTH_MIN / tol
    depth_max = SR_DEPTH_MAX * tol
    height_min = SR_HEIGHT_MIN / (1.1 if strict else 1.4)
    height_max = SR_HEIGHT_MAX * tol
    centroid_min = SR_CENTROID_Y_MIN / (1.05 if strict else 1.4)
    centroid_max = SR_CENTROID_Y_MAX * (1.10 if strict else 1.35)
    top_min = SR_TOP_Y_MIN / (1.05 if strict else 1.25)
    avg_h_min = SR_AVG_HORIZ_MIN / (1.15 if strict else 1.6)
    avg_h_max = SR_AVG_HORIZ_MAX * (1.15 if strict else 1.45)
    ratio_min = SR_WIDTH_DEPTH_RATIO_MIN / (1.0 if strict else 1.2)
    area_min = SR_BASE_AREA_MIN / (1.1 if strict else 1.5)

    if not (width_min <= geom["w"] <= width_max):
        return False
    if not (depth_min <= geom["d"] <= depth_max):
        return False
    if not (height_min <= geom["h"] <= height_max):
        return False
    if not (centroid_min <= geom["cy"] <= centroid_max):
        return False
    if geom["ymax"] < top_min:
        return False
    if not (avg_h_min <= geom["avg_horizontal"] <= avg_h_max):
        return False
    area = geom["w"] * geom["d"]
    if area < area_min:
        return False
    ratio = geom["w"] / max(geom["d"], 1e-6)
    if ratio < ratio_min:
        return False

    return True

def grid_cluster(pts):
    """简化的网格聚类"""
    if len(pts) == 0:
        return []
    
    # 按网格分组
    keys = (pts / GRID_SIZE).astype(int)
    grid_dict = {}
    for i, key in enumerate(keys):
        k = tuple(key)
        if k not in grid_dict:
            grid_dict[k] = []
        grid_dict[k].append(i)
    
    # 生成聚类
    clusters = []
    for idxs in grid_dict.values():
        if len(idxs) < MIN_CLUSTER_POINTS:
            continue
        cluster_pts = pts[idxs]
        xmin, ymin, zmin = cluster_pts.min(axis=0)
        xmax, ymax, zmax = cluster_pts.max(axis=0)
        cx, cz = cluster_pts[:, [0, 2]].mean(axis=0)
        w = float(xmax - xmin)
        h = float(ymax - ymin)
        d = float(zmax - zmin)
        cy = float(cluster_pts[:, 1].mean())
        clusters.append({
            "idxs": idxs,
            "pts": cluster_pts,
            "bbox": (xmin, xmax, ymin, ymax, zmin, zmax),
            "centroid_xz": (cx, cz),
            "centroid_y": cy,
            "dims": (w, h, d)
        })
    return clusters

class SimpleTrack:
    """简化的轨迹类"""
    def __init__(self, cx, cz, t, bbox, npts):
        self.centroids = deque(maxlen=ROLL_WIN)
        self.times = deque(maxlen=ROLL_WIN)
        self.c_smooth = np.array([cx, cz], float)
        self.last_bbox = bbox
        self.last_npts = npts
        self.last_centroid_y = None
        self.last_ymax = None
        self.last_dims = None
        self.geom_hits = 0
        self.last_geom_ok = False
        self.converted = False
        self.updated = True
        self.centroids.append((cx, cz))
        self.times.append(t)
        self._update_geom(bbox, npts)

    def _update_geom(self, bbox, npts):
        if bbox is not None:
            xmin, xmax, ymin, ymax, zmin, zmax = bbox
            self.last_bbox = bbox
            self.last_centroid_y = 0.5 * (ymin + ymax)
            self.last_ymax = ymax
            self.last_dims = (float(xmax - xmin), float(ymax - ymin), float(zmax - zmin))
        if npts is not None:
            self.last_npts = int(npts)
    
    def add(self, cx, cz, t, bbox, npts):
        self.c_smooth = (1 - EWMA_ALPHA) * self.c_smooth + EWMA_ALPHA * np.array([cx, cz], float)
        self.centroids.append((cx, cz))
        self.times.append(t)
        self._update_geom(bbox, npts)
    
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

# 测试主函数
def test_file(csv_file):
    print(f"\n{'='*70}")
    print(f"测试文件: {os.path.basename(csv_file)}")
    print("="*70)
    
    # 读取CSV
    with open(csv_file) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    # 按帧分组
    data = {}
    for r in rows:
        fn = int(r['frameNumber'])
        if fn not in data:
            data[fn] = []
        data[fn].append([float(r['x']), float(r['y']), float(r['z'])])
    
    frames = sorted(data.keys())
    print(f"总帧数: {len(frames)}")
    
    # 模拟追踪
    tracks = []
    conversion_count = 0
    MAX_MISS = 5
    
    for frame_idx, fn in enumerate(frames[:50]):  # 只测试前50帧
        pts = np.array(data[fn], float)
        if len(pts) == 0:
            continue
        
        # 聚类
        clusters = grid_cluster(pts)
        
        # 轨迹miss+1
        for tr in tracks:
            tr.miss = getattr(tr, 'miss', 0) + 1
            tr.updated = False
        
        # 简化的关联
        used = [False] * len(clusters)
        t = frame_idx * 0.1  # 假设100ms间隔
        
        for tr in tracks:
            best_ci = None
            best_d = None
            px, pz = tr.c_smooth
            
            for ci, c in enumerate(clusters):
                if used[ci]:
                    continue
                cx, cz = c["centroid_xz"]
                d = math.hypot(cx - px, cz - pz)
                if d <= 1.0 and (best_d is None or d < best_d):
                    best_d = d
                    best_ci = ci
            
            if best_ci is not None:
                used[best_ci] = True
                c = clusters[best_ci]
                cx, cz = c["centroid_xz"]
                npts = len(c["idxs"])
                tr.add(cx, cz, t, c["bbox"], npts)
                tr.miss = 0
                tr.updated = True
                geom_ok = is_scooter_rider(c, npts, strict=True)
                if geom_ok:
                    tr.geom_hits = min(tr.geom_hits + 1, 6)
                else:
                    tr.geom_hits = max(0, tr.geom_hits - 1)
                tr.last_geom_ok = geom_ok
        
        # 新建轨迹
        for ci, c in enumerate(clusters):
            if used[ci]:
                continue
            cx, cz = c["centroid_xz"]
            npts = len(c["idxs"])
            tr = SimpleTrack(cx, cz, t, c["bbox"], npts)
            tr.miss = 0
            tr.updated = True
            geom_ok = is_scooter_rider(c, npts, strict=True)
            tr.geom_hits = 1 if geom_ok else 0
            tr.last_geom_ok = geom_ok
            tracks.append(tr)
        
        for tr in tracks:
            if not getattr(tr, 'updated', False):
                tr.geom_hits = max(0, tr.geom_hits - 1)

        # 检查转换条件(Track→ScooterRider)
        for tr in tracks:
            if tr.converted:
                continue
            if len(tr.centroids) < 3:
                continue

            temp_cluster = {
                "bbox": tr.last_bbox,
                "centroid_y": tr.last_centroid_y,
            }
            npts = tr.last_npts if tr.last_npts is not None else 0
            geom_ok = tr.geom_hits >= 1 and is_scooter_rider(temp_cluster, npts, strict=False)
            speed = tr.speed_robust()
            dims = tr.last_dims or (0.0, 0.0, 0.0)
            height = dims[1]
            horiz_mean = 0.5 * (dims[0] + dims[2]) if dims else 0.0

            speed_ok = speed >= SR_SPEED_MIN
            slow_speed_ok = speed >= SR_SPEED_LOW and tr.geom_hits >= 3 and height >= 1.8 and horiz_mean <= SR_AVG_HORIZ_MAX * 1.05

            if geom_ok and (speed_ok or slow_speed_ok):
                tr.converted = True
                conversion_count += 1
                print(f"  [CONVERT] Frame{fn}: Track->SR (v={speed:.2f}m/s, npts={npts}, h={height:.2f}m)")
            elif geom_ok and not (speed_ok or slow_speed_ok):
                print(f"  [WAIT] Frame{fn}: 几何满足但速度偏低 (v={speed:.2f}m/s, geom_hits={tr.geom_hits}, h={height:.2f}m)")
            elif speed_ok and not geom_ok:
                print(f"  [GEOM] Frame{fn}: 速度满足但几何不满足 (v={speed:.2f}m/s, geom_hits={tr.geom_hits})")
        
        # 清理
        tracks = [tr for tr in tracks if tr.miss <= MAX_MISS]
    
    print(f"\n结果统计:")
    print(f"  Track→ScooterRider转换次数: {conversion_count}")
    
    # 判断文件类型
    filename_lower = csv_file.lower()
    is_pedestrian = 'walk' in filename_lower or 'man_' in filename_lower
    is_scooter = 'escooter' in filename_lower or 'scooter' in filename_lower
    
    if is_pedestrian:
        if conversion_count == 0:
            print(f"  结论: [OK] 行人未被误识别为滑板车")
        else:
            print(f"  结论: [ERR] 行人被误识别为滑板车")
    elif is_scooter:
        if conversion_count == 0:
            print(f"  结论: [ERR] 滑板车未被识别")
        else:
            print(f"  结论: [OK] 滑板车被识别为ScooterRider ({conversion_count}次转换)")
    else:
        print(f"  结论: Track→SR转换{conversion_count}次")

# 测试所有数据
if __name__ == '__main__':
    test_files = [
        # 行人数据
        r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\man_walk3.csv',
        r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\man_walk2.csv',
        # 滑板车数据
        r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\escooter1.csv',
        r'C:\Users\h\OneDrive\桌面\IWR6843-Read-Data-Python-MMWAVE-SDK-main\IWR6843-Read-Data-Python-MMWAVE-SDK-main\escooter2.csv',
    ]
    
    for f in test_files:
        if os.path.exists(f):
            test_file(f)
        else:
            print(f"\n文件不存在: {os.path.basename(f)}")
    
    print(f"\n{'='*70}")
    print("测试完成!")
    print("="*70)
