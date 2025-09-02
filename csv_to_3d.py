#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
csv_to_3d3.py — 3D radar point cloud with frame timing

把点云的可视化运动改为沿 y 轴显示：
采用绕 x 轴 +90° 旋转：x' = x, y' = z, z' = -y （即 z → y）
"""

import csv
from datetime import datetime
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.opengl as gl

# === 坐标映射模式（默认把原始 z 方向的位移显示到 y 轴上）===
# 可选: "native" | "map_z_to_y" | "map_z_to_x" | "map_x_to_y"
COORD_MODE = "map_z_to_y"

def transform_xyz(x, y, z):
    m = COORD_MODE
    if m == "native":
        return (x, y, z)
    elif m == "map_z_to_y":
        # 绕 x 轴 +90°：x' = x, y' = z, z' = -y
        return (x, z, -y)
    elif m == "map_z_to_x":
        # 绕 y 轴 +90°：x' = z, y' = y, z' = -x
        return (z, y, -x)
    elif m == "map_x_to_y":
        # 绕 z 轴 +90°：x' = -y, y' = x, z' = z （把原始 x 映射到 y）
        return (-y, x, z)
    else:
        return (x, y, z)

def main():
    # TODO: 改成你的 CSV 路径
    csv_file = 'keep_walking.csv'

    # 1) 按 detIdx==0 分帧，并记录每帧时间戳
    data = {}       # frame_id -> list[(x,y,z)]
    time_map = {}   # frame_id -> datetime
    frame_id = -1

    with open(csv_file, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            det_idx = int(row.get('detIdx', 0) or 0)
            if det_idx == 0:
                frame_id += 1
                data[frame_id] = []
                ts_str = row.get('timeStamp')
                if ts_str:
                    try:
                        time_map[frame_id] = datetime.fromisoformat(ts_str)
                    except Exception:
                        pass
            try:
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                x, y, z = transform_xyz(x, y, z)
            except Exception:
                continue
            data[frame_id].append((x, y, z))

    frames = sorted(data.keys())
    if not frames:
        raise RuntimeError("No frames parsed from CSV")

    # 2) 计算相对时间轴（秒）
    rel_times = [0.0] * len(frames)
    if len(frames) > 1 and len(time_map) >= 2:
        last = time_map.get(frames[0])
        for i in range(1, len(frames)):
            fid = frames[i]
            if fid in time_map and last is not None:
                dt = (time_map[fid] - last).total_seconds()
                if dt <= 0:
                    dt = 0.001
                rel_times[i] = rel_times[i-1] + dt
                last = time_map[fid]
            else:
                rel_times[i] = rel_times[i-1] + 0.05
    else:
        for i in range(1, len(frames)):
            rel_times[i] = rel_times[i-1] + 0.05

    total_time = rel_times[-1]

    # 3) Qt & OpenGL
    app = QtWidgets.QApplication([])
    view = gl.GLViewWidget()
    view.setWindowTitle('Radar 3D Live')
    view.setCameraPosition(azimuth=45, elevation=20, distance=20)
    view.opts['distance'] = 20
    view.show()

    axis = gl.GLAxisItem(); axis.setSize(x=10, y=10, z=10); view.addItem(axis)
    grid = gl.GLGridItem(); grid.setSize(10, 10); grid.setSpacing(1, 1); view.addItem(grid)

    scatter = gl.GLScatterPlotItem(size=3, color=(1, 0, 0, 1))
    view.addItem(scatter)

    timer = QtCore.QTimer(); timer.setInterval(20)
    elapsed = QtCore.QElapsedTimer(); elapsed.start()
    idx = 0

    def update():
        nonlocal idx
        sec = elapsed.elapsed() / 1000.0
        if total_time > 0 and sec > total_time:
            elapsed.restart(); idx = 0; sec = 0.0
        if idx < len(frames) - 1 and sec >= rel_times[idx + 1]:
            idx += 1
        pts = np.array(data[frames[idx]]) if data[frames[idx]] else np.zeros((0, 3))
        scatter.setData(pos=pts)
        view.setWindowTitle(f'Radar 3D Live: {rel_times[idx]:.3f}s (Frame {frames[idx]}) [{COORD_MODE}]')

    timer.timeout.connect(update)
    timer.start()
    QtWidgets.QApplication.instance().exec_()

if __name__ == '__main__':
    main()
