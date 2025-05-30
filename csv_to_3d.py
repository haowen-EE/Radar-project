#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
csv_to_3d_with_time.py

Dynamic 3D radar point-cloud visualization synchronized by real CSV timestamps.
Reads a CSV with fields: timeStamp, subFrame, detIdx, x, y, z, ...
Groups rows into sequential frames based on detIdx reset.
Computes per-frame relative times based on timeStamp.
Updates GL scatter plot per frame according to actual time intervals.
"""

import csv
from datetime import datetime
import numpy as np
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.opengl as gl


def main():
    # CSV 文件路径
    csv_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv'

    # 1. 读取 CSV，按 detIdx==0 分割帧并记录时间戳
    data = {}        # frame_id -> list of (x,y,z)
    time_map = {}    # frame_id -> datetime timestamp
    frame_id = -1
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 检测点索引，用于新帧识别
            det_idx = int(row.get('detIdx', 0))
            if det_idx == 0:
                # 新帧开始
                frame_id += 1
                data[frame_id] = []
                # 记录该帧的时间戳
                ts_str = row.get('timeStamp', None)
                if ts_str:
                    try:
                        time_map[frame_id] = datetime.fromisoformat(ts_str)
                    except Exception:
                        pass
            # 添加点坐标
            try:
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
            except Exception:
                continue
            data[frame_id].append((x, y, z))

    frames = sorted(data.keys())
    if not frames:
        raise RuntimeError('No frames parsed from CSV')

    # 2. 计算每帧的相对时间轴（秒）
    rel_times = []
    if len(frames) > 1 and len(time_map) >= 2:
        rel_times = [0.0] * len(frames)
        last_time = time_map.get(frames[0], None)
        for i in range(1, len(frames)):
            fid = frames[i]
            if fid in time_map and last_time is not None:
                diff = (time_map[fid] - last_time).total_seconds()
                if diff <= 0:
                    diff = 0.001  # 同一时间戳的帧，给微小间隔
                rel_times[i] = rel_times[i-1] + diff
                last_time = time_map[fid]
            else:
                # 缺失时间戳，默认50ms
                rel_times[i] = rel_times[i-1] + 0.05
    else:
        # 无效时间戳或单帧，统一50ms间隔
        rel_times = [i * 0.05 for i in range(len(frames))]

    total_time = rel_times[-1]

    # 3. 初始化 Qt 与 OpenGL
    app = QtWidgets.QApplication([])
    view = gl.GLViewWidget()
    view.opts['distance'] = 20
    view.setWindowTitle('Radar 3D Live: 0.000 s')
    view.show()

    # 坐标轴与网格
    axis = gl.GLAxisItem()
    axis.setSize(x=10, y=10, z=10)
    view.addItem(axis)
    grid = gl.GLGridItem()
    grid.setSize(10, 10)
    grid.setSpacing(1, 1)
    view.addItem(grid)

    # 散点图对象
    scatter = gl.GLScatterPlotItem()
    view.addItem(scatter)

    # 4. 定时器驱动更新
    timer = QtCore.QTimer()
    timer.setInterval(20)  # 20ms 检查一次
    elapsed = QtCore.QElapsedTimer()
    elapsed.start()
    idx = 0

    def update():
        nonlocal idx
        sec = elapsed.elapsed() / 1000.0
        # 循环回放
        if sec > total_time:
            elapsed.restart()
            idx = 0
            sec = 0.0
        # 当达到下一个帧时间时，前进一帧
        if idx < len(frames) - 1 and sec >= rel_times[idx + 1]:
            idx += 1
        # 更新散点数据
        pts = np.array(data[frames[idx]])
        scatter.setData(pos=pts, size=3, color=(1, 0, 0, 1))
        # 更新标题
        view.setWindowTitle(f'Radar 3D Live: {rel_times[idx]:.3f} s (Frame {frames[idx]})')

    timer.timeout.connect(update)
    timer.start()

    # 5. 启动 Qt 应用
    QtWidgets.QApplication.instance().exec_()


if __name__ == '__main__':
    main()
