#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
csv_to_3d.py

用 PyQtGraph 在 Windows 下实时显示 CSV 点云，兼容 Python 3.13。
"""

import numpy as np
import csv
from collections import defaultdict

# 从 pyqtgraph.Qt 导入 QtWidgets 而不是 QtGui
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.opengl as gl

def main():
    # 1. 读取 CSV，并按 subFrame 分组
    data = defaultdict(list)
    with open('xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            fid = int(row['subFrame'])
            data[fid].append((float(row['x']), float(row['y']), float(row['z'])))
    frames = sorted(data.keys())

    # 2. 创建 Qt 应用和 OpenGL 视图
    app = QtWidgets.QApplication([])      # ← 这里用 QtWidgets
    view = gl.GLViewWidget()
    view.opts['distance'] = 20
    view.show()
    view.setWindowTitle('动态点云 (PyQtGraph)')

    # 创建一个散点图 Item
    scatter = gl.GLScatterPlotItem()
    view.addItem(scatter)

    # 3. 定时器更新函数
    idx = 0
    def update():
        nonlocal idx
        pts = np.array(data[frames[idx]])
        scatter.setData(pos=pts, size=3, color=(1,0,0,1))
        idx = (idx + 1) % len(frames)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)  # 20 Hz

    # 4. 启动应用
    QtWidgets.QApplication.instance().exec_()

if __name__ == '__main__':
    main()
