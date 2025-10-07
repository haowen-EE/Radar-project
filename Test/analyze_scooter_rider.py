#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
分析静止滑板车+人的点云数据特征
用于设计运动滑板车+人的识别参数
"""

import csv
import numpy as np
from collections import defaultdict

def analyze_scooter_rider_data(csv_file='unmove_scooter.csv'):
    """分析滑板车+人组合的点云特征"""
    
    # 读取数据
    data = defaultdict(list)
    frame_id = -1
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            det_idx = int(row.get('detIdx', 0) or 0)
            if det_idx == 0:
                frame_id += 1
            try:
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                data[frame_id].append([x, y, z])
            except:
                pass
    
    # 分析所有帧
    print('=' * 60)
    print('静止滑板车+人 点云数据分析报告')
    print('=' * 60)
    print(f'\n总帧数: {len(data)}')
    
    all_points = []
    for fid, pts in data.items():
        if pts:
            all_points.extend(pts)
    
    pts_array = np.array(all_points)
    print(f'总点数: {len(pts_array)}')
    print(f'平均每帧点数: {len(pts_array) / len(data):.1f}')
    
    # 计算边界框
    x_min, x_max = pts_array[:, 0].min(), pts_array[:, 0].max()
    y_min, y_max = pts_array[:, 1].min(), pts_array[:, 1].max()
    z_min, z_max = pts_array[:, 2].min(), pts_array[:, 2].max()
    
    width = x_max - x_min
    height = y_max - y_min
    depth = z_max - z_min
    volume = width * height * depth
    
    print(f'\n{"="*60}')
    print('空间尺寸特征（整体边界框）')
    print('=' * 60)
    print(f'X 轴（横向）: [{x_min:.3f}, {x_max:.3f}] m  →  宽度: {width:.3f} m')
    print(f'Y 轴（垂直）: [{y_min:.3f}, {y_max:.3f}] m  →  高度: {height:.3f} m')
    print(f'Z 轴（纵深）: [{z_min:.3f}, {z_max:.3f}] m  →  深度: {depth:.3f} m')
    print(f'体积: {volume:.3f} m³')
    
    # 质心
    centroid = pts_array.mean(axis=0)
    print(f'\n质心位置: X={centroid[0]:.3f} m, Y={centroid[1]:.3f} m, Z={centroid[2]:.3f} m')
    
    # 每帧统计
    frame_points = [len(data[fid]) for fid in data.keys() if len(data[fid]) > 0]
    print(f'\n{"="*60}')
    print('点数分布统计')
    print('=' * 60)
    print(f'最少点数: {min(frame_points)}')
    print(f'最多点数: {max(frame_points)}')
    print(f'中位数点数: {np.median(frame_points):.1f}')
    print(f'标准差: {np.std(frame_points):.1f}')
    
    # 分析点云在Y轴的分层（区分滑板车和人）
    y_sorted = np.sort(pts_array[:, 1])
    y_25 = np.percentile(y_sorted, 25)
    y_50 = np.percentile(y_sorted, 50)
    y_75 = np.percentile(y_sorted, 75)
    
    print(f'\n{"="*60}')
    print('垂直分层分析（区分滑板车底部和人体）')
    print('=' * 60)
    print(f'下四分位 (25%): Y = {y_25:.3f} m  [可能是滑板车踏板]')
    print(f'中位数 (50%):   Y = {y_50:.3f} m  [可能是人腿部/腰部]')
    print(f'上四分位 (75%): Y = {y_75:.3f} m  [可能是人上半身]')
    
    # 计算水平扩散（XZ平面）
    xz_points = pts_array[:, [0, 2]]
    xz_center = xz_points.mean(axis=0)
    distances = np.sqrt(((xz_points - xz_center) ** 2).sum(axis=1))
    radius_50 = np.percentile(distances, 50)
    radius_90 = np.percentile(distances, 90)
    
    print(f'\n{"="*60}')
    print('水平扩散特征（XZ平面，俯视图）')
    print('=' * 60)
    print(f'质心到点云的距离中位数: {radius_50:.3f} m')
    print(f'质心到点云的距离90%分位: {radius_90:.3f} m')
    print(f'有效半径范围: [{radius_50:.3f}, {radius_90:.3f}] m')
    
    # 生成识别参数建议
    print(f'\n{"="*60}')
    print('🎯 运动滑板车+人识别参数建议')
    print('=' * 60)
    
    # 保守估计：考虑运动时点云可能更分散
    width_margin = 1.3
    height_margin = 1.2
    depth_margin = 1.3
    points_margin_low = 0.7
    points_margin_high = 1.5
    
    print('\n1. 尺寸范围（考虑运动扩散，增加30%余量）:')
    print(f'   宽度: {width * points_margin_low:.3f} ~ {width * width_margin:.3f} m')
    print(f'   高度: {height * points_margin_low:.3f} ~ {height * height_margin:.3f} m')
    print(f'   深度: {depth * points_margin_low:.3f} ~ {depth * depth_margin:.3f} m')
    print(f'   体积: {volume * 0.5:.3f} ~ {volume * 2.0:.3f} m³')
    
    print(f'\n2. 点数范围（静止: {int(np.median(frame_points))} 点）:')
    print(f'   最小点数: {int(min(frame_points) * points_margin_low)}')
    print(f'   最大点数: {int(max(frame_points) * points_margin_high)}')
    print(f'   典型范围: {int(np.median(frame_points) * 0.7)} ~ {int(np.median(frame_points) * 1.5)}')
    
    print(f'\n3. 形状特征:')
    aspect_xz = max(width, depth) / min(width, depth)
    aspect_y = height / min(width, depth)
    print(f'   水平纵横比 (max(W,D)/min(W,D)): {aspect_xz:.2f}')
    print(f'   高度比 (H/min(W,D)): {aspect_y:.2f}')
    print(f'   建议: 纵横比 > 1.5, 高度比 > 2.0 (人+滑板车较高且狭长)')
    
    print(f'\n4. 速度范围（根据电动滑板车特性）:')
    print(f'   最低速度: 1.5 m/s (~5.4 km/h) - 排除静止和步行')
    print(f'   最高速度: 8.0 m/s (~28.8 km/h) - 常见限速范围')
    print(f'   典型速度: 3.0 ~ 6.0 m/s (~10-22 km/h)')
    
    print(f'\n5. 质心高度特征:')
    print(f'   静止质心高度: {centroid[1]:.3f} m')
    print(f'   建议范围: {centroid[1] * 0.7:.3f} ~ {centroid[1] * 1.3:.3f} m')
    print(f'   (人+滑板车质心较高，区别于单独的物体)')
    
    # 返回参数字典
    params = {
        'width': (width * points_margin_low, width * width_margin),
        'height': (height * points_margin_low, height * height_margin),
        'depth': (depth * points_margin_low, depth * depth_margin),
        'volume': (volume * 0.5, volume * 2.0),
        'points': (int(min(frame_points) * points_margin_low), int(max(frame_points) * points_margin_high)),
        'points_typical': (int(np.median(frame_points) * 0.7), int(np.median(frame_points) * 1.5)),
        'aspect_xz_min': 1.5,
        'aspect_y_min': 2.0,
        'speed': (1.5, 8.0),
        'centroid_y': (centroid[1] * 0.7, centroid[1] * 1.3)
    }
    
    print(f'\n{"="*60}')
    print('分析完成！')
    print('=' * 60)
    
    return params


if __name__ == '__main__':
    params = analyze_scooter_rider_data()
    print('\n\n📋 可以直接用于代码的参数：')
    print('=' * 60)
    for key, value in params.items():
        print(f'{key}: {value}')
