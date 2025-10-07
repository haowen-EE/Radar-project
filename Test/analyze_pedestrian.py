#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
分析行人数据特征
"""
import pandas as pd
import numpy as np

files = ['man_walk2.csv', 'man_walk3.csv', 'keep_walking.csv']

for f in files:
    print(f'\n{"="*60}')
    print(f'文件: {f}')
    print("="*60)
    
    df = pd.read_csv(f)
    speeds = df['v'].abs()
    
    # 基本统计
    print(f'总点数: {len(df)}')
    print(f'速度范围: {speeds.min():.3f} - {speeds.max():.3f} m/s')
    print(f'速度中位数: {speeds.median():.3f} m/s')
    print(f'速度均值: {speeds.mean():.3f} m/s')
    print(f'速度标准差: {speeds.std():.3f} m/s')
    
    # 速度分布
    print(f'\n速度分布:')
    print(f'  0-1.0 m/s: {(speeds < 1.0).sum()} ({(speeds < 1.0).sum()/len(speeds)*100:.1f}%)')
    print(f'  1.0-2.0 m/s: {((speeds >= 1.0) & (speeds < 2.0)).sum()} ({((speeds >= 1.0) & (speeds < 2.0)).sum()/len(speeds)*100:.1f}%)')
    print(f'  2.0-2.5 m/s: {((speeds >= 2.0) & (speeds < 2.5)).sum()} ({((speeds >= 2.0) & (speeds < 2.5)).sum()/len(speeds)*100:.1f}%)')
    print(f'  2.5-3.0 m/s: {((speeds >= 2.5) & (speeds < 3.0)).sum()} ({((speeds >= 2.5) & (speeds < 3.0)).sum()/len(speeds)*100:.1f}%)')
    print(f'  >3.0 m/s: {(speeds >= 3.0).sum()} ({(speeds >= 3.0).sum()/len(speeds)*100:.1f}%)')
    
    # 空间特征
    print(f'\nY坐标(相对雷达高度):')
    print(f'  范围: {df["y"].min():.3f} - {df["y"].max():.3f} m')
    print(f'  中位数: {df["y"].median():.3f} m')

print(f'\n{"="*60}')
print('分析总结:')
print("="*60)
print('如果行人数据大部分速度<2.5m/s,而滑板车通常>2.5m/s,')
print('可以使用速度作为主要区分特征。')
