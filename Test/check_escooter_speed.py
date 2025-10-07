#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""检查滑板车数据的速度分布"""
import csv

files = ['escooter1.csv', 'escooter2.csv']

for f in files:
    print(f'\n{"="*60}')
    print(f'文件: {f}')
    print("="*60)
    
    with open(f) as csvfile:
        reader = csv.DictReader(csvfile)
        speeds = [abs(float(r['v'])) for r in reader]
    
    speeds_sorted = sorted(speeds)
    
    print(f'总点数: {len(speeds)}')
    print(f'速度范围: {min(speeds):.3f} - {max(speeds):.3f} m/s')
    print(f'速度中位数: {speeds_sorted[len(speeds)//2]:.3f} m/s')
    print(f'速度均值: {sum(speeds)/len(speeds):.3f} m/s')
    
    print(f'\n速度阈值分析:')
    print(f'  >2.0 m/s: {sum(1 for v in speeds if v>2.0)}/{len(speeds)} ({sum(1 for v in speeds if v>2.0)/len(speeds)*100:.1f}%)')
    print(f'  >2.5 m/s: {sum(1 for v in speeds if v>2.5)}/{len(speeds)} ({sum(1 for v in speeds if v>2.5)/len(speeds)*100:.1f}%)')
    print(f'  >2.8 m/s: {sum(1 for v in speeds if v>2.8)}/{len(speeds)} ({sum(1 for v in speeds if v>2.8)/len(speeds)*100:.1f}%)')
    print(f'  >3.0 m/s: {sum(1 for v in speeds if v>3.0)}/{len(speeds)} ({sum(1 for v in speeds if v>3.0)/len(speeds)*100:.1f}%)')
    
    # 检查速度=0的点数
    zero_speed = sum(1 for v in speeds if v == 0.0)
    print(f'\n  速度=0: {zero_speed}/{len(speeds)} ({zero_speed/len(speeds)*100:.1f}%)')

print(f'\n{"="*60}')
print('结论:')
print('如果大部分速度为0或<2.8m/s,说明CSV中的v列不是实时速度!')
print('可能需要从位置变化计算速度。')
print("="*60)
