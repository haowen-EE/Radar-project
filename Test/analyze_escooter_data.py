"""
分析 escooter1.csv 和 escooter2.csv 的点云特征
找出远距离滑板车的实际尺寸、点数、质心分布
"""
import csv
import numpy as np
from collections import defaultdict

def analyze_csv(filename):
    print(f"\n{'='*60}")
    print(f"分析文件: {filename}")
    print(f"{'='*60}")
    
    # 按帧读取
    data = {}
    fid = -1
    with open(filename, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            det_idx = int(row.get('detIdx', 0) or 0)
            if det_idx == 0:
                fid += 1
                data[fid] = []
            try:
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                if fid >= 0:
                    data[fid].append((x, y, z))
            except:
                continue
    
    print(f"总帧数: {len(data)}")
    
    # 统计每帧的特征
    frame_stats = []
    for fid, pts in data.items():
        if len(pts) < 3:
            continue
        P = np.array(pts)
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        
        # 距离(雷达中心到质心)
        cx, cy, cz = x.mean(), y.mean(), z.mean()
        dist = np.sqrt(cx**2 + cz**2)
        
        # 尺寸
        w = x.max() - x.min()
        h = y.max() - y.min()
        d = z.max() - z.min()
        
        frame_stats.append({
            'fid': fid,
            'npts': len(pts),
            'dist': dist,
            'w': w,
            'h': h,
            'd': d,
            'cy': cy,
            'y_min': y.min(),
            'y_max': y.max()
        })
    
    # 按距离分组统计
    near = [s for s in frame_stats if s['dist'] < 3.0]
    mid = [s for s in frame_stats if 3.0 <= s['dist'] < 5.0]
    far = [s for s in frame_stats if s['dist'] >= 5.0]
    
    def print_stats(name, stats):
        if not stats:
            print(f"\n{name}: 无数据")
            return
        print(f"\n{name} ({len(stats)}帧):")
        print(f"  距离: {np.min([s['dist'] for s in stats]):.1f} - {np.max([s['dist'] for s in stats]):.1f} m")
        print(f"  点数: {np.min([s['npts'] for s in stats])} - {np.max([s['npts'] for s in stats])} (中位数: {np.median([s['npts'] for s in stats]):.0f})")
        print(f"  宽度: {np.min([s['w'] for s in stats]):.2f} - {np.max([s['w'] for s in stats]):.2f} m (中位数: {np.median([s['w'] for s in stats]):.2f})")
        print(f"  高度: {np.min([s['h'] for s in stats]):.2f} - {np.max([s['h'] for s in stats]):.2f} m (中位数: {np.median([s['h'] for s in stats]):.2f})")
        print(f"  深度: {np.min([s['d'] for s in stats]):.2f} - {np.max([s['d'] for s in stats]):.2f} m (中位数: {np.median([s['d'] for s in stats]):.2f})")
        print(f"  质心Y: {np.min([s['cy'] for s in stats]):.2f} - {np.max([s['cy'] for s in stats]):.2f} m (中位数: {np.median([s['cy'] for s in stats]):.2f})")
        print(f"  Y范围: [{np.min([s['y_min'] for s in stats]):.2f}, {np.max([s['y_max'] for s in stats]):.2f}]")
        
        # 计算高宽比
        ratios = [s['h'] / ((s['w'] + s['d']) / 2) for s in stats if (s['w'] + s['d']) > 0]
        if ratios:
            print(f"  高宽比: {np.min(ratios):.2f} - {np.max(ratios):.2f} (中位数: {np.median(ratios):.2f})")
    
    print_stats("近距离 (<3m)", near)
    print_stats("中距离 (3-5m)", mid)
    print_stats("远距离 (≥5m)", far)
    
    # 找出点数少的帧(可能是远距离)
    low_point_frames = [s for s in frame_stats if s['npts'] < 15]
    print(f"\n低点数帧 (<15点): {len(low_point_frames)}帧")
    if low_point_frames:
        print(f"  距离分布: {np.min([s['dist'] for s in low_point_frames]):.1f} - {np.max([s['dist'] for s in low_point_frames]):.1f} m")
        print(f"  点数分布: {np.min([s['npts'] for s in low_point_frames])} - {np.max([s['npts'] for s in low_point_frames])}")
    
    # 找出不符合当前ScooterRider参数的帧
    # 当前参数: 点数15-250, 宽0.25-1.8, 高1.2-2.4, 深0.25-1.8, 质心Y 0.4-1.1
    rejected = []
    for s in frame_stats:
        reasons = []
        if s['npts'] < 15 or s['npts'] > 250:
            reasons.append(f"点数{s['npts']}")
        if not (0.25 <= s['w'] <= 1.8):
            reasons.append(f"宽{s['w']:.2f}")
        if not (1.2 <= s['h'] <= 2.4):
            reasons.append(f"高{s['h']:.2f}")
        if not (0.25 <= s['d'] <= 1.8):
            reasons.append(f"深{s['d']:.2f}")
        if not (0.4 <= s['cy'] <= 1.1):
            reasons.append(f"质心Y{s['cy']:.2f}")
        
        if reasons:
            rejected.append((s, reasons))
    
    print(f"\n不符合当前ScooterRider参数的帧: {len(rejected)}/{len(frame_stats)} ({len(rejected)*100/len(frame_stats):.1f}%)")
    
    # 按拒绝原因分类
    reason_counts = defaultdict(int)
    for s, reasons in rejected:
        for r in reasons:
            reason_counts[r] += 1
    
    print("\n拒绝原因统计:")
    for reason, count in sorted(reason_counts.items(), key=lambda x: x[1], reverse=True):
        print(f"  {reason}: {count}帧")
    
    # 找出典型的被拒绝帧(远距离)
    far_rejected = [(s, reasons) for s, reasons in rejected if s['dist'] >= 4.0]
    if far_rejected:
        print(f"\n远距离被拒绝帧 (≥4m): {len(far_rejected)}帧")
        sample = far_rejected[:5]
        for s, reasons in sample:
            print(f"  帧{s['fid']}: 距离{s['dist']:.1f}m, {s['npts']}点, [{s['w']:.2f}×{s['h']:.2f}×{s['d']:.2f}], cy={s['cy']:.2f}, 原因: {', '.join(reasons)}")

# 分析两个文件
analyze_csv('escooter1.csv')
analyze_csv('escooter2.csv')
