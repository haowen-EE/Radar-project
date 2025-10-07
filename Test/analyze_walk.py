import csv
import numpy as np

# 读取数据
data = {}
fid = -1
with open('man_walk3.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        det_idx = int(row.get('detIdx', 0) or 0)
        if det_idx == 0:
            fid += 1
            data[fid] = []
        try:
            x, y, z = float(row['x']), float(row['y']), float(row['z'])
            if fid >= 0:
                data[fid].append((x, y, z))
        except:
            continue

print(f"man_walk3.csv 分析:")
print(f"总帧数: {len(data)}")

# 统计
stats = []
for pts in data.values():
    if len(pts) < 3:
        continue
    P = np.array(pts)
    npts = len(pts)
    cy = P[:, 1].mean()
    h = P[:, 1].max() - P[:, 1].min()
    w = P[:, 0].max() - P[:, 0].min()
    d = P[:, 2].max() - P[:, 2].min()
    stats.append((npts, cy, h, w, d))

npts_list = [s[0] for s in stats]
cy_list = [s[1] for s in stats]
h_list = [s[2] for s in stats]
w_list = [s[3] for s in stats]
d_list = [s[4] for s in stats]

print(f"\n点数: {min(npts_list)}-{max(npts_list)} (中位数 {int(np.median(npts_list))})")
print(f"质心Y: {min(cy_list):.2f}-{max(cy_list):.2f}m (中位数 {np.median(cy_list):.2f})")
print(f"高度: {min(h_list):.2f}-{max(h_list):.2f}m (中位数 {np.median(h_list):.2f})")
print(f"宽度: {min(w_list):.2f}-{max(w_list):.2f}m (中位数 {np.median(w_list):.2f})")
print(f"深度: {min(d_list):.2f}-{max(d_list):.2f}m (中位数 {np.median(d_list):.2f})")

# 检查当前ScooterRider参数匹配情况
SR_POINTS_MIN, SR_POINTS_MAX = 3, 300
SR_CENTROID_Y_MIN, SR_CENTROID_Y_MAX = 0.20, 5.00
SR_HEIGHT_MIN, SR_HEIGHT_MAX = 0.30, 4.00
SR_WIDTH_MIN, SR_WIDTH_MAX = 0.25, 1.80

match_count = 0
for npts, cy, h, w, d in stats:
    if (SR_POINTS_MIN <= npts <= SR_POINTS_MAX and
        SR_CENTROID_Y_MIN <= cy <= SR_CENTROID_Y_MAX and
        SR_HEIGHT_MIN <= h <= SR_HEIGHT_MAX and
        SR_WIDTH_MIN <= w <= SR_WIDTH_MAX and
        SR_WIDTH_MIN <= d <= SR_WIDTH_MAX):
        match_count += 1

print(f"\n匹配当前ScooterRider参数的帧: {match_count}/{len(stats)} ({match_count*100/len(stats):.1f}%)")
print("这就是为什么行人被误判为滑板车!")
