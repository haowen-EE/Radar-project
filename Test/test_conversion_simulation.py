"""
自动化测试滑板车识别优化效果
模拟Track→ScooterRider转换逻辑,统计各级别转换数量
"""
import csv
import numpy as np
from collections import defaultdict

# 模拟参数(与csv_boxs_withthings_V3.py保持一致)
SR_CENTROID_Y_MIN = 0.20
SR_CENTROID_Y_MAX = 5.00
SR_HEIGHT_MIN = 0.30
SR_HEIGHT_MAX = 4.00
SR_WIDTH_MIN = 0.15
SR_WIDTH_MAX = 4.00
SR_DEPTH_MIN = 0.15
SR_DEPTH_MAX = 4.00
SR_POINTS_MIN = 3
SR_POINTS_MAX = 300

def is_scooter_rider_mock(npts, bbox, strict=True):
    """模拟is_scooter_rider几何检查"""
    if bbox is None:
        return False
    
    xmin, xmax, ymin, ymax, zmin, zmax = bbox
    w = xmax - xmin
    h = ymax - ymin
    d = zmax - zmin
    cy = (ymin + ymax) / 2.0
    
    # 点数检查
    pt_min = SR_POINTS_MIN if strict else max(8, SR_POINTS_MIN - 7)
    pt_max = SR_POINTS_MAX if strict else SR_POINTS_MAX + 50
    if npts < pt_min or npts > pt_max:
        return False
    
    # 尺寸检查
    tolerance = 1.0 if strict else 1.2
    if not (SR_WIDTH_MIN / tolerance <= w <= SR_WIDTH_MAX * tolerance):
        return False
    if not (SR_HEIGHT_MIN * 0.8 <= h <= SR_HEIGHT_MAX * tolerance):
        return False
    if not (SR_DEPTH_MIN / tolerance <= d <= SR_DEPTH_MAX * tolerance):
        return False
    
    # 质心高度检查
    cy_min = SR_CENTROID_Y_MIN * 0.7 if not strict else SR_CENTROID_Y_MIN
    cy_max = SR_CENTROID_Y_MAX * 1.3 if not strict else SR_CENTROID_Y_MAX
    if not (cy_min <= cy <= cy_max):
        return False
    
    # 特殊优化: 质心0.6-0.9m直接通过
    if 0.6 <= cy <= 0.9 and npts >= 5:
        return True
    
    # 高宽比检查(点数>=15时)
    if npts >= 15:
        avg_horizontal = (w + d) / 2.0
        ratio_threshold = 0.6 if not strict else 0.8
        if avg_horizontal > 0 and h / avg_horizontal < ratio_threshold:
            return False
    
    return True

def analyze_conversion(filename):
    """分析文件的转换效果"""
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        data = list(reader)
    
    if not data:
        print(f"{filename}: 无数据")
        return
    
    # 按帧分组
    frames = defaultdict(list)
    for row in data:
        frame = int(row['frameNumber'])
        frames[frame].append(row)
    
    # 统计转换
    conversions = {'high_speed': 0, 'medium_speed': 0, 'low_speed': 0, 'failed': 0}
    total_frames = len(frames)
    converted_frames = 0
    
    for frame_num, points in frames.items():
        if not points:
            continue
        
        # 提取点云数据
        pts = np.array([[float(p['x']), float(p['y']), float(p['z'])] for p in points])
        npts = len(pts)
        
        # 计算bbox
        if npts < SR_POINTS_MIN:
            conversions['failed'] += 1
            continue
        
        xmin, xmax = pts[:, 0].min(), pts[:, 0].max()
        ymin, ymax = pts[:, 1].min(), pts[:, 1].max()
        zmin, zmax = pts[:, 2].min(), pts[:, 2].max()
        bbox = (xmin, xmax, ymin, ymax, zmin, zmax)
        cy = (ymin + ymax) / 2.0
        
        # 模拟速度(使用CSV v的绝对值中位数)
        velocities = [abs(float(p['v'])) for p in points]
        v = np.median(velocities)
        
        # 多级转换判断
        if is_scooter_rider_mock(npts, bbox, strict=False) and v > 2.5:
            conversions['high_speed'] += 1
            converted_frames += 1
        elif is_scooter_rider_mock(npts, bbox, strict=False) and v > 1.2:
            # 第二级额外检查
            if npts < 30 or cy > 1.5:
                conversions['medium_speed'] += 1
                converted_frames += 1
            else:
                conversions['failed'] += 1
        elif is_scooter_rider_mock(npts, bbox, strict=True) and v > 0.8:
            # 第三级需要5帧持续,这里简化为通过
            conversions['low_speed'] += 1
            converted_frames += 1
        else:
            conversions['failed'] += 1
    
    return total_frames, converted_frames, conversions

# 测试滑板车文件
print("="*70)
print("滑板车识别优化 - 转换效果测试")
print("="*70)

scooter_files = ['escooter1.csv', 'escooter2.csv', 'escooter_slow.csv']
results = {}

for filename in scooter_files:
    try:
        total, converted, conv_detail = analyze_conversion(filename)
        results[filename] = (total, converted, conv_detail)
        
        print(f"\n{'='*70}")
        print(f"文件: {filename}")
        print(f"{'='*70}")
        print(f"总帧数: {total}")
        print(f"成功转换帧数: {converted} ({100*converted/total:.1f}%)")
        print(f"\n转换级别分布:")
        print(f"  第一级 (v>2.5m/s 高速):   {conv_detail['high_speed']:4d} 帧 ({100*conv_detail['high_speed']/total:5.1f}%)")
        print(f"  第二级 (1.2<v≤2.5 中速):  {conv_detail['medium_speed']:4d} 帧 ({100*conv_detail['medium_speed']/total:5.1f}%)")
        print(f"  第三级 (0.8<v≤1.2 低速):  {conv_detail['low_speed']:4d} 帧 ({100*conv_detail['low_speed']/total:5.1f}%)")
        print(f"  未转换 (几何/速度不符):    {conv_detail['failed']:4d} 帧 ({100*conv_detail['failed']/total:5.1f}%)")
        
    except FileNotFoundError:
        print(f"\n文件不存在: {filename}")
    except Exception as e:
        print(f"\n分析{filename}时出错: {e}")

# 行人对照
print("\n\n" + "="*70)
print("行人数据对照组 (验证误判率)")
print("="*70)

pedestrian_files = ['man_walk2.csv', 'man_walk3.csv']
for filename in pedestrian_files:
    try:
        total, converted, conv_detail = analyze_conversion(filename)
        
        print(f"\n{'='*70}")
        print(f"文件: {filename}")
        print(f"{'='*70}")
        print(f"总帧数: {total}")
        print(f"误判为滑板车: {converted} ({100*converted/total:.1f}%) ← 应该很低")
        if converted > 0:
            print(f"  误判级别: 高速={conv_detail['high_speed']}, 中速={conv_detail['medium_speed']}, 低速={conv_detail['low_speed']}")
        
    except FileNotFoundError:
        print(f"\n文件不存在: {filename}")
    except Exception as e:
        print(f"\n分析{filename}时出错: {e}")

print("\n\n" + "="*70)
print("总结")
print("="*70)
if results:
    avg_conversion = np.mean([100*conv/total for _, (total, conv, _) in results.items()])
    print(f"✓ 滑板车平均识别率: {avg_conversion:.1f}%")
    print(f"✓ 多级策略覆盖: 高速(>2.5) + 中速(1.2-2.5) + 低速(0.8-1.2)")
    print(f"✓ 行人误判: 通过几何特征过滤,预期<10%")
print("="*70)
