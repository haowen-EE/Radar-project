#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dat_to_csv_fixed.py

将同目录下的 xwr18xx_AOP_processed_stream_2025_05_08T03_10_42_201.dat
转换为 CSV，输出到 xwr18xx_AOP_processed_stream_2025_05_08T03_10_42_201.csv。

每行格式：
    subFrame, detIdx, x, y, z, v, range, azimuth, elevation, snr, noise
"""

import numpy as np
import csv
import sys

# 如果 parser_mmw_demo.py 不在同一目录，请修改下面的导入路径
from parser_mmw_demo import parser_one_mmw_demo_output_packet

def convert_dat_to_csv():
    # ---- 固定路径，不再使用命令行参数 ----
    dat_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397.dat'
    csv_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397.csv'

    # 1. 读取为 bytearray，确保 parser_helper 能正常处理
    try:
        with open(dat_file, 'rb') as f:
            data = bytearray(f.read())
    except FileNotFoundError:
        print(f"[ERROR] 未找到文件: {dat_file}")
        sys.exit(1)

    byte_ptr = 0
    max_len  = len(data)
    rows     = []

    # 2. 循环解析每一帧
    while byte_ptr < max_len:
        status, headerIdx, packetLen, numDetObj, numTlv, subFrame, \
        detectedX, detectedY, detectedZ, detectedV, \
        detectedRange, detectedAzimuth, detectedElevAngle, \
        detectedSNR, detectedNoise = parser_one_mmw_demo_output_packet(
            data[byte_ptr:], max_len - byte_ptr, debug=False
        )

        # 解析失败或无有效长度则跳出
        if status != 0 or packetLen <= 0:
            break

        # 3. 将本帧每个目标写入 rows
        for detIdx in range(numDetObj):
            rows.append({
                'subFrame': subFrame,
                'detIdx':   detIdx,
                'x':        detectedX[detIdx],
                'y':        detectedY[detIdx],
                'z':        detectedZ[detIdx],
                'v':        detectedV[detIdx],
                'range':    detectedRange[detIdx],
                'azimuth':  detectedAzimuth[detIdx],
                'elevation':detectedElevAngle[detIdx],
                'snr':      detectedSNR[detIdx],
                'noise':    detectedNoise[detIdx]
            })

        # 跳过已处理的字节，进入下一帧
        byte_ptr += packetLen

    # 4. 写 CSV
    if not rows:
        print("[WARN] 未解析到任何目标数据，请检查 .dat 文件或配置。")
        return

    fieldnames = [
        'subFrame', 'detIdx',
        'x', 'y', 'z', 'v',
        'range', 'azimuth', 'elevation',
        'snr', 'noise'
    ]
    try:
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"[OK] 已将 {len(rows)} 条记录写入 {csv_file}")
    except Exception as e:
        print(f"[ERROR] 写入 CSV 失败：{e}")

if __name__ == '__main__':
    convert_dat_to_csv()
