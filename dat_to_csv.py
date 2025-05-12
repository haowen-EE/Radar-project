#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dat_to_csv_with_timestamp.py

Convert mmwave demo .dat file to CSV and add human-readable timestamp for each detection row.
"""

import re
import datetime
import numpy as np
import csv
import sys
from parser_mmw_demo import parser_one_mmw_demo_output_packet

def convert_dat_to_csv():
    # Input/output filenames (修改为实际文件名)
    dat_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397.dat'
    csv_file = 'xwr18xx_AOP_processed_stream_2025_05_12T04_57_17_397_with_timestamp2.csv'

    # Parse base timestamp from filename
    m = re.search(r'stream_(\d{4}_\d{2}_\d{2}T\d{2}_\d{2}_\d{2}_\d{3})', dat_file)
    if m:
        base_ts = datetime.datetime.strptime(
            m.group(1), '%Y_%m_%dT%H_%M_%S_%f'
        )
    else:
        base_ts = datetime.datetime.now()

    # Frame interval in milliseconds (请根据 cfg 文件中的 Frame Duration 调整)
    frame_interval_ms = 71.429  # ms
    frame_count = 0

    # Read .dat file
    try:
        with open(dat_file, 'rb') as f:
            data = bytearray(f.read())
    except FileNotFoundError:
        print(f"[ERROR] File not found: {dat_file}")
        sys.exit(1)

    byte_ptr = 0
    max_len = len(data)
    rows = []

    # Parse each packet/frame
    while byte_ptr < max_len:
        status, headerIdx, packetLen, numDetObj, numTlv, subFrame, \
        detectedX, detectedY, detectedZ, detectedV, \
        detectedRange, detectedAzimuth, detectedElevAngle, \
        detectedSNR, detectedNoise = parser_one_mmw_demo_output_packet(
            data[byte_ptr:], max_len - byte_ptr, debug=False
        )
        if status != 0 or packetLen <= 0:
            break

        # Compute timestamp for this frame
        frame_ts = base_ts + datetime.timedelta(milliseconds=frame_count * frame_interval_ms)
        frame_count += 1

        # Append each detected object row with timestamp
        for detIdx in range(numDetObj):
            rows.append({
                'timeStamp':  frame_ts.isoformat(sep=' ', timespec='milliseconds'),
                'subFrame':   subFrame,
                'detIdx':     detIdx,
                'x':          detectedX[detIdx],
                'y':          detectedY[detIdx],
                'z':          detectedZ[detIdx],
                'v':          detectedV[detIdx],
                'range':      detectedRange[detIdx],
                'azimuth':    detectedAzimuth[detIdx],
                'elevation':  detectedElevAngle[detIdx],
                'snr':        detectedSNR[detIdx],
                'noise':      detectedNoise[detIdx]
            })

        byte_ptr += packetLen

    # Write to CSV
    fieldnames = [
        'timeStamp', 'subFrame', 'detIdx', 'x', 'y', 'z', 'v',
        'range', 'azimuth', 'elevation', 'snr', 'noise'
    ]
    try:
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"[OK] Written {len(rows)} rows to {csv_file}")
    except Exception as e:
        print(f"[ERROR] CSV write failed: {e}")

if __name__ == '__main__':
    convert_dat_to_csv()
