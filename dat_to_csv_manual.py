#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dat_to_csv_manual.py
------------------------------------------------------------
用途：不使用命令行参数/PowerShell，直接在代码里手动填写 .dat 和 .cfg 路径，批量生成 CSV。

使用步骤：
1) 打开本文件，找到 “【用户配置区】” 把 PAIRS 列表改成你的 .dat/.cfg 路径对；可填多对。
   - 建议用 Windows 原始字符串：r"C:\path\to\file.dat"（反斜杠不用转义）
   - 每个 .dat 旁边填上对应 .cfg；若确实没有 .cfg，会用默认 50ms 帧周期
2) 如需统一输出目录，设置 OUTDIR 变量为一个存在/不存在均可的路径；留空(None)则输出到 .dat 同目录。
3) 保存后，直接双击运行 或 在命令行运行：python dat_to_csv_manual.py

输出列：
timeStamp, frameNumber, subFrame, detIdx,
x, y, z, v, range, azimuth_deg, elevation_deg, snr, noise
------------------------------------------------------------
"""

import re
import sys
import csv
import math
import datetime
from pathlib import Path
import struct
from typing import List, Tuple, Optional

# =========================【用户配置区】=========================
# 在这里手动填写 .dat 与 .cfg 的路径对（可以填多对）。
# 示例已给出（请按需修改为你本机的路径）。
PAIRS: List[Tuple[str, str]] = [
    # 例子（把下方路径换成你的实际路径；注意使用 r"..." 原始字符串以避免反斜杠转义）
    (r"C:\Users\h\OneDrive\桌面\data\escooter_slow_without_RSC.dat",
     r"C:\Users\h\OneDrive\桌面\data\escooter_slow_without_RSC.cfg"),
    # (r"C:\...\keep_walking.dat", r"C:\...\keep_walking.cfg"),
    # (r"C:\...\old_man_pass.dat", r"C:\...\old_man_pass.cfg"),
    # (r"C:\...\twopeople_parallel.dat", r"C:\...\twopeople_parallel.cfg"),
]

# 可选：统一输出目录（None 表示输出到各自 .dat 的同目录下）
OUTDIR: Optional[str] = None
# 如果你希望程序结束后窗口停留，便于查看日志，请保持 True；
# 如果你在终端中运行且不想暂停，改为 False。
PAUSE_AT_END: bool = True
# ============================================================


# -------------------- 以下为解析与转换实现 --------------------
MAGIC = bytes([2,1,4,3,6,5,8,7])  # 0x0201040306050807 (TI mmWave demo packet magic)


def parse_cfg_frame_ms(cfg_path: Path) -> float:
    """从 .cfg 中解析帧周期（毫秒）。优先读 Visualizer 的注释 'Frame Duration(msec):'，
    若没有则回退解析 'frameCfg' 行的第 5 个参数（framePeriodicity(msec)）。"""
    try:
        text = cfg_path.read_text(errors='ignore')
    except Exception:
        # 缺 .cfg 或不可读则默认 50 ms
        return 50.0

    m = re.search(r'Frame Duration\(msec\):\s*([0-9.]+)', text)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            pass

    m2 = re.search(r'^\s*frameCfg\s+([^\r\n]+)', text, flags=re.MULTILINE)
    if m2:
        parts = m2.group(1).split()
        if len(parts) >= 5:
            try:
                return float(parts[4])
            except Exception:
                pass

    return 50.0  # 兜底


def iter_frames(blob: bytes):
    """遍历 .dat 中的帧。返回 (frame_start_offset, header_dict)。"""
    i = 0
    n = len(blob)
    while i < n:
        j = blob.find(MAGIC, i)
        if j < 0 or (j + 8 + 40) > n:
            break
        # 帧头（紧跟 magic 之后）：10 * uint32（40字节）
        try:
            ver, total_len, platform, frame_num, time_cycles, num_obj_hdr, num_tlvs, subframe, r0, r1 = struct.unpack(
                "<10I", blob[j+8 : j+8+40]
            )
        except Exception:
            i = j + 1
            continue

        # 长度校验
        if total_len <= 0 or (j + total_len) > n:
            i = j + 1
            continue

        header = {
            "version": ver,
            "totalLen": total_len,
            "platform": platform,
            "frameNumber": frame_num,
            "timeCpuCycles": time_cycles,
            "numDetectedObj_hdr": num_obj_hdr,
            "numTLVs": num_tlvs,
            "subFrameNumber": subframe,
        }
        yield j, header
        i = j + total_len


def parse_tlvs_in_frame(blob: bytes, frame_start: int, header: dict):
    """解析一帧中的 TLV：
       - type 1：点云（float） -> [(x,y,z,v), ...]
       - type 7：sideInfo（int16）-> [(snr, noise), ...]
    """
    ptr = frame_start + 0x28  # SDK 3.x/Visualizer 3.6+：TLV 起点
    end = frame_start + header["totalLen"]
    pts = []
    side = []
    while ptr + 8 <= end:
        try:
            tlv_type, tlv_len = struct.unpack("<II", blob[ptr:ptr+8])
        except Exception:
            break

        payload = blob[ptr+8 : ptr+8+tlv_len]
        if tlv_type == 1 and tlv_len >= 16 and (tlv_len % 16 == 0):
            N = tlv_len // 16
            try:
                floats = struct.unpack("<" + "f"*(4*N), payload)
                for i in range(N):
                    x, y, z, v = floats[4*i : 4*i+4]
                    pts.append((x, y, z, v))
            except Exception:
                pass
        elif tlv_type == 7 and tlv_len >= 4 and (tlv_len % 4 == 0):
            try:
                vals = struct.unpack("<" + "hh"*(tlv_len//4), payload)
                pairs = [(vals[2*i], vals[2*i+1]) for i in range(tlv_len//4)]
                side.extend(pairs)
            except Exception:
                pass

        ptr += 8 + tlv_len
    return pts, side


def derive_angles(x: float, y: float, z: float):
    rng = math.sqrt(x*x + y*y + z*z)
    az = math.degrees(math.atan2(y, x))
    el = math.degrees(math.atan2(z, math.sqrt(x*x + y*y)))
    return rng, az, el


def convert_one(dat_path: Path, cfg_path: Optional[Path], csv_out: Optional[Path] = None) -> Path:
    """把单个 .dat（配对 .cfg）转换为 CSV。"""
    blob = dat_path.read_bytes()
    frames = list(iter_frames(blob))
    frame_ms = parse_cfg_frame_ms(cfg_path) if cfg_path else 50.0

    # 以文件名中形如 2025_05_12T04_57_17_397 的时间作为基准；否则用当前时间
    m = re.search(r"(\\d{4}_\\d{2}_\\d{2}T\\d{2}_\\d{2}_\\d{2}_\\d{3})", dat_path.name)
    if m:
        try:
            base_ts = datetime.datetime.strptime(m.group(1), "%Y_%m_%dT%H_%M_%S_%f")
        except Exception:
            base_ts = datetime.datetime.now()
    else:
        base_ts = datetime.datetime.now()

    rows = []
    for idx, (start, header) in enumerate(frames):
        pts, side = parse_tlvs_in_frame(blob, start, header)
        ts = base_ts + datetime.timedelta(milliseconds=idx * frame_ms)
        for i, (x, y, z, v) in enumerate(pts):
            snr = noise = None
            if i < len(side):
                snr, noise = side[i]
            rng, az, el = derive_angles(x, y, z)
            rows.append({
                "timeStamp": ts.isoformat(sep=" ", timespec="milliseconds"),
                "frameNumber": header["frameNumber"],
                "subFrame": header["subFrameNumber"],
                "detIdx": i,
                "x": x, "y": y, "z": z, "v": v,
                "range": rng, "azimuth_deg": az, "elevation_deg": el,
                "snr": snr, "noise": noise,
            })

    if csv_out is None:
        csv_out = dat_path.with_suffix(".csv")

    # 写 CSV
    csv_out.parent.mkdir(parents=True, exist_ok=True)
    with csv_out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "timeStamp", "frameNumber", "subFrame", "detIdx",
            "x", "y", "z", "v", "range", "azimuth_deg", "elevation_deg",
            "snr", "noise"
        ])
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    return csv_out


def main():
    # 1) 检查配置
    if not PAIRS:
        print("请先在代码顶部的 PAIRS 列表中填入 (.dat, .cfg) 路径对，然后再运行。")
        if PAUSE_AT_END:
            input("按回车键退出...")
        return

    outdir = Path(OUTDIR) if OUTDIR else None

    # 2) 逐对处理
    ok = 0
    for idx, (dat_str, cfg_str) in enumerate(PAIRS, 1):
        try:
            dat_p = Path(dat_str)
            cfg_p = Path(cfg_str) if cfg_str else None
            if not dat_p.exists():
                print(f"[{idx}] ❌ 未找到 .dat：{dat_p}")
                continue
            if cfg_p and not cfg_p.exists():
                print(f"[{idx}] ⚠️  未找到 .cfg：{cfg_p}（将使用默认 50 ms 帧周期）")
                cfg_p = None

            out_path = (outdir / (dat_p.stem + ".csv")) if outdir else dat_p.with_suffix(".csv")
            csv_path = convert_one(dat_p, cfg_p, out_path)
            print(f"[{idx}] ✅ {dat_p.name} -> {csv_path}")
            ok += 1
        except Exception as e:
            print(f"[{idx}] ❌ 转换失败：{dat_str}  错误：{e}")

    print(f"\n完成：成功 {ok} / 共 {len(PAIRS)}")
    if PAUSE_AT_END:
        input("按回车键退出...")


if __name__ == "__main__":
    main()
