#!/usr/bin/env python3
"""
解析 MCP serial 返回的 JSON 数据中的 VOFA+ JustFloat 帧
"""
import json
import struct
import re
import sys

FRAME_TAIL = b'\x00\x00\x80\x7f'

def decode_mcp_string(s: str) -> bytes:
    """将 MCP 返回的 Unicode 转义字符串解码为原始字节"""
    # 先处理标准的 unicode 转义
    s = s.replace('\\n', '\n')  # 保留 \n 为换行
    # 将字符串编码为 bytes，然后解码 unicode 转义
    try:
        raw = s.encode('utf-8').decode('unicode_escape').encode('latin-1')
    except:
        raw = s.encode('latin-1', errors='replace')
    return raw


def parse_justfloat_frames(data: bytes):
    """从字节流中提取 JustFloat 帧"""
    frames = []
    idx = 0
    while idx <= len(data) - 4:
        tail_pos = data.find(FRAME_TAIL, idx)
        if tail_pos < 0:
            break
        if tail_pos >= 32:
            frame_start = tail_pos - 32
            try:
                vals = struct.unpack('<8f', data[frame_start:tail_pos])
                frames.append(vals)
            except struct.error:
                pass
        idx = tail_pos + 4
    return frames


def main():
    if len(sys.argv) < 2:
        print("用法: python parse_mcp_data.py <content.json>")
        return

    filepath = sys.argv[1]
    with open(filepath, 'r', encoding='utf-8') as f:
        content = json.load(f)

    raw_str = content.get('result', '')
    
    # 按行分割，每行是一个 "[下位机] ..." 条目
    lines = raw_str.split('\n')
    
    all_frames = []
    for line in lines:
        # 去掉前缀
        if '[下位机]' in line:
            data_part = line.split('[下位机]')[-1].strip()
        elif '[用户]' in line:
            continue  # 跳过用户命令行
        else:
            data_part = line.strip()
        
        if not data_part:
            continue
        
        # 解码转义字符串为字节
        raw_bytes = decode_mcp_string(data_part)
        
        # 解析 JustFloat 帧
        frames = parse_justfloat_frames(raw_bytes)
        all_frames.extend(frames)
    
    if not all_frames:
        print("[WARN] 没有找到有效的 JustFloat 帧!")
        return

    print(f"[INFO] 共解析 {len(all_frames)} 帧")
    print(f"\n{'='*70}")
    print(f"{'帧号':>5} | {'speed1':>8} {'speed2':>8} {'speed3':>8} {'speed4':>8} | {'duty1':>7} {'duty2':>7} {'duty3':>7} {'duty4':>7}")
    print(f"{'-'*70}")
    
    for i, f in enumerate(all_frames):
        print(f"{i:5d} | {f[0]:8.1f} {f[1]:8.1f} {f[2]:8.1f} {f[3]:8.1f} | {f[4]:7.1f} {f[5]:7.1f} {f[6]:7.1f} {f[7]:7.1f}")
    
    # 统计分析
    speed1_vals = [f[0] for f in all_frames]
    duty1_vals = [f[4] for f in all_frames]
    
    # 过滤非零值
    nonzero_speeds = [s for s in speed1_vals if abs(s) > 0.5]
    
    print(f"\n{'='*70}")
    print(f"M1 速度统计:")
    print(f"  总帧数:        {len(all_frames)}")
    print(f"  非零帧数:      {len(nonzero_speeds)}")
    if nonzero_speeds:
        print(f"  非零速度范围:  [{min(nonzero_speeds):.1f}, {max(nonzero_speeds):.1f}] RPM")
        print(f"  非零速度均值:  {sum(nonzero_speeds)/len(nonzero_speeds):.1f} RPM")
    print(f"  duty1 范围:    [{min(duty1_vals):.1f}, {max(duty1_vals):.1f}]")


if __name__ == '__main__':
    main()
