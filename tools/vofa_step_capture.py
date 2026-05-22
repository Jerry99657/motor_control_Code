#!/usr/bin/env python3
"""
VOFA+ JustFloat 阶跃响应采集工具
用于采集电机速度阶跃响应数据，供PID调参分析
"""

import serial
import struct
import time
import csv
import sys

# ============ 配置 ============
PORT = "COM8"
BAUD = 115200
# JustFloat 帧尾
FRAME_TAIL = b'\x00\x00\x80\x7f'
# 采集参数
STEP_SPEED = 50       # 阶跃速度百分比
CAPTURE_TIME = 3.0    # 采集时长(秒)
# ================================

def find_frame(data: bytes):
    """在字节流中查找 JustFloat 帧并解码"""
    frames = []
    idx = 0
    while idx <= len(data) - 4:
        # 找帧尾
        tail_pos = data.find(FRAME_TAIL, idx)
        if tail_pos < 0:
            break
        # 帧尾前应有 8 个 float (32字节) = speed[4] + duty[4]
        payload_end = tail_pos
        if payload_end >= 32:
            frame_start = payload_end - 32
            try:
                vals = struct.unpack('<8f', data[frame_start:payload_end])
                frames.append(vals)
            except struct.error:
                pass
        idx = tail_pos + 4
    return frames


def main():
    print(f"[INFO] 连接 {PORT} @ {BAUD} ...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)

    # 1. 清空缓冲区 + 停止电机
    ser.reset_input_buffer()
    ser.write(b'STOP\r\n')
    time.sleep(1.0)
    ser.reset_input_buffer()
    print("[INFO] 电机已停止，缓冲区已清空")

    # 2. 发送阶跃指令
    cmd = f"M1:{STEP_SPEED}\r\n"
    print(f"[INFO] 发送阶跃指令: {cmd.strip()}")
    t_start = time.time()
    ser.write(cmd.encode())

    # 3. 采集数据
    raw_buf = b''
    records = []
    frame_count = 0
    print(f"[INFO] 采集 {CAPTURE_TIME}s ...")

    while (time.time() - t_start) < CAPTURE_TIME:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            raw_buf += chunk
            frames = find_frame(raw_buf)
            if frames:
                # 只取最后一帧，避免重复
                for f in frames:
                    t_rel = time.time() - t_start
                    records.append({
                        't': round(t_rel, 4),
                        'rpm1': round(f[0], 2),
                        'rpm2': round(f[1], 2),
                        'rpm3': round(f[2], 2),
                        'rpm4': round(f[3], 2),
                        'duty1': round(f[4], 2),
                        'duty2': round(f[5], 2),
                        'duty3': round(f[6], 2),
                        'duty4': round(f[7], 2),
                    })
                    frame_count += 1
                # 清除已解析的部分
                last_tail = raw_buf.rfind(FRAME_TAIL)
                if last_tail >= 0:
                    raw_buf = raw_buf[last_tail + 4:]

    # 4. 停止电机
    ser.write(b'STOP\r\n')
    time.sleep(0.1)
    ser.close()

    # 5. 保存 CSV
    out_path = "step_response.csv"
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys() if records else [])
        writer.writeheader()
        writer.writerows(records)

    print(f"\n[INFO] 采集完成! 共 {frame_count} 帧, 保存到 {out_path}")

    # 6. 打印摘要
    if records:
        print("\n========== 阶跃响应摘要 ==========")
        print(f"  阶跃指令: M1:{STEP_SPEED}%")
        print(f"  采集时长: {CAPTURE_TIME}s")
        print(f"  总帧数:   {frame_count}")
        print(f"  采样率:   ~{frame_count/CAPTURE_TIME:.0f} Hz")
        print()
        # 打印前10帧和后10帧
        print("  前10帧 (t, rpm1, duty1):")
        for r in records[:10]:
            print(f"    t={r['t']:.3f}s  rpm1={r['rpm1']:8.1f}  duty1={r['duty1']:6.1f}%")
        if len(records) > 20:
            print("    ...")
        print("  后10帧:")
        for r in records[-10:]:
            print(f"    t={r['t']:.3f}s  rpm1={r['rpm1']:8.1f}  duty1={r['duty1']:6.1f}%")

        # 关键指标
        rpms = [r['rpm1'] for r in records]
        duties = [r['duty1'] for r in records]
        rpm_target = max(rpms) if rpms else 0
        # 找上升时间 (10% -> 90%)
        rpm_10 = rpm_target * 0.1
        rpm_90 = rpm_target * 0.9
        t_10 = t_90 = None
        for r in records:
            if t_10 is None and r['rpm1'] >= rpm_10:
                t_10 = r['t']
            if t_90 is None and r['rpm1'] >= rpm_90:
                t_90 = r['t']
        # 超调量
        overshoot = 0
        if rpm_target > 0:
            rpm_max = max(rpms)
            overshoot = (rpm_max - rpm_target) / rpm_target * 100
        # 稳态值 (最后20%数据的平均值)
        steady_n = max(1, len(rpms) // 5)
        rpm_steady = sum(rpms[-steady_n:]) / steady_n
        duty_steady = sum(duties[-steady_n:]) / steady_n

        print(f"\n========== 性能指标 ==========")
        print(f"  稳态转速:     {rpm_steady:.1f} RPM")
        print(f"  稳态占空比:   {duty_steady:.1f}%")
        if t_10 and t_90:
            print(f"  上升时间:     {(t_90-t_10)*1000:.0f} ms (10%-90%)")
        print(f"  超调量:       {overshoot:.1f}%")
        print(f"  最大转速:     {max(rpms):.1f} RPM")


if __name__ == "__main__":
    main()
