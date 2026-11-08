#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Tìm chuỗi `axes` cho mpu_driver.py từ chiều lắp thật (chạy trên Pi, không cần ROS).

1. Xe đứng yên: trục chip đọc ~+1 g là trục hướng LÊN (gia tốc kế đo phản lực).
2. Xoay xe NGƯỢC chiều kim đồng hồ nhìn từ trên (~90°): trục gyro tích phân lớn
   nhất phải trùng trục hướng lên và dương. Hai bước lệch nhau -> lắp nghiêng/hỏng.
EKF xe thật chỉ dùng vyaw nên chỉ trục z của imu_link cần đúng; x/y được chọn
cho thành hệ tay phải, KHÔNG kiểm chứng hướng tiến.
Dùng: imu_mount_check.py [--address 0x68] [--rotate-seconds 6]
Ghi chú: có ghi PWR_MGMT_1/CONFIG/GYRO_CONFIG/ACCEL_CONFIG giống mpu_driver.py.
"""
import argparse
import math
import statistics
import struct
import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

from mpu_registers import (ACCEL_CONFIG, ACCEL_LSB, ACCEL_XOUT_H, CONFIG, G, GYRO_CONFIG,
                        GYRO_LSB, KNOWN_IDS, PWR_MGMT_1, WHO_AM_I, parse_axes)


def suggest(up_index, up_sign):
    """Chuỗi axes với z = trục hướng lên; x, y hoán vị vòng để giữ tay phải."""
    names = 'xyz'
    x, y = (up_index+1) % 3, (up_index+2) % 3
    # Úp ngược (lên = -k): quay 180° quanh x' -> đảo dấu cả y' và z'.
    ys = '' if up_sign > 0 else '-'
    zs = '' if up_sign > 0 else '-'
    text = f'{names[x]} {ys}{names[y]} {zs}{names[up_index]}'
    parse_axes(text)  # tự kiểm tra định thức +1
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bus', type=int, default=1)
    ap.add_argument('--address', type=lambda s: int(s, 0), default=0x68)
    ap.add_argument('--rotate-seconds', type=float, default=6.)
    a = ap.parse_args()
    bus = SMBus(a.bus)
    who = bus.read_byte_data(a.address, WHO_AM_I)
    print(f'WHO_AM_I=0x{who:02x} ({KNOWN_IDS.get(who, "UNKNOWN")})')
    for reg, value in ((PWR_MGMT_1, 0x01), (CONFIG, 3), (GYRO_CONFIG, 0), (ACCEL_CONFIG, 0)):
        bus.write_byte_data(a.address, reg, value)
    time.sleep(.2)

    def read():
        d = bytes(bus.read_i2c_block_data(a.address, ACCEL_XOUT_H, 14))
        ax, ay, az, _t, gx, gy, gz = struct.unpack('>hhhhhhh', d)
        return [v/ACCEL_LSB*G for v in (ax, ay, az)], [math.radians(v/GYRO_LSB) for v in (gx, gy, gz)]

    print('Bước 1: để xe ĐỨNG YÊN 3 s...')
    acc, gyr = [], []
    end = time.monotonic()+3.
    while time.monotonic() < end:
        x, w = read(); acc.append(x); gyr.append(w); time.sleep(.01)
    g = [statistics.fmean(v) for v in zip(*acc)]
    bias = [statistics.fmean(v) for v in zip(*gyr)]
    noise = max(statistics.pstdev(v) for v in zip(*gyr))
    k = max(range(3), key=lambda i: abs(g[i]))
    sign = 1 if g[k] > 0 else -1
    tilt = math.degrees(math.acos(min(1., abs(g[k])/math.sqrt(sum(v*v for v in g)))))
    print('  gia tốc chip [x y z] = [%.2f %.2f %.2f] m/s^2, |g| = %.2f' % (*g, math.sqrt(sum(v*v for v in g))))
    print('  gyro bias = [%.4f %.4f %.4f] rad/s, nhiễu std %.4f rad/s' % (*bias, noise))
    print(f'  trục hướng LÊN: {"+" if sign > 0 else "-"}{"xyz"[k]} (nghiêng {tilt:.1f}° so với thẳng đứng)')
    if noise > .02:
        print('  CẢNH BÁO: nhiễu gyro lớn - xe không đứng yên hoặc rung motor; đo lại')
    if tilt > 5.:
        print('  CẢNH BÁO: chip lắp nghiêng > 5°; vyaw sẽ lẫn một phần quay ngang')

    print(f'Bước 2: trong {a.rotate_seconds:.0f} s tới, XOAY XE NGƯỢC CHIỀU KIM ĐỒNG HỒ (nhìn từ trên) ~90° rồi dừng...')
    time.sleep(1.)
    ang = [0., 0., 0.]
    last = time.monotonic(); end = last+a.rotate_seconds
    while time.monotonic() < end:
        _, w = read(); now = time.monotonic(); dt = now-last; last = now
        ang = [s+(v-b)*dt for s, v, b in zip(ang, w, bias)]
        time.sleep(.005)
    j = max(range(3), key=lambda i: abs(ang[i]))
    print('  góc tích phân chip [x y z] = [%.1f %.1f %.1f]°' % tuple(math.degrees(v) for v in ang))
    if abs(math.degrees(ang[j])) < 30.:
        print('  KHÔNG ĐỦ: quay chưa tới 30°; chạy lại và xoay rõ hơn')
        return 2
    rot_sign = 1 if ang[j] > 0 else -1
    print(f'  trục quay (CCW dương): {"+" if rot_sign > 0 else "-"}{"xyz"[j]}, {abs(math.degrees(ang[j])):.0f}°')
    if j != k or rot_sign != sign:
        print('  LỖI: trục quay khác trục hướng lên -> kiểm tra lại chiều xoay hoặc cách lắp')
        return 3
    text = suggest(k, sign)
    print(f'\nĐặt cho mpu_driver.py:  axes: "{text}"')
    print('(chỉ trục z đã kiểm chứng; x/y chọn cho đúng tay phải, chưa đo hướng tiến)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
