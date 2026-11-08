# SPDX-License-Identifier: Apache-2.0
"""Thanh ghi MPU6050/MPU6500 và ánh xạ trục; không phụ thuộc ROS (dùng chung với
imu_mount_check.py chạy trực tiếp trên Pi)."""

# Thanh ghi chung MPU6050/MPU6500.
SMPLRT_DIV, CONFIG, GYRO_CONFIG, ACCEL_CONFIG = 0x19, 0x1A, 0x1B, 0x1C
ACCEL_XOUT_H, PWR_MGMT_1, WHO_AM_I = 0x3B, 0x6B, 0x75
# 0x68 MPU6050; 0x70 MPU6500 (module "GY-521" đo được 29/10/2026); 0x71/0x73
# MPU9250/9255 (dùng phần MPU6500 bên trong). Bản đồ thanh ghi gyro/accel giống nhau.
KNOWN_IDS = {0x68: 'MPU6050', 0x70: 'MPU6500', 0x71: 'MPU9250', 0x73: 'MPU9255'}
GYRO_LSB = 131.  # LSB/(°/s) ở ±250 °/s
ACCEL_LSB = 16384.  # LSB/g ở ±2 g
G = 9.80665


def parse_axes(text):
    """'-y x z' -> [(1, -1), (0, 1), (2, 1)]: (chỉ số trục chip, dấu) cho x,y,z."""
    items = text.split()
    if len(items) != 3:
        raise ValueError('axes needs three items, e.g. "x y z" or "-y x z"')
    out = []
    for item in items:
        sign = -1 if item.startswith('-') else 1
        name = item.lstrip('+-')
        if name not in ('x', 'y', 'z'):
            raise ValueError(f'invalid axis {item!r}')
        out.append(('xyz'.index(name), sign))
    if sorted(i for i, _ in out) != [0, 1, 2]:
        raise ValueError('axes must use x, y and z exactly once')
    # Ma trận ánh xạ phải là phép quay (định thức +1), không được là phép soi gương.
    m = [[0]*3 for _ in range(3)]
    for row, (col, sign) in enumerate(out):
        m[row][col] = sign
    det = (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
           + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))
    if det != 1:
        raise ValueError('axes must be a right-handed rotation (determinant +1)')
    return out


def remap(values, axes):
    return [sign*values[i] for i, sign in axes]
