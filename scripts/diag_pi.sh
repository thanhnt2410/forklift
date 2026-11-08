---
name: pi-terms-cong-serial-va-vai-tro-pi-desktop
description: Pi 4B (192.168.100.205) chay toan bo xu ly cam bien/xe, Desktop chi RViz/teleop; cong serial gan theo by-path socket vat ly vi CP2102 trung serial
metadata:
  type: project
---

Pi 4B `thainguyen1233@192.168.100.205` (pass 02012004, macOS `~/ros_ws/forklift`) la may duy nhihat chay ca cam bien va xe.
Desktop (Ubuntu PC hien tai) chi chay hien thi va teleop.

**Why:** Cac node mo cong serial vat ly nen phai chay tren may cam hardware; RViz/teleop/node Python thuan mem chay o dau cung duoc
mi la cung ROS_DOMAIN_ID. Loi `MCU_FAULT_8` (encoder) va `message filter ... queue is full` (SLAM) xuat hien cung luc 2026-10-28.

**How to apply:** Khi dinh danh loi ve ESP32/LiDAR/SLAM, doi chieu cong theo socket by-path chu khong theo by-id
(ESP32 = `...usb-0:1.3:1.0-port0`, LiDAR = `...usb-0:1.4:1.0-port0`) vi ca hai cung la CP2102 serial 0001 nen by-id trung ten.
Script chan doan tong hop: `scripts/diag_pi.sh` (do cong serial, telemetry tho, tan so /scan, tai he thong).
Lien quan [[forklift-firmware-fault-va-warning]].
