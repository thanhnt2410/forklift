# P0 — Khảo sát mã nguồn upstream

Ngày thực hiện: 23/10/2026. Nguồn: `Trkkhrmn/ros2_amr_mecanumbot`,
commit `385b761c8c96757d2cade890c105f1dbfe187ed4`.

Đã clone đầy đủ từ GitHub, sau đó đưa vào workspace. Người dùng chuyển nội dung
repo ra thư mục gốc trong lúc triển khai; đường dẫn hiện tại là `src/<package>`,
`hardware/`, `docker/`, `docs/`. Không còn yêu cầu một repo lồng bên trong `src`.
Commit và SHA-256 các file đã khảo sát lưu trong
[`upstream_manifest.json`](../results/p0/upstream_manifest.json).

## Kết quả đọc mã thực tế

| File gốc | Phát hiện | Quyết định |
|---|---|---|
| `src/mecanum_description/urdf/mecanum_amr.urdf.xacro` | Bánh 75 mm bán kính; khung 0,8 × 0,6 m; plugin `libgazebo_ros_planar_move.so`; camera và ray sensor của Classic | Viết lại model theo số đo, thay hệ plugin, giữ ý tưởng bố trí xe nâng |
| Cùng file, `lift_joint` | Khớp prismatic hành trình 0,30 m, chưa phải thông số cơ cấu của người dùng | Dùng joint nâng nhưng đánh dấu hành trình P1 là giả định |
| `src/mecanum_description/config/lift_controllers.yaml` | Khai báo effort group controller | Dùng trajectory controller position cho SIM và hợp đồng REAL sau này |
| `src/mecanum_control/src/mecanum_drive_node.cpp` | Tự tính động học, xuất `/wheel_speeds`, tự publish odom; đọc bốn phần tử `joint_states.velocity` theo thứ tự | Thay bằng mecanum controller, tránh đọc encoder bằng thứ tự mảng và tránh odom trùng |
| `src/mecanum_hw_bridge/mecanum_hw_bridge/hw_bridge_node.py` | Gửi UART, có timeout lệnh; không thấy đường đọc telemetry encoder trong node được khảo sát | Không xem bridge này là hardware interface encoder hoàn chỉnh |
| `hardware/stm32/Core/motor_lift_stub.c` | Các hàm motor và lift là TODO/stub | ESP32/PID là công việc P2, chưa thực hiện ở P1 |
| `hardware/stm32/main.c` | Có khung watchdog, HAL/timer/UART còn cần tích hợp | Không nạp firmware này lên ESP32 |
| `hardware/docs/PROTOCOL.md` | Telemetry tùy chọn, không có framing/checksum/sequence bắt buộc | Thiết kế UART hai chiều ở P2/P3 |
| `src/mecanum_simulation/worlds/minimal.world` | Ground và light dùng primitive, không tải model mạng | Kế thừa và chuyển sang SDF world Fortress |
| `src/mecanum_simulation/worlds/warehouse.world` | World lớn, asset và material Classic | Giữ tham khảo; P1 dùng kho primitive tự chứa |
| `src/mecanum_bringup/launch/sim_bringup.launch.py` | Ghép SIM/control/navigation/task của Classic | Tạo launch mới cho Fortress, chỉ khởi chạy phạm vi P1 |

Các package `mecanum_*` còn nguyên để tham khảo và có `COLCON_IGNORE`, tránh
build/chạy nhầm plugin Classic hoặc node phát odometry cũ. Đây là lựa chọn kiến trúc,
không phải kết luận các package gốc đều không chạy được.

## Baseline Gazebo Fortress

Máy có `gz_ros2_control_demos` 0.7.18 và `mecanum_drive_controller` 2.52.1.
Demo được chạy headless bằng model/launch từ package cài sẵn, không chạy Classic.

Phát hiện cụ thể: demo hiện dùng `gz:expressed_in="base_link"` trong `fdir1`,
nhưng Physics của Fortress 6.17.1 trên máy nhận `ignition:expressed_in`.
Chạy nguyên trạng: tiến và xoay có tác dụng, đi ngang không có chuyển động vật lý.
Đổi namespace sang `ignition` trên bản sao demo, giữ nguyên các hướng ma sát:
cả tiến, ngang và xoay đạt. Không sửa file trong `/opt/ros`.

Xem [`baseline_unpatched.json`](../results/p0/baseline_unpatched.json),
[`baseline.json`](../results/p0/baseline.json) và log demo trong `results/p0/`.
Vị trí vật lý đo qua Gazebo `dynamic_pose/info`, tách biệt odometry controller.

## Giới hạn của P0

- Chưa đo cơ khí, encoder, điện và tải thực vì không có quyền truy cập robot vật lý.
- Chưa chạy upstream Classic; không ghi nhận video Classic như kết quả đã hoàn thành.
- Chưa kiểm chứng firmware trên MCU, Nav2 hoặc SLAM; các phần này ngoài P0/P1.
- Dependency baseline được khóa; việc lấy lại cùng commit không tự bao gồm bản sửa P1.
