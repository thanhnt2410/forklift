# P0/P1 — Ma trận kế thừa

Nguồn AMR: commit `385b761c8c96757d2cade890c105f1dbfe187ed4` (MIT).

| Nguồn | Đích hiện tại | Mức kế thừa và thay đổi |
|---|---|---|
| `mecanum_description/urdf/mecanum_amr.urdf.xacro` | `forklift_description/urdf/*.xacro` | Tham khảo khung xe nâng, LiDAR và khớp nâng; tái dựng hình học, inertia và interface theo kích thước mới |
| `mecanum_simulation/worlds/minimal.world` | `forklift_gazebo/worlds/minimal.sdf` | Kế thừa ground/light/scene; đổi physics/system plugin, đưa mặt đất về z=0, thêm tường |
| World minimal đã chuyển đổi | `forklift_gazebo/worlds/warehouse.sdf` | Thêm kệ và trạm bằng primitive tự chứa; không mang asset QR/camera chưa dùng |
| `mecanum_bringup/launch/sim_bringup.launch.py` | `forklift_bringup/launch/sim.launch.py` | Tham khảo vai trò bringup; viết launch mới với spawn/controller sequencing và kiểm tra lỗi |
| `mecanum_control/src/mecanum_drive_node.cpp` | Controller chuẩn và `forklift_control/scripts/command_gate.py` | Không dùng lại thuật toán/node; controller chuẩn tính động học, gate chọn nguồn và giới hạn |
| Bridge Jetson/STM32 và firmware | Giữ `mecanum_hw_bridge/`, `hardware/` tham khảo | Không chạy trong P1; ESP32 interface phát triển ở P2/P3 |
| Nav2, SLAM, task manager upstream | Giữ các package `mecanum_*` | Khảo sát/tích hợp ở P4/P6, chưa công bố hoạt động ở bản P1 |
| OSRF `gz_ros2_control_demos/urdf/test_mecanum_drive.xacro.urdf` 0.7.18 | `forklift_description/urdf/forklift_cad.urdf.xacro` | Kế thừa collision cầu và ma sát dị hướng; sửa namespace Fortress, thu nhỏ theo kích thước xe |

Các file khác trong `forklift_*` và công cụ kiểm thử được viết cho dự án này.
Xem `THIRD_PARTY_NOTICES.md` để biết license và nguồn chi tiết.
