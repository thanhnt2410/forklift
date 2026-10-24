# Xe nâng AMR mecanum — ROS 2 Humble / Gazebo Fortress

> Điều hướng hiện tại chạy **path đã duyệt**, cho phép lùi theo cung trên toàn hành trình.
> Đoạn cuối thẳng vào trục pallet: A 1,30 m, B 0,90 m; rút tối thiểu 0,35 m để thoát trạm,
> sau đó tiếp tục lùi theo tuyến. Giới hạn tiến/lùi giữ 0,25/0,16 m/s.
> [Cấu hình và luồng nhiệm vụ](docs/curved_navigation.md),
> [path và quỹ đạo Gazebo](results/approved_route_sim/verified_demo_v2/route_simulation.png),
> [video](results/approved_route_sim/verified_demo_v2/mission_demo.mp4).

**Triển khai sau phê duyệt — 24/10/2026 (SIM):** chu trình lấy → mang A–B → thả → Home
đạt [29/29 kiểm tra](results/approved_route_sim/verified_demo_v2/summary.json).
Ba tuyến Nav2 dùng FollowPath với các đỉnh đã duyệt, kiểm tra costmap trước mỗi chặng;
LiDAR vẫn căn chỉnh/chèn càng. Nav2 cập nhật footprint khi mang/thả tải.
Thời gian khoảng 186 giây, RTF 98,9%; Home hướng -75° như bản xem trước.
60 unit Python và 5 kiểm tra C++ đạt. [Bằng chứng và giới hạn](results/approved_route_sim/README.md).
Hồi quy dừng: [hủy khi đang lùi 9/9](results/approved_route_sim/cancel_reverse/summary.json),
[mất LiDAR 10/10](results/approved_route_sim/scan_loss/summary.json); cả ba lượt cuối đã dọn sạch tiến trình.
Kết quả các cấu hình cũ bên dưới được giữ làm lịch sử, không cộng gộp thành nghiệm thu nhiều chu trình.

> Cập nhật CAD URDF20: mô hình mặc định đã đổi sang kích thước CAD.
> Xem [tích hợp URDF20](docs/forklift_model.md). Kết quả kiểm thử cũ bên dưới thuộc mô hình trước khi đổi CAD.

Đồ án phát triển từ [Trkkhrmn/ros2_amr_mecanumbot](https://github.com/Trkkhrmn/ros2_amr_mecanumbot), baseline commit `385b761c8c96757d2cade890c105f1dbfe187ed4`.

Nội dung repo đã được chuyển ra thư mục gốc workspace theo thao tác của người dùng. Các package nằm trực tiếp tại `src/forklift_*`. Package `mecanum_*` gốc được giữ đối chiếu và có `COLCON_IGNORE`, tránh build/chạy nhầm plugin Classic. [README gốc](docs/upstream_README.md).

Mô hình CAD, mesh và thông số xe được quản lý thống nhất trong
[`forklift_description`](src/forklift_description); không cần package `urdf20` riêng.

**Phạm vi hiện tại chỉ mô phỏng:** P3 có hardware interface + MCU giả qua PTY;
P4 có SLAM, bản đồ lưu, AMCL omni và Nav2. Không nạp firmware hoặc mở cổng xe thật.
[Hướng dẫn P3/P4](docs/p3_p4_simulation.md), [báo cáo kiểm thử](docs/p3_p4_status.md).

**Đã bổ sung EKF + IMU theo giao diện MPU6050 trong Gazebo:** `/wheel/odometry` +
`/imu/data_raw` → EKF → `/odom`. Mặc định bật, chỉ EKF phát `odom → base_link`.
[Thiết kế, kiểm thử và giới hạn MPU6050](docs/ekf_mpu6050.md).
Nav2 với EKF đạt 38/38 kiểm tra, gồm 10/10 đích và dừng khi mất IMU.

## P5 — nâng có giám sát (SIM)

Đã có homing dò chậm, action move/home, feedback/cancel/timeout, công tắc ảo,
watchdog và E-stop. Chưa homing/fault/đang nâng hoặc càng cao thì xe bị khóa.
[Hướng dẫn P5 và kết quả kiểm tra](docs/p5_lift.md).

```bash
ros2 launch forklift_bringup p5.launch.py
# Terminal khác đã source workspace:
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}' --feedback
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.10}' --feedback
```

Navigation/mapping mặc định dùng supervisor mới, auto-home một lần trong SIM.
`/lift/target` thuộc helper P1 cũ; trong P5 dùng action `/lift/move`.
Đã bổ sung P6-SIM lấy/giao pallet mẫu bằng tiếp xúc vật lý; chưa thử nâng thật.

## P6 — lấy/giao pallet tại trạm cố định (SIM)

Kiện cao 30 cm để LiDAR quét mặt hàng; docking đưa càng vào pallet động riêng,
nâng thấp bằng tiếp xúc vật lý rồi vận chuyển A→B→Home. Task manager có
cancel/fault, xác nhận tiếp xúc tải và mux Nav2/docking qua các khóa P4/P5.
[Hướng dẫn, kiểm thử và giới hạn P6](docs/p6_simulation.md).
Đã đạt **10/10 chu trình** và **11/11 tình huống lỗi** trong SIM.
[Video P6](results/p6/demo/mission_demo.mp4), [kết quả tổng hợp](results/p6/summary.json).

```bash
ros2 launch forklift_bringup p6.launch.py
# Terminal khác đã source workspace, chờ home/navigation ready:
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission \
  '{mission: a_to_b, timeout: 300.0}' --feedback
```

## Phạm vi P1

- Bánh đường kính **100 mm**, khoảng cách tâm–tâm ngang **260 mm**, dọc **195 mm**.
- Mô hình xe, hai càng và joint nâng; khung, khối lượng và hành trình dùng giá trị tạm cho SIM.
- gz_ros2_control, mecanum controller, joint state broadcaster và trajectory controller.
- Di chuyển tiến/lùi/ngang/xoay/kết hợp; chọn nguồn teleop/Nav2, giới hạn và timeout.
- LiDAR GPU 360 tia, `/scan`, `/clock`, `/odom`, joint states và TF.
- World `minimal` và `warehouse` tự chứa, không tải asset mạng khi chạy.
- Nâng theo mét, hủy nâng, khóa di chuyển khi nâng/hạ hoặc càng cao.

Chưa có firmware ESP32, triển khai REAL hoặc homing thực. P6 chỉ dùng pallet/kiện mẫu ở trạm cố định; chưa tự nhận dạng/lấy hàng bất kỳ.

## Build

Đã thử trên Ubuntu 22.04.5, ROS 2 Humble, Fortress 6.17.1. [Môi trường](docs/environment.md), [phiên bản package](results/p0/packages.tsv).

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Khi chuyển máy, kiểm tra dependency bằng rosdep; giữ `COLCON_IGNORE` của các package Classic khi build Fortress.

## Chạy P3/P4

MCU giả độc lập, SLAM hoặc tự hành trên map kho đã lưu:

```bash
# Chọn một chế độ trong cùng ROS domain:
ros2 launch forklift_hardware mock_hardware.launch.py
ros2 launch forklift_bringup mapping.launch.py
ros2 launch forklift_bringup navigation.launch.py
```

Hai lệnh bringup P4 mở Gazebo + RViz. Khi navigation đã active và
`/navigation/status` báo `READY`, chọn **Nav2 Goal** trên RViz.
Map tạo từ LiDAR SLAM tại `src/forklift_navigation/maps/warehouse.yaml`.
Chi tiết lưu map, gửi goal, khôi phục lỗi scan/TF và các giới hạn ở
[hướng dẫn P3/P4](docs/p3_p4_simulation.md).

## Chạy mô phỏng

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch forklift_bringup sim.launch.py
```

Mặc định world `minimal`, có Gazebo GUI và RViz2. Chạy kho hoặc tắt cửa sổ:

```bash
ros2 launch forklift_bringup sim.launch.py world:=warehouse
# Hoặc:
ros2 launch forklift_bringup sim.launch.py gui:=false rviz:=false
```

GPU LiDAR vẫn cần renderer OpenGL/EGL hoặc display phù hợp khi tắt GUI. Không chạy hai bringup trong cùng ROS domain/Gazebo partition. Ctrl+C tại terminal launch để dừng phiên.

## Điều khiển di chuyển

Với `sim.launch.py` mặc định, homing càng trước khi lái (xe bị khóa khi UNHOMED):

```bash
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}'
```

Mở terminal thứ hai, source lại môi trường và chạy:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel_teleop -p speed:=0.15 -p turn:=0.4
```

Theo bảng phím trên terminal: phím thường để tiến/lùi/quay, giữ **Shift** để dùng nhóm phím holonomic đi ngang. Mặc định chỉ nhận teleop. Lệnh không cập nhật quá 0,3 giây được đưa về zero; giữ phím hoặc tăng tốc độ lặp bàn phím nếu xe ngắt quãng.

Ví dụ đi ngang trái; Ctrl+C để ngừng phát:

```bash
ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist '{linear: {y: 0.08}}'
```

`/cmd_vel` là đầu ra của command gate. Gửi vào `/cmd_vel_teleop` hoặc `/cmd_vel_nav`, tránh tạo publisher thứ hai trên đầu ra.

Cho phép nhận nguồn Nav2 khi tích hợp sau này; teleop có ưu tiên khi đang phát:

```bash
ros2 service call /command_gate/set_autonomous std_srvs/srv/SetBool '{data: true}'
```

Lệnh này không khởi chạy Nav2. Đổi lại thủ công bằng `{data: false}`. Đổi chế độ xóa lệnh cũ và yêu cầu lệnh mới.

## Càng nâng SIM

Nâng 10 cm, hạ về mốc vận chuyển hoặc hủy nâng:

```bash
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}'
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.10}' --feedback
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.0}' --feedback
ros2 service call /lift/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /lift/status
```

P5 dùng hành trình **0–0,24 m từ mốc homing**. Xe bị khóa khi chưa home, đang nâng/hạ, fault, phản hồi quá hạn hoặc càng raw cao hơn 0,02 m. Chỉ dùng action supervisor; `/lift_controller/*` là giao diện nội bộ. Miền 0–0,25 m và `/lift/target` chỉ còn ở helper P1 khi chạy `lift_mode:=legacy`. Xem [P5](docs/p5_lift.md).

## Dừng phần mềm trong SIM

```bash
ros2 service call /command_gate/estop std_srvs/srv/SetBool '{data: true}'
```

Nhả bằng `{data: false}`. Với P5, reset I/O và supervisor rồi homing lại theo [hướng dẫn phục hồi](docs/p5_lift.md); sau đó mới phát lệnh mới. Xe không phát lại lệnh cũ. Chức năng SIM này không thay thế nút E-stop và mạch giữ tải trên robot thật.

## Kiểm thử

Các bài thử tạo ROS domain/partition riêng, kiểm tra rồi tự dọn tiến trình:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_checks.py baseline
python3 scripts/run_checks.py p1 --output results/p1_final
python3 scripts/run_checks.py p1 --world warehouse --output results/p1_warehouse
MPLCONFIGDIR=/tmp/forklift-matplotlib python3 -s scripts/plot_results.py results/p1_final
# P3 và P4:
python3 scripts/run_mock_checks.py
python3 scripts/run_p4_checks.py navigation --output results/p4_final
python3 -s -m pytest -q tests/test_navigation_guard.py
```

Đòi hỏi DDS/Gazebo mở được socket cục bộ; nếu sandbox chặn socket, chạy terminal ROS thông thường. Không kết nối MCU.

Kết quả: `checks.json`, `test.log`, `telemetry.csv` và `simulation_results.png`. Vị trí vật lý Gazebo chỉ vào `/test/poses` khi thử; không cấp vào `/odom` hoặc TF điều khiển.

Hai điểm tương thích đã xử lý: dùng `ignition:expressed_in` cho hướng ma sát Fortress; thêm khoảng đệm 5 mm ở giới hạn cơ học nâng SIM để tránh DART khóa joint khi trọng lực làm vượt giới hạn. Miền lệnh vẫn 0–0,25 m.

[Báo cáo P0/P1](docs/p0_p1_status.md) ghi kết quả và phần chưa xác minh.
Kết quả P3 ở `results/p3/`; bản đồ và khảo sát ở `results/p4/`;
nghiệm thu Nav2 ở `results/p4_final/`.

## Tài liệu

- [PLAN.md](PLAN.md): kế hoạch và trạng thái các giai đoạn.
- [Khảo sát upstream](docs/upstream_audit.md), [ma trận kế thừa](docs/reuse_matrix.md).
- [Thông số đo/còn thiếu](config/robot_measurements.yaml).
- [Hình học](src/forklift_description/config/robot.yaml): dùng chung cho URDF, động học và command gate khi launch; thông số tạm được đánh dấu.
- [Controller](src/forklift_control/config/controllers.yaml), [giới hạn lệnh](src/forklift_control/config/command_gate.yaml).
- [Nguồn và license](THIRD_PARTY_NOTICES.md): MIT cho phần upstream; Apache-2.0 cho package mới, license đi kèm từng package.
