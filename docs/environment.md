# Môi trường P0/P1

Đối chiếu ngày 23/10/2026, máy phát triển x86_64 Ubuntu 22.04.5 LTS.
ROS 2 Humble ở `/opt/ros/humble`; Python 3.10, CMake/ament/colcon.
Gazebo **Fortress / Ignition Gazebo 6.17.1**, physics DART (ign-physics 5.4.0).
Không chuyển sang Gazebo Classic hoặc Harmonic.

Phiên bản Debian đầy đủ: [`results/p0/packages.tsv`](../results/p0/packages.tsv).
Baseline nguồn AMR: [`dependencies.repos`](../dependencies.repos).

Các thành phần chính đã có sẵn: gz_ros2_control 0.7.18,
mecanum_drive_controller 2.52.1, ros2_control 2.53.1,
ros_gz_sim/bridge 0.244.23, joint_state_broadcaster và
joint_trajectory_controller 2.52.1. Không cài thêm package hệ thống cho P0/P1.

Gazebo 6 dùng executable `ign gazebo`. Namespace transport/message ở bridge là
`ignition.msgs.*`; tên ROS package vẫn là `gz_ros2_control`, `ros_gz_*`.
Không thay tên plugin tùy ý dựa vào chữ `gz`/`ign` trong tên executable.

Trong sandbox, ROS DDS và Gazebo transport báo lỗi socket `Operation not permitted`.
Các bài mô phỏng thực tế được chạy bên ngoài sandbox với ROS domain/Gazebo
partition riêng. Log đặt dưới `/tmp` hoặc thư mục kết quả; không cần sửa `.bashrc`.

LiDAR GPU cần OpenGL/EGL hoặc display có renderer. Phiên kiểm thử sử dụng display
`:1`; có cảnh báo EGL nhưng scan vẫn được xác minh có dữ liệu hữu hạn. Trên máy
khác, `gui:=false` chỉ tắt cửa sổ, không có nghĩa GPU LiDAR không cần renderer.

Nav2/SLAM Toolbox được phát hiện đã cài, chưa được kiểm chứng trong P1.
Driver X4 chưa kết nối thiết bị; không có số liệu hiệu năng Pi/ESP32 ở giai đoạn này.

Máy có NumPy 2 ở user site nhưng matplotlib hệ thống được build với NumPy 1.
Công cụ vẽ dùng `python3 -s` để chọn bộ NumPy/matplotlib của Ubuntu và đặt
`MPLCONFIGDIR=/tmp/forklift-matplotlib`; không sửa hoặc hạ phiên bản môi trường của người dùng.

Khớp nâng dùng khoảng đệm giới hạn cơ học SIM 5 mm và giới hạn vận tốc vật lý
0,10 m/s, trong khi trajectory vận hành tối đa 0,04 m/s. Hai khoảng dự phòng này
tránh chạm chính xác giới hạn DART/Fortress; không phải thông số cơ khí robot thật.
Xem [issue #192](https://github.com/ros-controls/gz_ros2_control/issues/192) và
[issue #1684](https://github.com/gazebosim/gz-sim/issues/1684).

## Bổ sung P3/P4 SIM (23/10/2026)

Workspace build sáu package `forklift_*`, gồm `forklift_hardware` và `forklift_navigation`. Nav2 1.1.20, SLAM Toolbox 2.6.10. Backend PTY dùng wall time và MCU giả; backend Gazebo/Nav2 dùng `/clock` với `use_sim_time=true`. Hai backend chạy riêng, không cùng điều khiển một tập joint. Không có firmware được nạp hoặc thiết bị thật được mở. Xem [hướng dẫn P3/P4](p3_p4_simulation.md).

## Bổ sung EKF/MPU6050 SIM

`robot_localization` 3.5.4 (gói Humble) đã có sẵn. IMU Gazebo 100 Hz qua `ros_gz_bridge`, EKF 30 Hz; không thêm driver phần cứng. Dữ liệu kiểm thử và replay ở `results/ekf/`; [thiết kế và giới hạn](ekf_mpu6050.md).

## Bổ sung P5-SIM

Workspace có tám package `forklift_*`, thêm `forklift_interfaces` (rosidl action/message) và `forklift_lift` (supervisor + SIM I/O). Gazebo vẫn dùng position actuator lý tưởng. Launch có khối lượng thử bắt đầu pause, kích hoạt lift controller trước, rồi resume; bánh khởi tạo chạm sàn để tránh va đập ở hard stop DART. [Hướng dẫn P5](p5_lift.md).
