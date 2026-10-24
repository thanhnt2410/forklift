# Odometry bánh + MPU6050 qua EKF — mô phỏng

Đã triển khai ngày 23/10/2026; chưa đọc I²C hoặc nạp firmware ESP32.
MPU6050 là cảm biến sáu trục (gyro + accelerometer), không có từ kế tích hợp.
Vì vậy cấu hình này **không dùng yaw tuyệt đối từ IMU**.
Nguồn: [datasheet TDK/InvenSense](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf).

## Luồng dữ liệu đang chạy

```text
Phản hồi 4 bánh → mecanum controller → /wheel/odometry ──┐
Gazebo IMU → /imu/sim → imu_adapter → /imu/data_raw ─────┴→ EKF → /odom
                                                        TF odom → base_link
LiDAR + bản đồ + TF odom → AMCL → TF map → odom → Nav2
```

`/odom` là đầu ra đã lọc của EKF, giữ tên để tương thích Nav2/SLAM và các client
hiện có. Topic mặc định `odometry/filtered` của robot_localization được remap
sang `/odom`; không chạy thêm một bộ tính odom khác trên topic này.
Raw wheel odometry được giữ riêng để đo và đối chiếu.

- EKF 2D, 30 Hz, `world_frame=odom`, `base_link_frame=base_link`.
- Fuse `vx`, **`vy`** và `vyaw` từ wheel twist; không fuse pose bánh đã tích phân
  từ cùng nguồn encoder. Không áp đặt `vy=0` vì xe mecanum đi ngang được.
- Fuse **`angular_velocity.z`** của IMU; wheel yaw rate có covariance lớn hơn gyro.
- Không fuse orientation hoặc acceleration của IMU. Gia tốc vẫn được xuất để
  khảo sát, có trọng lực, nhưng chưa tích phân thành vận tốc khi thiếu hiệu chuẩn.
- Controller bánh tắt `enable_odom_tf`; EKF là nguồn phát `odom → base_link`.
  AMCL hoặc SLAM là nguồn `map → odom`, chạy luân phiên theo chế độ.

Tham số: [ekf.yaml](../src/forklift_control/config/ekf.yaml).
Theo hướng dẫn [cấu hình robot_localization](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst).

## IMU mô phỏng và giao diện dành cho MPU6050

Gazebo tạo mẫu gyro/acceleration ở 100 Hz từ chuyển động vật lý, độc lập với
encoder. Adapter xóa orientation lý tưởng và đặt `orientation_covariance[0]=-1`
trước khi xuất `/imu/data_raw`. Không lấy `/test/poses` để cấp vào EKF.

Nhiễu Gaussian SIM: gyro σ=0,005 rad/s, acceleration σ=0,03 m/s²; covariance
được khai báo tương ứng. Đây là **giả định mô phỏng**, không phải số liệu hiệu chuẩn
hoặc mô hình đầy đủ MPU6050. Chưa mô phỏng bias nhiệt, lượng tử ADC, DMP, bus I²C
hay rung từ motor. EKF này cũng không tự ước lượng và khử bias gyro.

Vị trí/góc lắp IMU tạm thời ở `imu_xyz`/`imu_rpy` trong
[robot.yaml](../src/forklift_description/config/robot.yaml). Khi chuyển sang MPU6050 thật:

1. Driver xuất `sensor_msgs/Imu` trên `/imu/data_raw`, timestamp lúc lấy mẫu,
   `frame_id=imu_link`; gyro **rad/s**, gia tốc **m/s²**.
2. Đo bias khi xe đứng yên, kiểm tra chiều trục, đổi từ đơn vị raw/°/s/g và
   khai báo covariance theo phép đo. Không xuất yaw tích phân như heading tuyệt đối.
3. Cập nhật TF theo vị trí/góc lắp board thực. Trục thân xe: x trước, y trái, z lên.
4. Thay nguồn Gazebo/adapter bằng driver phần cứng, tránh hai nguồn cùng topic.
   Phần này chưa triển khai; không có chân GPIO, cổng serial hoặc I²C nào được mở.

## Chạy và khôi phục lỗi

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_bringup navigation.launch.py
```

EKF mặc định bật trong `sim`, `mapping` và `navigation`. Để so sánh với wheel-only,
dừng phiên hiện tại rồi chạy `use_ekf:=false`; không chạy hai backend trong cùng domain.
Backend PTY P3 tiếp tục chỉ kiểm tra encoder/protocol, chưa có IMU độc lập nên
không tạo gyro giả từ chính odometry bánh để đưa vào EKF.

Guard Nav2 theo dõi thêm **IMU và raw wheel odometry**, vì EKF có thể tiếp tục
phát dự đoán dù mất một đầu vào. Mất nguồn quá 0,7 s khiến xe dừng và giữ lỗi;
khôi phục dữ liệu không tự bỏ khóa. Mất TF/scan vẫn giữ các bảo vệ trước đây.

```bash
# Chỉ chèn lỗi trong SIM:
ros2 service call /imu_adapter/drop std_srvs/srv/SetBool '{data: true}'
ros2 service call /imu_adapter/drop std_srvs/srv/SetBool '{data: false}'
# Hủy goal cũ trong RViz, kiểm tra phản hồi, rồi reset:
ros2 service call /navigation_guard/reset std_srvs/srv/Trigger '{}'
```

[Video demo EKF](../results/ekf/navigation_demo.mp4) ·
[Đồ thị replay](../results/ekf/replay/fusion_comparison.png) ·
[Tổng hợp kiểm tra](../results/ekf/summary.json)

## Kết quả kiểm tra

| Phép kiểm tra | Kết quả và bằng chứng |
|---|---|
| Hợp đồng cảm biến/TF thực chạy | 5/5: [kiểm tra](../results/ekf/sensor_contract.json) |
| Adapter + guard/gate | 9/9 Python test: [log](../results/ekf/unit_tests.log) |
| Hồi quy di chuyển/nâng/watchdog | 37/37: [checks](../results/ekf/p1/checks.json) |
| Nav2 với EKF | 38/38, gồm 10/10 đích, vật cản, mất scan/TF/**IMU**, reset: [summary](../results/ekf/navigation/acceptance_summary.json) |
| Sai số đích Nav2 | Lớn nhất 0,1464 m và 6,06° trong bộ 10 đích; đối chiếu pose vật lý Gazebo |
| Replay wheel yaw rate sai +50% | Sai số góc 0,8 rad ở wheel-only, khoảng 0,0021 rad ở EKF: [checks](../results/ekf/replay/checks.json) |
| Đi ngang với EKF | Replay giữ được khoảng 0,601 m chuyển động ngang, không bị ràng buộc `vy=0` |

Replay là phép thử chèn lỗi dữ liệu đo có chân trị định trước, không phải mô phỏng
tiếp xúc/trượt bánh thực và không dùng để suy ra độ chính xác của MPU6050 thật.

Tái hiện trên domain riêng, sau khi source workspace:

```bash
ROS_DOMAIN_ID=118 ROS_LOCALHOST_ONLY=1 python3 tests/check_ekf_fusion.py
python3 scripts/run_checks.py p1 --domain 117 --output results/ekf/p1
python3 scripts/run_p4_checks.py navigation --domain 119 --output results/ekf/navigation
# Khi một phiên Gazebo+EKF đang chạy, dùng cùng domain với phiên đó:
python3 tests/check_ekf_contract.py
```

## Giới hạn

Gyro giúp ước lượng góc khi bánh trượt; **không trực tiếp sửa được quãng đường
trượt ngang/dọc**. Cần tiếp tục dùng LiDAR/AMCL và đo trên sàn thật. Yaw gyro vẫn
có thể drift theo thời gian; không coi MPU6050 là la bàn. Kết quả Nav2 trên đây
là kiểm tra tích hợp, không chứng minh EKF cải thiện mọi sai số vị trí so với P4 cũ.
