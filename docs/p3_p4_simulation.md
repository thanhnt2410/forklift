# P3/P4 trong mô phỏng

Ngày triển khai: **23/10/2026**. Theo yêu cầu người dùng, hoãn P2/REAL:
**không nạp ESP32, không kết nối hoặc chạy xe thật**.

## Hai backend khác nhau

```text
P3: controller manager → PtySystem → PTY → MCU giả → encoder giả → odometry
P4: Nav2 → smoother → navigation_guard → command_gate → mecanum controller
                                                → gz_ros2_control → Gazebo
```

P3 kiểm tra vòng giao tiếp và lifecycle hardware bằng MCU giả, không có môi trường
vật lý Gazebo. P4 dùng backend vật lý Gazebo đã kiểm chứng P1 để chạy LiDAR, SLAM
và Nav2. Không chạy hai controller manager cùng điều khiển một robot trong một domain.
Hai backend giữ chung joint, geometry và giao diện lệnh/trạng thái.

**Bổ sung hiện tại:** Gazebo mặc định dùng wheel odometry + IMU qua EKF;
`/odom` đã lọc, raw wheel ở `/wheel/odometry`. Guard theo dõi thêm IMU và raw wheel.
[Hướng dẫn EKF/MPU6050 và kết quả 38/38](ekf_mpu6050.md).

## P3 — chạy MCU giả

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_hardware mock_hardware.launch.py
```

Terminal khác có thể dùng teleop như README. Chỉ có mô hình tín hiệu/odom ảo;
không có bánh motor hay GPIO thật. Xem [giao thức PTY](uart_protocol.md).

Tái hiện kiểm thử:

```bash
python3 scripts/run_mock_checks.py
colcon test --packages-select forklift_hardware
colcon test-result --test-result-base build/forklift_hardware --verbose
```

Các bài thử dùng domain riêng. Kết quả ở `results/p3/`: nominal, mất STATE,
CRC hỏng, reboot và ngắt PTY. Mốc 1 m/quay 360° được đối chiếu với trạng thái motor
ảo độc lập với lượng tử encoder; đây chưa phải hiệu chuẩn trên sàn thật.

## P4 — dựng bản đồ

```bash
ros2 launch forklift_bringup mapping.launch.py
```

Trong terminal thứ hai, dùng teleop quét kho rồi lưu map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
ros2 run nav2_map_server map_saver_cli -f src/forklift_navigation/maps/warehouse \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```

Hoặc chạy khảo sát và lưu tự động có thể tái hiện:

```bash
python3 scripts/run_p4_checks.py mapping
colcon build --symlink-install --packages-select forklift_navigation
```

Khảo sát dùng teleop theo waypoint để thu scan; **SLAM Toolbox** tạo map từ scan và
odometry. Không lấy hình học world/ground truth để xuất bản đồ giả.
Map kho đã tạo ở `src/forklift_navigation/maps/warehouse.yaml` và `.pgm`.

Dừng phiên mapping trước khi chạy localization; chỉ một thành phần phát `map → odom`.

## P4 — tự hành đến đích trên bản đồ đã lưu

```bash
ros2 launch forklift_bringup navigation.launch.py
```

Mặc định là world kho và map kho tương ứng. Chờ Nav2 báo `Managed nodes are active`
và `/navigation/status` là `READY`, rồi dùng **Nav2 Goal** trong RViz.
RViz phải có panel **Navigation 2** (hiển thị Navigation/Localization `active`).
Chọn **Nav2 Goal**, nhấn giữ chuột trái tại ô trống trên bản đồ, kéo để chọn
hướng mũi tên rồi thả chuột. Panel này nhận goal từ công cụ và gửi action Nav2.
RViz có công cụ **2D Pose Estimate** để đặt lại vị trí khi cần.

Hoặc gửi đích cụ thể bằng action:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.7, y: 0.6}, orientation: {w: 1.0}}}}' \
  --feedback
```

Chế độ này bật nguồn autonomous; teleop vẫn ưu tiên khi đang phát lệnh.
AMCL ban đầu dùng vị trí spawn `(0,0,0)` đã biết trong mô phỏng; không áp dụng
giả định đó cho robot thật hoặc khi tự di chuyển model sang chỗ khác.

## Chuỗi lệnh và bảo vệ phản hồi

- Controller/behavior Nav2 → `/nav/cmd_vel_raw` → velocity smoother →
  `/nav/cmd_vel_smoothed` → navigation guard → `/cmd_vel_nav` → command gate → `/cmd_vel`.
- DWB có vận tốc `vy` khác zero, AMCL dùng `OmniMotionModel`, smoother giữ cả `vx/vy/wz`.
- Footprint được dựng từ geometry, bao gồm bánh, thân và càng nhô ra.
- Navigation guard theo dõi scan, odom, TF `map→odom` và covariance AMCL;
  mất dữ liệu sau khi đã sẵn sàng sẽ giữ lỗi và xuất zero.
- Command gate tiếp tục giới hạn tốc độ/timeout/E-stop và khóa khi càng cao/đang nâng.
- Với nguồn autonomous, tín hiệu readiness mất/false khiến gate xuất zero ngay,
  bỏ qua ramp gia tốc bình thường; manual teleop vẫn có thể phục vụ phục hồi trong SIM.
- Dữ liệu physics `/test/poses` chỉ dùng trong kiểm thử; không cấp vào điều khiển.

Sau khi xử lý nguyên nhân lỗi scan/TF và hủy goal cũ, reset chủ động:

```bash
ros2 service call /navigation_guard/reset std_srvs/srv/Trigger '{}'
```

Nếu có path nhưng xe đứng yên, kiểm tra khóa trước khi gửi lại goal:

```bash
ros2 topic echo /navigation/status std_msgs/msg/String --once
```

Nếu là `FAULT_LATCHED`, hủy goal bằng **Cancel** trong panel Navigation 2,
đảm bảo mô phỏng đang chạy và phản hồi đã trở lại, rồi gọi reset như trên
và chọn goal mới. Trạng thái `Navigation: active` trong panel chỉ phản ánh
lifecycle Nav2, không thể hiện khóa guard. Tạm dừng hoặc gián đoạn mô phỏng
có thể kích hoạt khóa; dữ liệu trở lại không tự bỏ khóa.

Nếu phản hồi chưa sẵn sàng, reset bị từ chối. Dịch vụ sau tạo mất scan **trong SIM**:

```bash
ros2 service call /scan_relay/drop_scan std_srvs/srv/SetBool '{data: true}'
# Khôi phục cảm biến rồi reset guard:
ros2 service call /scan_relay/drop_scan std_srvs/srv/SetBool '{data: false}'
```

Tái hiện nghiệm thu Nav2:

```bash
python3 scripts/run_p4_checks.py navigation
# Hoặc quan sát cả Gazebo và RViz:
python3 scripts/run_p4_checks.py navigation --gui
```

Mặc định ghi trong `results/p4/`; dùng `--output results/p4_final` để tách lượt nghiệm thu. Các bài thử kiểm tra nhiều đích,
chuyển động ngang, vật cản mới trong costmap, hủy goal và lỗi scan/TF.
Bộ 10 đích yêu cầu ít nhất 9 lượt sai số ≤0,15 m và ≤10° theo kế hoạch;
mọi bài vật cản/dừng lỗi phải đạt. Từng lượt trượt ngưỡng vẫn giữ `passed: false`
trong `navigation_checks.json`, cùng tổng hợp `acceptance_summary.json`.
[Báo cáo triển khai](p3_p4_status.md) ghi điều kiện và kết quả thực tế.

## Giới hạn

P4 chỉ tự đi tới **pose đã chỉ định**, chưa nhận dạng pallet, căn chỉnh đưa càng
vào hàng hay tự nâng kiện hàng. Nâng có giám sát đã triển khai ở [P5-SIM](p5_lift.md); tự lấy/giao hàng vẫn thuộc P6. LiDAR 2D không đảm bảo
thấy vật thấp hơn mặt phẳng quét; khung/càng/tải và cảm biến thực vẫn phải đo lại.
Driver X4, CPU Pi, encoder thật, PID ESP32, tải nâng và E-stop vật lý chưa được thử.
