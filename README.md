# Xe nâng AMR mecanum

Đồ án xe nâng tự hành dùng **ROS 2 Humble** và **Gazebo Fortress**, phát triển từ
[Trkkhrmn/ros2_amr_mecanumbot](https://github.com/Trkkhrmn/ros2_amr_mecanumbot).
Robot sử dụng bốn bánh mecanum, LiDAR, IMU và cơ cấu nâng vít me.

Hiện có thể chạy mô phỏng lấy hàng tại A → vận chuyển sang B → thả hàng → về Home.
Firmware ESP32 đang được phát triển; **chưa nạp board hoặc chạy xe thật**.

## Chức năng hiện tại

- Mô hình CAD URDF20 được tích hợp trong `forklift_description`.
- Điều khiển bốn bánh mecanum, nâng/hạ và homing càng trong mô phỏng.
- SLAM, định vị AMCL và điều hướng Nav2; EKF kết hợp odometry bánh với IMU mô phỏng.
- Bộ bám đường mặc định **MPPI**, cho phép tiến/lùi theo đường cong.
- Nhiệm vụ dùng Nav2 FollowPath theo các điểm đã duyệt; LiDAR căn chỉnh và chèn càng.
- Khi chạy Nav2 dùng vận tốc tiến/lùi và quay; đi ngang dùng cho teleop hoặc docking.

Đường nhiệm vụ được công bố đầy đủ trước khi xe di chuyển. Nếu tuyến không hợp lệ,
xe dừng/báo lỗi thay vì tự chọn tuyến khác. Giới hạn vận tốc nhiệm vụ hiện tại:
**tiến 0,25 m/s, lùi 0,16 m/s**; docking và chèn càng chạy chậm hơn.

## Môi trường cần có

- Ubuntu **22.04**, ROS 2 **Humble** đã được cài và cấu hình.
- Gazebo **Fortress**; executable là `ign gazebo`.
- `colcon`, `rosdep` và các dependency ROS trong `src/*/package.xml`.
- Các thành phần chính: `gz_ros2_control`, `ros_gz_bridge`, `ros_gz_sim`,
  `ros2_controllers`, Nav2/MPPI, SLAM Toolbox, `robot_localization` và RViz2.
- OpenGL/EGL hoạt động để mô phỏng GPU LiDAR, kể cả khi không mở Gazebo GUI.

Các package gốc `mecanum_*` được giữ để đối chiếu và có `COLCON_IGNORE`.
Giữ các file này khi build hệ thống Fortress; các package đang sử dụng mang tên `forklift_*`.

## Tải mã nguồn và build

Ví dụ đặt dự án tại `~/ros_ws/forklift`:

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
git clone https://github.com/thanhnt2410/forklift.git
cd forklift
source /opt/ros/humble/setup.bash
```

Nếu đã có workspace, vào thư mục dự án hiện tại, không cần clone lại.
Cài công cụ nếu máy chưa có:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Khởi tạo `rosdep` một lần bằng `sudo rosdep init` nếu máy chưa được khởi tạo, sau đó:

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

ROS 2 Humble và Gazebo Fortress cần được cài trước bước này. Không dùng Gazebo
Classic thay cho Fortress. `dependencies.repos` chỉ lưu baseline upstream để tham khảo;
không import đè lên mã nguồn `src/` đã chỉnh sửa.

**Trong mỗi terminal mới**, nạp môi trường trước khi chạy lệnh ROS:

```bash
cd ~/ros_ws/forklift
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Chạy nhiệm vụ lấy–giao hàng

Terminal 1 — mở thế giới có pallet, Gazebo, RViz, Nav2 và bộ quản lý nhiệm vụ:

```bash
ros2 launch forklift_bringup p6.launch.py controller:=mppi
```

Launch tự homing càng trong SIM. Chờ `/navigation/status` báo `READY` và
`/lift/state` có `homed: true`, không có lỗi. Có thể xem trong terminal đã source:

```bash
ros2 topic echo /navigation/status --once
ros2 topic echo /lift/state --once
```

Terminal 2 — gửi nhiệm vụ:

```bash
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission '{mission: a_to_b, timeout: 300.0}' --feedback
```

Chu trình: **Home → tới A → căn chỉnh/chèn càng → nâng → vận chuyển tới B →
thả hàng → rút càng → về Home**. Theo dõi tiến trình bằng feedback của action,
RViz hoặc topic `/mission/status`.

Một số tùy chọn:

```bash
# Tắt hai cửa sổ, vẫn chạy mô phỏng và cần renderer cho LiDAR:
ros2 launch forklift_bringup p6.launch.py gui:=false rviz:=false

# Dùng bộ RPP để đối chiếu thay cho MPPI:
ros2 launch forklift_bringup p6.launch.py controller:=rpp
```

Chọn một lệnh launch cho mỗi phiên. Dừng phiên bằng **Ctrl+C tại terminal launch**
và chờ tiến trình thoát trước khi mở phiên mới. Không chạy đồng thời nhiều bringup
trong cùng ROS domain/Gazebo partition.

## Chạy thủ công trong mô phỏng

Dừng phiên nhiệm vụ trước khi chuyển sang chế độ này.

Terminal 1 — mở mô phỏng cơ bản và tự homing càng:

```bash
ros2 launch forklift_bringup sim.launch.py auto_home:=true
```

Terminal 2 — sau khi càng đã homing, chạy điều khiển bằng phím mũi tên:

```bash
ros2 run forklift_control arrow_teleop.py
```

Click vào terminal điều khiển để nhận phím:

| Phím | Chức năng |
|---|---|
| ↑ / ↓ | Tiến 0,15 m/s / lùi 0,12 m/s |
| ← / → | Quay trái / phải, 0,4 rad/s |
| A / D hoặc Shift + ← / → | Đi ngang trái / phải, 0,12 m/s |
| R hoặc Page Up | Nâng càng thêm 1 cm |
| F hoặc Page Down | Hạ càng 1 cm |
| H hoặc Home | Homing càng |
| Space hoặc S | Dừng bánh và càng nâng |
| Q hoặc Ctrl+C | Dừng và thoát điều khiển |

Các phím chữ nhận cả chữ thường và chữ hoa (Shift/Caps Lock). Chọn bộ gõ tiếng Anh
để terminal gửi đúng phím A/D/R/F/H. Sau khi cập nhật code, thoát node bàn phím
cũ rồi chạy lại lệnh trên để nạp bản mới.

Giữ phím để lặp lệnh. Terminal không cung cấp sự kiện nhả phím: sau khoảng
0,35 s không nhận phím mới, node gửi zero qua command gate. Tốc độ dừng còn chịu
giới hạn gia tốc của gate. Lần giữ phím đầu có thể ngắt ngắn trước khi hệ điều hành
bắt đầu lặp phím; Space dùng để gửi dừng chủ động.
Không chạy cùng bàn phím khác, node tay cầm hoặc nhiệm vụ tự động.

Nhả phím di chuyển và chờ xe dừng trước khi bấm R/F/H. Lệnh nâng/hạ chạy tới
đích mỗi nấc 1 cm, không cần giữ phím; Space dừng giữa chừng. Khi đang nâng/home,
lệnh lái bị bỏ qua, không được xếp hàng để chạy lại sau đó. Cần homing trước
khi nâng/hạ; chiều cao được giới hạn 0–0,115 m trong SIM.

Có thể chọn thế giới kho bằng `world:=warehouse` khi launch mô phỏng.
Nếu không bật `auto_home`, gọi homing trước khi lái:

```bash
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}' --feedback
```

Nâng 10 cm, hạ về mốc homing hoặc dừng nâng, mỗi lần chọn một lệnh:

```bash
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.10}' --feedback
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.0}' --feedback
ros2 service call /lift/stop std_srvs/srv/Trigger '{}'
```

Trong SIM, xe bị khóa khi chưa homing, đang nâng/hạ hoặc trạng thái càng không cho
phép di chuyển. Gửi vận tốc teleop vào `/cmd_vel_teleop`; `/cmd_vel` là đầu ra nội bộ.

## Tay cầm PS4 trong mô phỏng

Đây là cấu hình SDL chuẩn để dùng khi có tay cầm; hiện có thể dùng phím mũi tên
như trên. Tay cầm USB 2,4 GHz chưa rõ model cần enumerate và kiểm tra ánh xạ trước.

Cài `ros-humble-joy`, build lại workspace và kết nối tay cầm PS4 qua USB hoặc
Bluetooth. Chạy mô phỏng thủ công như trên, rồi ở terminal đã source khác:

```bash
ros2 launch forklift_bringup ps4.launch.py
```

Thả L1 và đưa cần về giữa, sau đó giữ L1 để chạy: cần trái tiến/lùi/đi ngang,
cần phải xoay. L1 + D-pad lên/xuống nâng/hạ từng 1 cm; L1 + Triangle homing;
Circle gửi dừng bánh và nâng. Nhả L1 hoặc mất Joy sẽ gửi zero trong SIM.
Không chạy đồng thời với bàn phím hoặc nhiệm vụ tự động.

Xem [kết nối, ánh xạ nút và giới hạn PS4](docs/ps4_teleop.md).
Đã có tích hợp phần mềm; chưa kiểm chứng bằng tay cầm thật trong phiên này.

## SLAM và điều hướng riêng

Các chế độ dưới đây đã bao gồm Gazebo và RViz. Chạy từng chế độ riêng, không mở
chồng với `p6.launch.py` hoặc `sim.launch.py`:

```bash
# Lập bản đồ bằng LiDAR; dùng teleop để di chuyển:
ros2 launch forklift_bringup mapping.launch.py

# Định vị và điều hướng trên bản đồ kho đã có, mặc định MPPI:
ros2 launch forklift_bringup navigation.launch.py
```

Trong chế độ navigation, chờ trạng thái `READY` rồi dùng **Nav2 Goal** trên RViz.
Nếu pose ban đầu sai, đặt lại bằng **2D Pose Estimate**. Bản đồ mặc định ở
`src/forklift_navigation/maps/warehouse.yaml`; dùng bản đồ khác bằng tham số `map`:

```bash
ros2 launch forklift_bringup navigation.launch.py map:=/duong/dan/toi/map.yaml
```

## Phần cứng và firmware ESP32

Phần cứng dự kiến: ESP32 DevKit WROOM 38 chân, bốn GA25-370 có encoder A/B,
bốn BTS7960, TB6600/NEMA17 cho nâng và MPU6050 dự kiến nối Raspberry Pi.

Đã có khung firmware đọc encoder, PID bánh, UART v2 và điều khiển nâng. Tuy nhiên,
PI khởi đầu **Kp=0,08; Ki=0,12; Kd=0** đã được đặt cho cả bốn bánh để tuning.
**Firmware hiện bật profile thử bánh**, trần PWM 25%, giới hạn 2 rad/s và boot
disarm; nâng dùng **một công tắc NC dưới GPIO32**, R tới 2.000 xung từ home,
F/H hạ về công tắc rồi đặt lại vị trí 0. Bấm R nhiều lần không cộng dồn;
quãng nâng thực tế sẽ đo và hiệu chỉnh sau. Mức điện, chân boot,
chiều motor/encoder và PI chưa được kiểm chứng trên xe. Đã bổ sung backend USB/UART v2 và
`real.launch.py` để dùng cùng bàn phím với ESP32, đọc encoder và điều khiển nâng.
Các tham số thử nằm ở `firmware/esp32/forklift_esp32/board_config.h`.

Sau khi firmware đã được cấu hình đúng và nạp riêng, mở kết nối bằng:

```bash
ros2 launch forklift_bringup real.launch.py port:=/dev/serial/by-id/TEN_THIET_BI_ESP32
```

Launch mặc định disarm; cần kiểm tra telemetry và arm chủ động trước khi dùng
bàn phím. Xem [hướng dẫn ESP32: cấu hình, arm, bàn phím và reset](docs/esp32_manual_control.md).
Đã kiểm tra qua PTY với lõi firmware C++; chưa kiểm chứng kết nối USB hoặc xe thật.

Theo lựa chọn phần cứng hiện tại, R_EN/L_EN kéo HIGH ngoài và khi lỗi/mất liên lạc,
firmware giữ PWM cuối của bánh để người vận hành ngắt nguồn thủ công; lệnh dừng
chủ động vẫn có hiệu lực. Cơ cấu nâng giữ homing/giới hạn và dừng khi lỗi.
Đây là cấu hình firmware phần cứng, **không thay đổi cơ chế dừng của mô phỏng ROS**.

Xem [hướng dẫn firmware và GPIO](firmware/esp32/README.md),
[cấu hình ESP32/PID](firmware/esp32/forklift_esp32/board_config.h) và
[thông số phần cứng](config/robot_measurements.yaml).

## Cấu trúc và cấu hình chính

| Đường dẫn | Nội dung |
|---|---|
| `src/forklift_bringup/` | Launch và cấu hình hiển thị RViz |
| `src/forklift_description/` | URDF, CAD, mesh và hình học xe |
| `src/forklift_control/` | Điều khiển mecanum, command gate và EKF |
| `src/forklift_gazebo/` | World và bridge ROS–Gazebo |
| `src/forklift_navigation/` | SLAM, AMCL, Nav2, MPPI và bản đồ |
| `src/forklift_lift/`, `src/forklift_interfaces/` | Giám sát nâng, message và action |
| `src/forklift_task_manager/` | Tuyến đã duyệt, docking và nhiệm vụ |
| `src/forklift_hardware/` | Backend PTY/MCU giả và cầu nối ESP32 USB/UART v2 |
| `firmware/esp32/` | Mã firmware ESP32 đang phát triển |
| `docs/` | Tài liệu thiết kế và hướng dẫn chi tiết |
| `results/final/` | Nơi lưu kết quả cuối được chọn để đưa lên Git |

Các file thường cần chỉnh:

- [Hình học CAD dùng trong SIM](src/forklift_description/config/robot.yaml).
- [Cấu hình MPPI](src/forklift_navigation/config/nav2_mppi.yaml).
- [Các điểm path đã duyệt](src/forklift_task_manager/config/approved_route.json).
- [Trạm và docking](src/forklift_task_manager/config/stations.yaml).
- [Cấu hình nâng SIM](src/forklift_lift/config/lift.yaml).

Thư mục test và các kết quả ngoài `results/final/` chỉ giữ trên máy phát triển,
không có trong bản clone Git. README này không yêu cầu các file đó để chạy mô phỏng.

## Khi gặp vấn đề

- **Không tìm thấy package:** source lại ROS/workspace, kiểm tra dependency và build.
- **Xe không chạy:** kiểm tra homing càng, `/lift/state` và `/navigation/status`.
- **Mô phỏng chậm hoặc trùng xe:** dừng các phiên launch cũ trước khi mở phiên mới.
- **Không có LiDAR khi tắt GUI:** kiểm tra renderer OpenGL/EGL; tắt GUI không bỏ nhu cầu GPU.

Thiết kế và tiến độ chi tiết ở [PLAN.md](PLAN.md),
[điều hướng theo tuyến](docs/curved_navigation.md) và
[tích hợp CAD](docs/forklift_model.md).
Nguồn kế thừa và giấy phép được ghi tại [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
