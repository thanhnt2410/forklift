# CLAUDE.md — Ngữ cảnh dự án Xe nâng AMR mecanum

> File này là bộ nhớ dài hạn cho Claude Code khi làm việc trong repo `forklift`.
> Cập nhật: **26/10/2026**. Mọi khẳng định dưới đây phải được đối chiếu lại với code
> trước khi dùng làm cơ sở cho quyết định kỹ thuật.

---

## 1. Vai trò và ngôn ngữ

- Đây là **đồ án sinh viên**: *"Thiết kế và điều khiển robot vận chuyển hàng trong
  nhà sử dụng bánh mecanum và cơ cấu nâng vít me trên nền ROS 2"*.
- **Trả lời bằng Tiếng Việt.** Thuật ngữ kỹ thuật giữ nguyên (ROS 2, mecanum,
  odometry, PID, encoder, LiDAR) hoặc kèm giải thích ngắn khi mới xuất hiện.
- Số liệu trong tài liệu là **tham số SIM / danh nghĩa**, không phải số đo phần
  cứng. Không được trình bày chúng như đã kiểm chứng.

---

## 2. Kiến trúc hệ thống

### 2.1 Công nghệ

| Thành phần | Lựa chọn |
|---|---|
| Hệ điều hành / ROS | Ubuntu 22.04 + **ROS 2 Humble** |
| Mô phỏng | **Gazebo Fortress** (`ign gazebo`), `gz_ros2_control`, `ros_gz_bridge` |
| Điều khiển bánh | `mecanum_drive_controller` (ros2_control), 4 bánh mecanum |
| Cơ cấu nâng | `JointTrajectoryController` + supervisor riêng (`forklift_lift`) |
| Cảm biến | YDLIDAR X4 (`/scan`, `laser_frame`), MPU6050 nối Raspberry Pi qua I2C |
| Định vị | EKF 2D 30 Hz (wheel odom + gyro z), SLAM Toolbox, AMCL (`OmniMotionModel`) |
| Điều hướng | Nav2 — **MPPI mặc định**, RPP làm phương án đối chiếu |
| Tính toán thực | **Raspberry Pi 4B** + **ESP32-WROOM 38 chân** |

### 2.2 Package

- **9 package đang dùng**, tiền tố `forklift_*` trong `src/`:
  `forklift_description` (URDF/CAD), `forklift_gazebo` (world, bridge),
  `forklift_control` (command gate, teleop, EKF), `forklift_hardware`
  (PTY mock, ESP32 bridge CSV, YDLIDAR), `forklift_interfaces` (message/action),
  `forklift_lift` (supervisor), `forklift_navigation` (SLAM/AMCL/Nav2/MPPI),
  `forklift_task_manager` (nhiệm vụ P6), `forklift_bringup` (launch, RViz).
- Package `mecanum_*` từ repo kế thừa **giữ lại để đối chiếu**, bị loại khỏi build
  bằng `COLCON_IGNORE`. Không xóa, không sửa, không build.
- `hardware/` (Jetson/STM32) và `third_party/` ở thư mục gốc là **tài sản kế thừa**
  của repo upstream, không thuộc kiến trúc Pi–ESP32. `docs/real_robot/` đang trống.

### 2.3 Sơ đồ dữ liệu (SIM)

```
Teleop/Nav2 → CommandGate → mecanum_controller → JointTrajectoryController
                                  ↓                      ↓
                          gz_ros2_control → Gazebo Fortress
                                  ↓
                         Wheel odometry + gyro z → EKF 30 Hz → /odom (odom→base_link)
                                  ↓
                    SLAM Toolbox hoặc AMCL ↔ /scan (base_link → laser_frame)
                                  ↓
                              Nav2 (MPPI/RPP)
```

**Quy tắc TF quan trọng:** `map → odom → base_link → laser_frame`. Chỉ có **một
nguồn phát `odom → base_link`**. Trong Gazebo đó là **EKF** (`enable_odom_tf=false`
cho base controller). Trong REAL/PTY đó là bridge hoặc controller.

### 2.4 Hai chế độ dùng chung giao diện

| | SIM (`sim.launch.py`, `p6.launch.py`) | REAL (`real.launch.py`) |
|---|---|---|
| Thời gian | `use_sim_time: true`, `/clock` | `use_sim_time: false` |
| Nguồn lệnh bánh | ros2_control qua Gazebo | `esp32_bridge.py` qua Serial CSV |
| LiDAR | Gazebo GPU LiDAR | `ydlidar_ros2_driver` (`lidar.launch.py`) |
| Odometry | EKF từ Gazebo | Bridge tích phân encoder |

---

## 3. Hiện trạng code sau các commit mới nhất

### 3.1 Năm commit gần đây và ý nghĩa

| Commit | Ngày | Nội dung |
|---|---|---|
| `d2259af` | 24/10 18:01 | Teleop Serial ESP32 + nâng **một công tắc dưới**. Thêm `esp32_bridge.py`, `esp32.yaml`, `real.launch.py`, `arrow_teleop.py` |
| `62f4d36` | 24/10 21:19 | Sửa nhả modem line (DTR rồi RTS) khi mở cổng + bộ đếm RX (`rx_bytes`/`rx_lines`/`discarded_frames`) vào diagnostics |
| `90155ce` | 24/10 22:24 | **Đơn giản hóa giao thức sang CSV từng dòng**: `simple_serial=true`, không CRC/boot ID/sequence/ACK; telemetry 20 Hz (50 ms) |
| `7e29c99` | 25/10 23:38 | **Mapping thật + dò cổng ESP32 tự động**: `lidar.launch.py`, `mapping_real.launch.py`, `mapping_real.rviz`, `esp32_serial_auto.py`, `ydlidar.yaml`, `scripts/sync_pi.sh`, sketch test `firmware/esp32_test/` |
| `d03ee21` | 26/01:07 | **Sửa firmware**: PI bánh `Kp=0,032; Ki=0,16; Kd=0,0`; `max_wheel_rad_s` 2 → 10; `max_duty` 0,25 → 0,50; nâng 2500 xung/s, hạ 2000 xung/s; **stall bánh từ fault chuyển thành warning** (thêm trường `warnings` vào telemetry); GPIO công tắc dưới 32 → 15; `command_gate` `max_vx` 0,25 → 0,30 và `max_vy` 0,20 → 0,30; `esp32.yaml` `max_wheel_speed` 2,0 → 6,0 và thêm `auto_arm: true` |

Mạch thay đổi chung: **từ giao thức nặng có CRC sang CSV đọc bằng mắt → dò cổng
tự động để chạy trên Pi → nới phạm vi thử bánh và hạ mức stall xuống warning.**

### 3.2 Firmware ESP32 (`firmware/esp32/forklift_esp32/`)

**Profile đang bật:** `base_bench_profile=true`, `single_lower_bench_profile=true`,
`simple_serial=true`. Các cờ xác nhận điện (`strapping_levels_reviewed`, ...) vẫn
`false` — **chưa kiểm chứng trên phần cứng thật**.

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Baud | **115200**, 8N1 | Giống phía ROS (`esp32.yaml:4`) — xem §6.1 |
| PI bốn bánh (đã nạp) | **Kp=0,032; Ki=0,16; Kd=0,0** | Tất cả 4 bánh FL/FR/RL/RR; tuned từ benchmark test |
| Chu kỳ điều khiển | 10 ms (100 Hz) | `control_period_us` |
| Telemetry | 50 ms (**20 Hz**) | `telemetry_period_us` |
| Trần PWM | **50 %** | `max_duty = .50` |
| Giới hạn MCU | **10 rad/s** | `max_wheel_rad_s = 10.` |
| Giới hạn phía ROS | **6 rad/s** (0,30 m/s với r=0,05) | `esp32.yaml: max_wheel_speed: 6.0` |
| CPR danh nghĩa | **937,2** = 11 PPR × 4 (x4) × 21,3 | Chưa đo; `robot_measurements.yaml:36` |
| **Tốc độ nâng (đã nạp)** | **2500 xung/s** | `lift_max_steps_s = 2500.` — nâng 20.000 bước chỉ mất ~8,5 giây |
| **Gia tốc nâng (đã nạp)** | **3000 xung/s²** | `lift_accel_steps_s2 = 3000.` — khởi động dứt khoát, tránh trượt bước |
| **Tốc độ hạ (đã nạp)** | **2000 xung/s** | `home_steps_s = 2000.` — chạy mượt mà tới công tắc đáy |
| `steps_per_m` | 200000 danh nghĩa | 200 bước/vòng × 8 vi bước ÷ lead 8 mm; **chưa đo** |

**Giao thức CSV (mặc định):**

```text
Telemetry: S,ms,tFL,tFR,tRL,tRR,lift_steps,mode,homed,lower,fault,capabilities,warnings
Ví dụ:     S,1200,0,0,0,0,0,1,0,1,0,3,0
Lệnh xuống: C,wFL,wFR,wRL,wRR,lift_steps,enabled   |   H   |   U   |   D   |   R
```

- `mode`: 0 khóa, 1 disarmed, 2 active, 3 đang nâng/hạ, 4 fault.
- `fault`: 1 config, 2 E-stop, 8 encoder, 16 limit, 32 home timeout, 64 deadline,
  128 peripheral. (Bit 4 watchdog Serial cũ coi như bỏ.)
- `capabilities`: 0 lỗi cấu hình, 1 chỉ bánh, 3 bánh + nâng.
- `warnings`: bit0..3 = FL/FR/RL/RR. Stall > 0,5 s (ramp > 0,5 rad/s, duty > 20 %,
  tốc độ đo < 0,05 rad/s, hoặc encoder ngược chiều lệnh) **chỉ phát warning**, không
  latch fault, không disarm. Encoder nhảy số phi lý và bánh còn quay khi nâng vẫn
  là fault 8.
- Khởi động **disarm**, PWM=0; nhận ngay `C,...,1` không cần `D` trước; **không còn
  timeout Serial** — im lặng trên dây không phải lỗi.
- `H` hạ tối đa 30 s hoặc tới công tắc dưới; `U` nâng **thêm 20.000 xung** (~10 cm) từ vị trí
  hiện tại; `D` dừng; `R` reset fault (khác phím `R` của bàn phím ROS — node đổi
  phím thành `U`).

**Chân GPIO đang dùng** (thứ tự nội bộ luôn FL, FR, RL, RR):

| Chức năng | GPIO |
|---|---|
| FL RPWM/LPWM · encoder A/B | 25/26 · 34/35 |
| FR RPWM/LPWM · encoder A/B | 18/19 · **2**/14 |
| RL RPWM/LPWM · encoder A/B | 23/22 · 5/17 |
| RR RPWM/LPWM · encoder A/B | 21/27 · 16/4 |
| Công tắc dưới (COM–GND, NC–GPIO) | **15** |
| TB6600 PUL+ / DIR+ | 13 / 33 |

`motor_sign={1,-1,1,-1}`, `encoder_sign={1,1,1,-1}`. Công tắc trên và E-stop GPIO
**chưa gán** (`-1`). `R_EN/L_EN` kéo HIGH ngoài, không dùng GPIO.

**Chính sách khi lỗi (do người dùng chọn):** `base_enable_hardwired_high=true`,
`stop_base_on_fault=false`. Lỗi hoặc mất liên lạc → **giữ PWM cuối**, báo fault, để
người vận hành ngắt nguồn thủ công. Lệnh `D` hoặc `C enabled=0` vẫn đưa PWM về 0.
Cơ cấu nâng vẫn dừng STEP khi lỗi và giữ homing/giới hạn.

**Build & test lõi native** (không cần board):

```bash
cmake -S firmware/esp32 -B build/esp32_native -DCMAKE_BUILD_TYPE=Debug
cmake --build build/esp32_native -j4
ctest --test-dir build/esp32_native --output-on-failure
```

Biên dịch sketch bằng Arduino CLI, core đã khóa `esp32:esp32 3.3.8`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --build-path "$PWD/build/esp32" \
  firmware/esp32/forklift_esp32
```

Trong môi trường ptrace hiện tại phải chạy với `ASAN_OPTIONS=detect_leaks=0`
(LeakSanitizer không chạy được; không coi đó là kiểm tra leak đã đạt).

### 3.3 Cầu nối ROS ↔ ESP32 (`src/forklift_hardware/scripts/esp32_bridge.py`)

- Node Python duy nhất cho REAL; **không** cần Gazebo hay ros2_control.
- Xuất: `/joint_states`, `/odom`, `/tf`, `/lift/state`, `/lift/moving`,
  `/esp32/status`, `/lift/preset_target` (latched), `/diagnostics`.
- Vào: `/cmd_vel`, `/safety/estop`.
- Service: `/esp32/arm`, `/esp32/reset`, `/lift/reset`, `/lift/stop`.
- Timeout: `state_timeout=0.25`, `command_timeout=0.3`, `ack_timeout=0.25`.
- `auto_arm: true` — tự arm một lần khi telemetry tươi, bánh đứng yên và nhận lệnh
  zero từ command gate. Fault/disarm **hủy** auto arm cho phiên hiện tại.
- Dò cổng tự động qua `esp32_serial_auto.py` (`port:=auto`): nhận diện bằng
  telemetry firmware, bỏ qua cổng đang được chương trình khác dùng — **cần `fuser`
  của gói `psmisc`**.
- `real.launch.py` cũng chạy `command_gate.py` với `use_sim_time=False`,
  `autonomous_enabled=False`, `require_navigation_ready=False`, và **lấy
  `wheel_radius`/`kinematic_sum`/`max_wheel_speed` từ chính `esp32.yaml`** nên một
  file cấu hình cấp cho cả hai node.

**Tham số nâng trong `esp32.yaml` (REAL, chế độ xung):**

| Tham số | Giá trị | Nghĩa |
|---|---|---|
| `lift_preset_mode` | `true` | Chế độ xung, không dùng mét |
| `lift_raise_steps` | 20000 | Phím `R` nâng thêm **20.000 xung** (~10 cm) từ hiện tại |
| `lift_max_steps` | 20000 | Giới hạn chiều cao tối đa **~10 cm** |
| `lift_steps_per_m` | 200000.0 | Đổi tọa độ UART danh nghĩa, **chưa hiệu chuẩn** |
| `lift_max_height` | 0.1 | Tương ứng 20.000 xung; **chưa đo chiều cao thật** |
| `lift_tolerance` | 0.002 | Sai số cho phép khi check đạt vị trí |
| `publish_odom_tf` | `true` | Tắt khi có node EKF/IMU thật sở hữu `odom → base_link` |

**Bàn phím `arrow_teleop.py` (dùng chung cho SIM và REAL):**

| Phím | Chức năng |
|---|---|
| ↑ / ↓ | Tiến / lùi |
| ← / → | Quay |
| A / D hoặc Shift+←/→ | Đi ngang |
| R / Page Up | Nâng (SIM: +1 cm · ESP32: +1 bậc 2.000 xung) |
| F / Page Down | Hạ (SIM: −1 cm · ESP32: hạ tối đa 3 s hoặc tới công tắc) |
| H / Home | Homing |
| Space / S | Dừng bánh và nâng |
| Q / Ctrl+C | Dừng và thoát |

- Mặc định: `speed 0.30`, `reverse_speed 0.30`, `strafe_speed 0.30`, `turn 0.4`
  (trần `turn 0.6`), `key_timeout 0.35 s`, `lift_step 0.01`, `lift_max_height 0.115`.
  Terminal không có sự kiện nhả phím nên sau `key_timeout` node gửi zero.
- Publish vào `/cmd_vel_teleop`. Node **tự xin chế độ thủ công** từ
  `/command_gate/set_autonomous` (lặp lại mỗi 1 s tới khi được nhận) và cần phím mới
  sau đó; đợi `/lift/state` và `/odom` tươi (`state_timeout 0.5`).
- Phát hiện chế độ nâng qua topic latched `/lift/preset_target`: có dữ liệu →
  chế độ xung ESP32, không có → chế độ mét SIM.

**Lưu ý — hai backend khác nhau trong `forklift_hardware`:**

- **REAL**: `esp32_bridge.py` (Python) nói chuyện Serial CSV với ESP32, ở trên.
- **SIM/kiểm thử**: `src/forklift_hardware/src/pty_system.cpp` là `ros2_control`
  `SystemInterface` (`forklift_hardware.xml`, pluginlib) + `scripts/mock_mcu.py`,
  `launch/mock_hardware.launch.py`. MCU giả chỉ kiểm chứng **đường dữ liệu và lỗi
  phần mềm**, không kiểm chứng UART vật lý, baud, nhiễu hay PID thật.

**Công thức động học** (để trích vào báo cáo, `esp32_bridge.py:121-131`):

Động học ngược — vận tốc thân → bốn bánh:

$$\dot{\varphi}_{FL} = \frac{v_x - v_y - k\,\omega_z}{r},\quad
\dot{\varphi}_{FR} = \frac{v_x + v_y + k\,\omega_z}{r}$$

$$\dot{\varphi}_{RL} = \frac{v_x + v_y - k\,\omega_z}{r},\quad
\dot{\varphi}_{RR} = \frac{v_x - v_y + k\,\omega_z}{r}$$

với $r$ bán kính bánh và $k = \tfrac{1}{2}(l_x + l_y)$ là tổng chiếu tâm robot lên
trục X và Y (`kinematic_sum`).

Động học xuôi — bốn bánh → vận tốc thân:

$$v_x = \tfrac{r}{4}(\dot{\varphi}_{FL}+\dot{\varphi}_{FR}+\dot{\varphi}_{RL}+\dot{\varphi}_{RR}),\quad
v_y = \tfrac{r}{4}(-\dot{\varphi}_{FL}+\dot{\varphi}_{FR}+\dot{\varphi}_{RL}-\dot{\varphi}_{RR})$$

$$\omega_z = \tfrac{r}{4k}(-\dot{\varphi}_{FL}+\dot{\varphi}_{FR}-\dot{\varphi}_{RL}+\dot{\varphi}_{RR})$$

Odometry tích phân từ encoder (`esp32_bridge.py:588-615`):

$$\Delta\theta_i = \frac{(n_i - n_{i-1})\cdot 2\pi}{\text{CPR}},\qquad
(v_x, v_y, \omega_z) = \text{forward}(\Delta\theta, r, k)$$

rồi cộng dồn $(\Delta x, \Delta y, \Delta\psi)$ theo phương trình quán tính trên
mặt phẳng. CPR danh nghĩa $937{,}2 = 11 \times 4 \times 21{,}3$.

**Bộ chọn nguồn và khóa liên động** (`command_gate.py`, viết báo cáo cần cả hai):

- Chọn nguồn theo thứ tự ưu tiên: `teleop` trước `nav` khi `autonomous_enabled`;
  chỉ `teleop` khi thủ công. Lệnh cũ hơn `command_timeout` bị bỏ.
- Khóa xe (`allowed`) khi: có `estop`, đang nâng (`/lift/moving`), hoặc trạng thái
  càng từ `/joint_states` vượt `transport_height` hoặc `|velocity| ≥ 0.002 m/s`, hoặc
  dữ liệu cũ (`state_timeout`). **Khóa bỏ qua ramp**, đặt thẳng `output = 0`.
- Giới hạn sau khi chọn nguồn: kẹp cứng theo `max_vx`/`max_vy`/`max_wz`, rồi co về
  theo **phong bào tốc độ bánh**:

$$\frac{|v_x| + |v_y| + k\,|\omega_z|}{r} \le \text{max\_wheel\_speed}$$

  và chỉ sau đó mới áp ramp gia tốc `max_ax`/`max_ay`/`max_aw`, rồi **kiểm tra lại
  phong bào** một lần nữa (các ramp độc lập trên từng trục có thể nhóm ra khỏi
  phong bào khi chuyển giữa hai lệnh hợp lệ).

### 3.4 Cấu hình đang dùng (giá trị đã đọc từ file)

| File | Tham số chính |
|---|---|
| `src/forklift_control/config/command_gate.yaml` | `max_vx 0.30`, `max_vy 0.30`, `max_wz 0.6`, `max_ax/ay 0.5`, `max_aw 1.5`, `max_wheel_speed 12.0` (SIM), `transport_height 0.02`, `command_timeout 0.3`, `state_timeout 0.5` |
| `src/forklift_lift/config/lift.yaml` | **File config duy nhất của package.** Mục `lift_supervisor`: `max_height 0.115` (giá trị launch nạp, đè mặc định 0,24 trong `lift_supervisor.py:24`; hard cap 0,24 tại dòng 29), `position_tolerance 0.002`, `home_timeout 60.0`, `auto_home false`. Mục `lift_sim_io`: `lower_position 0.0015`, `upper_position 0.12`, `switch_hysteresis 0.0005`, `command_timeout 0.3` |
| `src/forklift_description/config/robot.yaml` | CAD: `wheel_radius 0.05`, `separation_x 0.210`, `separation_y 0.31029`, `lift_travel 0.12`, LiDAR `[0,0,0.2946]` 720 tia, IMU `[0,0,0.12]` |
| `src/forklift_navigation/config/nav2_mppi.yaml` | `GuardedMPPIController`, `vx_max 0.25`, `vx_min -0.16`, `wz_max 0.6`, `motion_model DiffDrive`, 7 critic, `controller_frequency 20.0` |
| `src/forklift_navigation/config/nav2.yaml` | RPP `StableReversingController` (`desired_linear_vel 0.25`), goal checker xy 0,035 / yaw 0,05, planner `TerminalApproachPlanner` |
| `src/forklift_task_manager/config/stations.yaml` | A `[0.35,0.65]`, B `[0.35,-0.60]`, Home `[0,0,-1.309]`, `carry_joint_position 0.016`, `approach_speed 0.035`, `insertion_speed 0.015`, `retreat_speed 0.08`, `withdrawal 0.35` |
| `src/forklift_hardware/config/ydlidar.yaml` | LiDAR thật **115200 baud**, dòng một chiều — **khác profile X4 mặc định 128000** |
| `config/robot_measurements.yaml` | Số liệu người dùng báo cáo vs số đo thật (`pending_physical_measurements` gần như toàn `null`) |

### 3.5 Chu trình nhiệm vụ P6 (SIM)

Action `/mission/run` (`forklift_interfaces/action/RunMission`), ví dụ:

```bash
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission \
  '{mission: a_to_b, timeout: 300.0}' --feedback
```

Các đoạn được công bố trước khi xe chạy (`mission_route.py:27`, 9 đoạn):
`NAV_PICK → ALIGN_PICK → INSERT_PICK → RETREAT_PICK → NAV_DROP → ALIGN_DROP →
INSERT_DROP → RETREAT_DROP → RETURN_HOME`.

Các trạng thái supervisor (`task_manager.py`): `PLAN_MISSION` (dòng 209) →
`CHECK_READY` (373) → `HOME_LIFT` (375, nếu cần) → `LOWER_FOR_PICK` (376) →
các đoạn điều hướng → `LIFT_PICK` (381) → `VERIFY_LOAD` (383) → `RETREAT_PICK` (384)
→ ... → `LOWER_RELEASE` (389) → `RETURN_HOME` (394) → `DONE` (395); chuyển sang
`CANCELLED` khi hủy, `FAULT` khi lỗi (397). Điểm dừng/bám là `INSERT` hoặc `ALIGN`
dùng chung (329). `p6.launch.py` còn chạy `motion_mux.py` (loại trừ nguồn lệnh),
`contact_throttle` và bridge `/p6/contacts`.

**Chạy nhanh từng chế độ:**

| Mục đích | Lệnh |
|---|---|
| Mô phỏng trong PTY/mock hoặc thủ công | `ros2 launch forklift_bringup sim.launch.py auto_home:=true` |
| Nhiệm vụ lấy–giao (mặc định MPPI) | `ros2 launch forklift_bringup p6.launch.py controller:=mppi` |
| Mapping thật trên Pi + SLAM trên laptop | `ros2 launch forklift_bringup mapping_real.launch.py` |
| Định vị/điều hướng trên bản đồ kho | `ros2 launch forklift_bringup navigation.launch.py` |
| Chỉ chạy LiDAR trên Pi | `ros2 launch forklift_bringup lidar.launch.py` |
| Teleop thật với ESP32 | `ros2 launch forklift_bringup real.launch.py` |

Đồng bộ code lên Pi: `bash scripts/sync_pi.sh [user@host] [remote_dir]`
(mặc định `thainguyen1233@192.168.100.205`). Script **không thay** `ydlidar.yaml`
trên Pi (`--ignore-existing`) để giữ cổng LiDAR đo tại chỗ, và yêu cầu `fuser`.

### 3.6 Bằng chứng và thư mục

`results/` trong Git chỉ có `results/final/.gitkeep` — trống. Kết quả kiểm thử
`results/p*` được nhắc trong `PLAN.md` **không nằm trong bản clone**; chỉ tồn tại
trên máy phát triển. Vì vậy **không kiểm chứng lại số lượt thử bằng cách đọc
`results/`**, mà phải hỏi hoặc chạy lại.

---

## 4. Quy tắc lập trình

### 4.1 Chung

1. **Trả lời bằng Tiếng Việt.** Tên biến, hàm, topic, action giữ nguyên tiếng Anh.
2. **Comment chi tiết bằng tiếng Việt** cho các quyết định không hiển nhiên, giới
   hạn, và công thức. Ưu tiên giải thích *tại sao* hơn *làm gì*.
3. Không tạo file tài liệu mới (`*.md`) nếu người dùng không yêu cầu.
4. Không commit nếu chưa được yêu cầu. Không push. Không xóa `results/`, package
   `mecanum_*`, hay `firmware/`.
5. Không tự chạy kiểm thử tránh vật cản hay chu trình Gazebo — người dùng đã yêu
   cầu rõ điều này trong `PLAN.md` (dòng 6–7). Chạy build/test lõi native thì được.

### 4.2 Code ROS 2

- Node tự chứa trong package `forklift_*`; không thêm phụ thuộc nội bộ mới nếu
  chưa cần.
- Topic/action/service dùng **đúng tên trong hợp đồng §2.3 và §3.5**; đổi tên thì
  cập nhật cả launch, RViz và tài liệu.
- `/cmd_vel` là **đầu ra nội bộ** sau command gate; teleop gửi vào
  `/cmd_vel_teleop`, Nav2 vào `/cmd_vel_nav`. Không publish trực tiếp `/cmd_vel`.
- `mecanum_drive_controller` nhận `Twist` **unstamped** (`use_stamped_vel=false`),
  remap từ `/cmd_vel` sang `/base_controller/reference_unstamped`.
- Mọi node SIM đặt `use_sim_time=true`, node REAL `false`. Không trộn lẫn.
- Node an toàn phải xử lý: timeout, dữ liệu cũ (stale), fault latch và **không tự
  phục hồi** — cần thao tác chủ động (reset/home lại).
- QoS: topic cảm biến dùng profile phù hợp (best effort, độ sâu nhỏ); topic trạng
  thái cần giữ (latched, `TRANSIENT_LOCAL`).

### 4.3 Firmware ESP32

- Tách biệt phần phụ thuộc phần cứng vào `board_hal.*`, phần logic kiểm thử được
  vào `firmware_core.*` để chạy native trên máy tính.
- Mọi thay đổi giao thức phải **đồng bộ cả firmware và ROS** (`simple_serial=true`
  ↔ `serial_protocol: simple`); không trộn bản v2 cũ (`S,2,...*CRC`) với bản simple.
- Giá trị kinh tế học/lift là **danh nghĩa cho tới khi đo được**; luôn ghi chú
  "chưa hiệu chuẩn" cạnh số liệu.

---

## 5. Đầu việc ưu tiên tiếp theo (theo `PLAN.md` §11)

### 5.1 Nhánh SIM / báo cáo (P7)

1. **Bổ sung các phép thử còn thiếu theo đúng giao thức gốc** — T3 (tiến 1 m, ngang
   1 m, quay 360°, mỗi bài 5 lần), T4 (relocalization lặp tại 3 vị trí), T5 (5 đích
   × 2 lần), T9 (chạy liên tục 20 phút kèm CPU/RAM/nhiệt/jitter/tuổi telemetry).
   Hiện chỉ có bộ 10 đích T5 trong SIM, chưa đủ giao thức lặp đôi.
2. **Kiểm tra tái hiện từ môi trường sạch** theo `README.md` và khóa dependency.
3. **Hoàn thiện báo cáo**: sơ đồ điện, kiến trúc, phương trình, dữ liệu đo, hình và
   video; tách rõ phần kế thừa, phần tự xây dựng và giới hạn đồ án.

### 5.2 Nhánh REAL (các gate vẫn mở, giữ nguyên thứ tự)

Thứ tự bắt buộc trong `PLAN.md` §11: **P0 đo lường → P2 thử bàn → P3 serial +
odometry → P4 X4/MPU6050/EKF/Nav2 → P5 nâng/cơ khí → P6 REAL.** Không dùng gate
SIM để bỏ qua bước nào.

- **P0 — đo thật** (`config/robot_measurements.yaml` gần như toàn `null`): CPR
  thực của encoder, chiều dài bánh/kích thước khung, khối lượng, bước vít me và
  lead, hành trình nâng, tải trọng, điện áp/dòng nguồn, mount và bias MPU6050.
- **P2 — thử bán ESP32**: xác nhận chiều motor/encoder từng bánh, đo PID 3 mức tốc
  độ × 2 chiều, kiểm chứng `steps_per_m` bằng thước, đo E-stop/dừng.
- **P3 — nhánh REAL**: hiệu chuẩn CPR/`wheel_radius`/dấu bánh/covariance; chạy
  1 m tiến, 1 m ngang, 360°, mỗi bài 5 lần trên sàn.
- **P4/P5/P6 REAL**: driver X4/MPU6050 thật, định vị/điều hướng trên Pi, đo nâng
  bằng dụng cụ ngoài, thử cơ khí có tải.

### 5.3 Việc kỹ thuật gần nhất, có thể làm ngay

1. **Nạp lại firmware ESP32 và build bridge trên Pi cùng phiên bản giao thức** —
   sau `d03ee21` firmware đã đổi `Kd`, nới PWM và đổi stall thành warning; bridge
   trên Pi phải khớp.
2. **Chốt mâu thuẫn baud** — xem §6.1.
3. Xác nhận `step_up_level`/mức điện TB6600 và cực tính `ENA` trước khi bật driver.
4. Kiểm tra `GPIO2` (FR encoder A) và `GPIO5` (RL encoder A) là chân strapping:
   `GPIO2` phải LOW hoặc trạng thái đúng khi vào chế độ nạp UART, nếu không tháo
   dây tín hiệu khi nạp.

---

## 6. Cạm bẫy và điểm lệch cần biết

### 6.1 Baud của giao thức ESP32

`PLAN.md` dòng 9 ghi **230400 baud**, nhưng code hiện tại là **115200** ở cả hai
phía: `firmware/esp32/forklift_esp32/board_config.h:54` và
`src/forklift_hardware/config/esp32.yaml:4`; `esp32_bridge.py:230` chỉ chấp nhận
115200 hoặc 230400. **Coi 115200 là giá trị đúng** cho tới khi người dùng chốt, và
nếu đổi thì phải đổi cả hai phía cùng lúc.

### 6.2 Các khác biệt lệch tài liệu

- `PLAN.md` §5 dòng 305 và 312 ghi `forklift_bringup/ ... chưa có real`,
  `firmware/esp32` (P2) và `real.launch.py` "**chưa được tạo**" — **đã có từ
  `d2259af`/`7e29c99`**. `lidar.launch.py` và `mapping_real.launch.py` **không được
  nhắc đến** trong `PLAN.md` (0 lần), nên tài liệu không hề nói về chúng.
- Số package: cả hai chỗ trong `PLAN.md` (§5 dòng 131 và dòng 312) đều ghi **9
  package `forklift_*`** — khớp thực tế (`ls -d src/forklift_* | wc -l` = 9;
  `mecanum_*` 7 package, cả 7 đều có `COLCON_IGNORE`).
- Chiều cao nâng SIM: `PLAN.md` (§P5) nói giới hạn **0,24 m**, nhưng cấu hình
  hiện tại trong [lift.yaml](src/forklift_lift/config/lift.yaml) là **0,115 m**
  (`max_height`), đè lên mặc định 0,24 của `lift_supervisor.py:24`. Giá trị đang
  thực sự dùng khi chạy qua `sim.launch.py` là **0,115 m**. Hard cap trong code vẫn
  là 0,24. **Không tự nâng lên 0,24 rồi cho Nav2 chạy** — công tắc gate nâng nằm
  ở `command_gate`/`PLAN.md` §6.3.
- `PLAN.md` §3.4 nói `lx+ly = 0.2275`, nhưng `esp32.yaml` dùng
  `kinematic_sum: 0.260145` cho REAL. Hai giá trị này **khác nghĩa**: SIM dùng
  `robot.yaml` (CAD, `separation_y 0.31029`), REAL dùng số đo người dùng. Không
  dùng CAD để ghi đè hiệu chuẩn REAL và ngược lại
  (`robot_measurements.yaml:101-107`).
- `results/` trống trong Git → **không kiểm chứng lại "29/29", "10/10" bằng đọc
  file được**.

### 6.3 Firmware mới nhất — PID và GPIO công tắc (28/10/2026)

**Thay đổi được nạp thực tế (từ code pasted 28/10):**
- **PI bánh:** Tất cả 4 bánh `Kp=0.032, Ki=0.16, Kd=0.0` (không phải Kd=0.05 như ghi cũ)
- **Tốc độ nâng:** 2500 xung/s; gia tốc 3000 xung/s² (hoàn thành 20.000 xung trong ~8,5 giây)
- **Tốc độ hạ:** 2000 xung/s (mượt đến công tắc)
- **GPIO công tắc dưới:** Thay đổi từ GPIO32 sang **GPIO15** (`INPUT_PULLUP`)
- **Lệnh nâng R:** Tăng từ 2.000 xung lên **20.000 xung** (~10 cm)
- **Thời gian hạ H:** Tăng từ 3 s lên **30 giây** (chờ tới công tắc)

**Tại sao thay đổi:**
- Tốc độ/gia tốc mới để hoàn thành nâng trước timeout 10 giây firmware
- GPIO15 thay GPIO32 vì strapping level (GPIO32 có vấn đề trong boot)

### 6.4 Những điều tuyệt đối không suy diễn

- **Firmware đã nạp** (28/10) với PID/tốc độ như trên; chưa nạp bản cũ.
- Xe thật chưa chạy; driver MPU6050 thật chưa có.
- `steps_per_m` 200000 và CPR 937,2 là **suy diễn từ thông số danh nghĩa**, chưa
  đo; goal `height: 0.01` hiện nghĩa là 2000 xung, **không có nghĩa là 1 cm**.
- PWM=0 **không** tương đương cách ly nguồn khi `R_EN/L_EN` kéo HIGH ngoài; không
  coi đó là cơ chế an toàn.
- LiDAR 2D và Nav2 **không chứng minh** khả năng đưa càng vào pallet chính xác;
  tuyến P6 phụ thuộc trạm cố định và pallet mẫu.
- Kết quả SIM không thay thế nghiệm thu phần cứng ở bất kỳ gate nào.

---

## 7. Tài liệu tham khảo trong repo

| File | Dùng khi cần |
|---|---|
| `README.md` | Cài đặt, chạy từng chế độ, bàn phím/PS4, mapping trên Pi + laptop |
| `PLAN.md` | Kế hoạch 12 tuần, trạng thái P0–P7, ma trận nghiệm thu T1–T9, rủi ro |
| `docs/esp32_manual_control.md` | Điều khiển bàn phím qua ESP32, thử bánh bằng Serial Monitor, xử lý `NO_TELEMETRY`, đo quãng nâng |
| `docs/ydlidar_pi.md` | LiDAR trên Pi, kết nối laptop, `ROS_DOMAIN_ID`, lưu bản đồ |
| `docs/curved_navigation.md` | Điều hướng theo tuyến, lùi theo cung |
| `docs/forklift_model.md` | Tích hợp CAD URDF20 |
| `docs/hardware_parameter_notes.md` | Diễn giải bảng thông số motor/encoder thiếu |
| `docs/uart_protocol.md` | Giao thức v1/v2 cũ (giữ để đối chiếu) |
| `firmware/esp32/README.md` | Profile firmware, bảng chân, build/test, giới hạn |
| `config/robot_measurements.yaml` | Số đo đã có vs còn thiếu, GPIO người dùng cung cấp |
| `THIRD_PARTY_NOTICES.md` | Nguồn kế thừa và giấy phép |
