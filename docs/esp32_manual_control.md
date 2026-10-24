# Điều khiển bàn phím qua ESP32 USB/UART

Backend này nối ROS với **UART v2 của firmware trong repo**, không cần Gazebo.
Đã kiểm tra phần mềm qua PTY với lõi C++ của firmware; chưa nạp firmware hoặc
kiểm chứng trên xe thật. Firmware hiện bật **profile thử bánh** theo yêu cầu dùng
PI mặc định để tuning: Kp=0,08, Ki=0,12, Kd=0; PWM tối đa 25%, bánh tối đa 2 rad/s.
Khởi động vẫn disarm, cần arm từ ROS. Profile không xác nhận mức điện/đấu dây.
Nâng hiện dùng **một công tắc NC dưới GPIO32**, điều khiển theo số xung, chưa
hiệu chuẩn quãng đường thực. Không có công tắc trên hoặc phản hồi E-stop GPIO.

## Luồng điều khiển

```text
arrow_teleop → /cmd_vel_teleop → command_gate → /cmd_vel
                                             ↓
                                       esp32_bridge
                                   C / H / D / R UART v2
                                             ↓
                                ESP32: PID bánh + STEP/DIR
                                             ↓
                          encoder / trạng thái / ACK → ROS
```

Bridge tính vận tốc bánh theo thứ tự **FL, FR, RL, RR**, nhận số đếm encoder,
phát `/joint_states`, `/odom` và tùy chọn TF `odom → base_link`. Odometry này chỉ
sử dụng encoder, **chưa kết hợp MPU6050 thật**. Launch hiện chỉ phục vụ điều khiển
thủ công; không khởi chạy Nav2, Gazebo, ros2_control hoặc supervisor nâng SIM.
ESP32 đảm nhiệm PID, homing, công tắc và tạo xung nâng; bridge cung cấp cùng các
action `/lift/home`, `/lift/move` và service `/lift/stop` cho bàn phím.
Không chạy launch SIM, node tay cầm hay backend khác cùng ROS domain với launch này.

## Cấu hình

[esp32.yaml](../src/forklift_hardware/config/esp32.yaml) dùng baud **230400, 8N1**.
Hình học CAD và CPR **937,2** là giá trị ban đầu, chưa phải hiệu chuẩn xe thật.
`max_wheel_speed: 2.0` phải khớp hoặc thấp hơn giới hạn trong firmware.
Với bánh bán kính 0,05 m, chuyển động thẳng bị giới hạn khoảng **0,10 m/s**;
chuyển động kết hợp được scale chung bốn bánh để giữ hướng vận tốc.

Firmware dùng dấu `motor_sign` và `encoder_sign` trong
[board_config.h](../firmware/esp32/forklift_esp32/board_config.h). Không đảo dấu
lần nữa trong bridge. Kiểm tra chiều từng bánh khi hoàn tất cấu hình thử bàn.

Profile mặc định là `lift_preset_mode: true`, `lift_raise_steps: 2000`,
`lift_max_steps: 2000`. Firmware phải dùng cùng số xung và thang tọa độ UART.
Nối **COM công tắc → GND, NC → GPIO32**. Input kéo lên 3,3 V nội; trạng thái
bình thường LOW, chạm đáy/hở HIGH. Không nối GPIO32 lên 5 V.

Sau khi arm, bấm **F hoặc H** để lấy home lần đầu. Xe chưa home sẽ khóa lái và
không nhận nâng. Homing tìm đáy, nhả công tắc rồi chạm lại chậm trước khi đặt 0.
Không tự chạy càng khi vừa mở kết nối. R tới 2.000 xung tuyệt đối từ home;
giữ/bấm R tiếp không tăng thêm quãng nâng. F chạy tới công tắc, không chỉ chạy
ngược đúng số xung đã nâng. Không nhấn được công tắc, không nhả hoặc không chạm
lại đều bị giới hạn bằng số xung và thời gian trong firmware.

Giá trị `lift_steps_per_m: 200000.0`, `lift_max_height: 0.01` hiện là **quy đổi
tọa độ UART danh nghĩa**, không xác nhận chiều cao 1 cm. Bridge kiểm tra hoàn tất
R bằng **đủ đúng 2.000 xung**, F/H bằng xác nhận home và công tắc dưới. Trạng thái
báo `lift_mode=PULSE_PRESET; lift_calibrated=False`; `/lift/state.height` và
`/joint_states` phần nâng vẫn là khoảng cách ước tính từ thang danh nghĩa.

Bridge phát `/lift/preset_target` để bàn phím tự nhận chế độ; không cần đổi lệnh
chạy bàn phím. Với SIM không có topic này, R/F vẫn nâng/hạ từng nấc như trước.
Trong pulse mode, action `/lift/move` chỉ nhận goal 0 (tìm công tắc) hoặc goal
preset (nâng một lần); goal ở giữa bị từ chối.

- Capability **0**: cấu hình/khởi tạo lỗi, không arm được.
- Capability **1**: chỉ bánh, không nhận R/F/H.
- Capability **3**: bánh+nâng; profile pulse dùng giá trị quy đổi dự kiến trước
  khi đo thực. Chế độ position (`lift_preset_mode: false`) vẫn cần hiệu chuẩn
  khoảng cách ở hai phía và nhận goal chiều cao liên tục.

## Đo quãng nâng và hiệu chỉnh

1. Thử không tải, bấm F/H về công tắc, đánh dấu vị trí ban đầu.
2. Bấm R một lần, đo quãng dịch chuyển thực tế **d mm** của càng, ghi mức vi bước
   TB6600 đang đặt. Nếu chiều chạy ngược, sửa `step_up_level` trước khi thử tiếp.
3. Thang đo thực tương ứng là `steps_per_m = 2000 × 1000 / d`.
   Muốn đổi quãng nâng thành **D mm** với cùng vi bước, số xung mới xấp xỉ
   `round(2000 × D / d)`.
4. Sửa `lift_raise_steps` và `lift_max_steps` trong `board_config.h` cùng hai
   tham số tương ứng trong `esp32.yaml`. Cập nhật `lift_max_height = max_steps /
   steps_per_m` phía ROS. Nếu đổi thang khoảng cách, sửa `c.steps_per_m` và
   `c.travel_m` trong firmware khớp với ROS; các budget home/backoff hiện theo
   số xung. Build/nạp lại firmware và khởi động lại launch sau thay đổi.

Ví dụ đo 2.000 xung được 8 mm: `steps_per_m=250000`; muốn nâng 10 mm thì đặt
2.500 xung và cập nhật cả trần xung. Ví dụ này không phải số đo xe hiện tại.
Số xung không phát hiện trượt bước; công tắc dưới giúp lấy lại mốc mỗi lần hạ.

## Build và mở kết nối

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select forklift_interfaces forklift_control forklift_hardware forklift_bringup
source install/setup.bash
ls -l /dev/serial/by-id/
ros2 launch forklift_bringup real.launch.py port:=/dev/serial/by-id/TEN_THIET_BI_ESP32
```

Thay tên thiết bị bằng đường dẫn thực; có thể dùng `/dev/ttyUSB0` nếu đó đúng là
ESP32. Tài khoản cần quyền đọc/ghi cổng serial. Bridge chiếm độc quyền cổng;
đóng Serial Monitor trước khi launch. Một số board reset khi mở USB serial;
bridge đợi telemetry chứa boot ID của phiên mới, không gửi lệnh chạy trước đó.

Muốn dùng cấu hình hiệu chuẩn riêng:

```bash
ros2 launch forklift_bringup real.launch.py port:=/dev/ttyUSB0 config:=/duong/dan/esp32.yaml
```

Mở terminal khác, source workspace như trên rồi xem trạng thái:

```bash
ros2 topic echo /esp32/status
```

Chờ `DISARMED; host=OK; mcu=DISARMED`, capability hợp lệ và `handshake=True`.
Lệnh arm dưới đây gửi yêu cầu; `success: true` nghĩa là **đã gửi ARMING**, phải
chờ trạng thái **ARMED** xác nhận ACK và mode ACTIVE của MCU trước khi điều khiển:

```bash
ros2 service call /esp32/arm std_srvs/srv/SetBool '{data: true}'
ros2 run forklift_control arrow_teleop.py
```

Chọn bộ gõ tiếng Anh, focus terminal bàn phím và chờ `Manual mode ready`.

| Phím | Chức năng |
|---|---|
| ↑ / ↓ | Tiến / lùi |
| ← / → | Quay trái / phải |
| A / D, hoặc Shift + ← / → | Đi ngang trái / phải |
| H / Home, F / Page Down | Hạ về công tắc, homing và đặt lại 0 |
| R / Page Up | Nâng tới preset 2.000 xung từ home, xe phải dừng |
| Space / S | Gửi zero bánh và D dừng/hủy nâng |
| Q / Ctrl+C | Dừng và thoát bàn phím |

Phím chữ nhận cả hoa/thường. Nhả phím hết timeout bàn phím vẫn là lệnh dừng chủ
động như SIM; command gate giữ giới hạn gia tốc và khóa lái khi nâng, chưa home
hoặc nâng quá chiều cao vận chuyển. Một goal nâng chạy đến đích, không cần giữ R/F; Space hủy giữa chừng.

## Dừng, lỗi và nối lại

Disarm chủ động:

```bash
ros2 service call /esp32/arm std_srvs/srv/SetBool '{data: false}'
```

Bridge gửi D, trạng thái sau đó cho biết ACK và mode MCU. Khi cần reset, xử lý
nguyên nhân, đợi bánh dừng và ACK của D rồi gọi:

```bash
ros2 service call /esp32/reset std_srvs/srv/Trigger '{}'
```

`/lift/reset` là alias cùng chức năng reset MCU, xóa homed; phải home lại.
Phản hồi service chỉ xác nhận **đã gửi R**. Chờ telemetry fault=0, DISARMED và
host=OK trước khi arm lại. Reset bị firmware từ chối sẽ không được coi là hoàn tất.

Theo lựa chọn đã có của dự án, **fault/mất liên lạc không tự ngắt PWM bánh**.
Khi telemetry/ACK/lệnh ROS mất quá hạn hoặc MCU reboot, bridge khóa phiên điều
khiển, ngừng tự gửi frame; không tự gửi D/C-disabled để thay đổi chính sách giữ
PWM của firmware. PWM thực tế có thể vẫn còn cho đến khi ngắt nguồn hoặc lệnh D
chủ động được nhận. `DISARMED` phía host khi lỗi **không xác nhận bánh đã dừng**.
Cơ cấu nâng vẫn có cơ chế dừng khi lỗi/watchdog ở firmware.

Dừng bằng Space, service disarm, E-stop ROS do người vận hành kích hoạt, hoặc
Ctrl+C launch là lệnh dừng chủ động, vẫn gửi D. Phản hồi GPIO E-stop của MCU được
báo lỗi theo chính sách firmware; E-stop ROS không thay cho mạch ngắt nguồn.

Nối lại telemetry không tự tiếp tục lệnh cũ. Cần dừng chủ động, reset nếu MCU có
fault rồi arm lại. Nếu USB bị rút, lỗi đọc/ghi hoặc ghi dở frame: khởi động lại
launch sau khi nối lại. Không tự reconnect hoặc mở khóa firmware.

`/esp32/status` và `/diagnostics` hiển thị boot ID, ACK, capability, fault và lỗi
host như `ACK_TIMEOUT`, `TELEMETRY_TIMEOUT`, `ROS_COMMAND_TIMEOUT`, `MCU_REBOOT`.
Lỗi kết nối không làm bridge phát lại odometry cũ thành dữ liệu mới.

## Khi báo NO_TELEMETRY trên Linux

Hai bên mặc định **230400 baud**. `invalid_frames=0` chỉ nói chưa có dòng lỗi được
đưa vào parser; riêng giá trị này chưa chứng minh cổng có nhận byte hay không.
Bridge hiện nhả **DTR trước, RTS sau** khi mở CP2102 và tắt RTS/CTS, HUPCL.
Các chân điều khiển USB có thể nối vào GPIO0/EN, giữ board reset/download trên
một số mạch; xem [tài liệu Espressif](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html).
Đây là xử lý một nguyên nhân có thể xảy ra, không thay cho chẩn đoán trên board.

Đóng Arduino Serial Monitor, chạy lại launch với bản bridge mới rồi xem:

```bash
ros2 topic echo /diagnostics --once
```

Trong mục `forklift_hardware/esp32`:

- `port`, `baud`: xác nhận đúng thiết bị và 230400.
- `modem_lines=released`: driver đã chấp nhận lệnh nhả DTR/RTS.
  `unsupported` dùng cho PTY hoặc thiết bị không hỗ trợ ioctl modem.
- `rx_bytes=0`: chưa nhận byte; kiểm tra đúng cổng, board boot/EN, cáp USB và
  Serial Monitor/phần mềm khác đang chiếm hoặc đọc cùng cổng.
- `rx_bytes` tăng, `rx_lines=0`: có byte nhưng thiếu newline hoàn chỉnh.
- `rx_lines` và `invalid_frames` tăng: có dòng, cần kiểm tra nội dung/giao thức/CRC.
- `discarded_frames` tăng: nhận dòng quá dài, bị bỏ đến newline tiếp theo.

Sau 5 giây chưa có telemetry hợp lệ, terminal launch sẽ báo các bộ đếm này.
Nếu `rx_bytes` vẫn bằng 0, nhấn EN/RESET một lần (không giữ BOOT), xem bộ đếm
có tăng không. GPIO12 nối encoder FR vẫn có thể ảnh hưởng boot; chỉ kiểm tra
thay đổi dây khi nguồn động cơ đã tắt. Bản sửa bridge này không cần nạp lại firmware.
