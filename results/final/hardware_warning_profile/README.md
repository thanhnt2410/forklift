# Xe thật: chuyển các điều kiện phụ thành warning

Ngày thực hiện: 2026-10-28. Chỉ sửa source/build; không gửi lệnh hoặc nạp firmware vào xe thật.

## Thay đổi và phạm vi

| Điều kiện | Xử lý hiện tại |
|---|---|
| Càng đứng yên cao hơn 2 cm | `real.launch.py` mặc định warning, vẫn cho lái; có thể bật lại bằng `enforce_transport_height:=true` |
| Chưa home trong profile pulse | Đã cho lái/nâng từ trước; kiểm thử lại, không giả lập `homed=true` |
| Bánh không có xung hoặc encoder ngược chiều | Warning theo bánh đã có từ trước; vẫn cho nhận lệnh mới |
| Xung encoder nhảy phi lý | Firmware xe thật bỏ mẫu, warning; không latching fault 8 |
| Xung nhảy đến bridge | Bỏ riêng mẫu lỗi trước tích phân odometry, giữ tốc độ hợp lệ trước đó; baseline raw được cập nhật để lần sau phục hồi |
| Hạ hết 3 giây chưa chạm đáy ở profile pulse | Đã dừng hạ mà không fault từ trước, vẫn chưa homed |
| Bánh thực sự quay khi nâng/hạ | Vẫn lỗi; fault 8 chưa bị che toàn bộ |
| Đang nâng/hạ, mất/cũ trạng thái, E-stop, lệnh sai, thiếu hiệu chuẩn điều khiển vị trí | Vẫn chặn |
| Giới hạn tốc độ/PWM, lỗi cấu hình/I/O/deadline/công tắc, timeout ROS/telemetry/ACK | Giữ nguyên |

Firmware mặc định `encoder_spikes_warn_only=true` trong `board_config.h`; cấu hình
lõi chung mặc định false để vẫn có chế độ strict. Ngưỡng MCU là
`2 * max_wheel_rad_s + 5 = 25 rad/s`, bridge dùng `encoder_velocity_limit: 25.0`.
MCU giữ warning xung nhảy ít nhất 0,5 giây để truyền ở 20 Hz; warning không phải
fault. `wheel_warning=8` là bánh RR, khác `fault=8`.

Chính sách giữ PWM khi lỗi/mất liên lạc đã có trong dự án không thay đổi. Không
coi việc ROS ngừng nhận lệnh là đã ngắt PWM; Stop/Space/E-stop phần mềm vẫn gửi
D chủ động nếu đường truyền còn hoạt động. Xem tài liệu điều khiển ESP32.
Odometry bị thiếu mẫu khi bỏ xung lỗi; không khẳng định định vị chính xác khi
encoder hỏng. Không tăng trần PWM/tốc độ và không bỏ kiểm tra cảm biến của Nav2.
Mô phỏng giữ điều kiện chiều cao cũ.

## Kết quả kiểm thử

- Build `forklift_control`, `forklift_hardware`, `forklift_bringup`: **3/3 đạt**.
- Python: **128/128 đạt**, gồm command gate, bridge filtering/protocol, serial
  auto-selection và hồi quy mission/docking/lift/navigation guard.
- Lõi C++ firmware: **45/45 đạt**, với AddressSanitizer + UndefinedBehaviorSanitizer.
  Tắt LeakSanitizer (`ASAN_OPTIONS=detect_leaks=0`) vì sandbox dùng ptrace;
  ASan/UBSan vẫn hoạt động. Đồng bộ các kỳ vọng test local đã cũ về watchdog,
  handshake và homing theo hành vi có sẵn trước thay đổi này.
- ROS + PTY: **9/9 đạt**, domain 214, chỉ serial giả lập. Chạy các node
  `command_gate` và `esp32_bridge` thật; bên MCU của bài này là bộ phát CSV giả lập,
  không phải ESP32. Lõi MCU được kiểm thử riêng ở bộ C++ phía trên.
- CLI launch đã nhận `enforce_transport_height`, mặc định false; cú pháp Python
  và `git diff --check` đạt.

Chín kiểm tra runtime: tự arm chưa home, lái chưa home, càng cao vẫn lái,
wheel warning 8 vẫn lái, xung lỗi không làm odometry nhảy, xung lỗi chỉ warning
và tiếp tục nhận lệnh, encoder phục hồi, E-stop phần mềm dừng/disarm, lỗi MCU cũ
không bị che. Dòng ERROR MCU_FAULT_8 cuối runtime.log được tiêm có chủ đích
cho kiểm tra tương thích firmware cũ và bài kiểm tra đã đạt.

Chưa thử trên xe thật, chưa biên dịch/nạp image ESP32 bằng Arduino toolchain.
Các test source local vẫn được giữ ngoài Git theo .gitignore có sẵn.

## Áp dụng

1. Nạp lại sketch `firmware/esp32/forklift_esp32/forklift_esp32.ino` cùng các file
   bên cạnh vào ESP32. Chỉ restart ROS không thay đổi lỗi MCU đã latching.
2. Dừng launch cũ, mở terminal ở workspace đã cập nhật và chạy:

   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch forklift_bringup real.launch.py
   ```

   Package ROS tại workspace này đã build. Nếu chép source sang Pi khác, cần
   build các package trên Pi trước khi source/relaunch.
3. Dùng bàn phím như trước. Càng cao phát `LIFT_HEIGHT_WARNING`, xung encoder lỗi
   phát `ENCODER_SPIKE_WARNING` hoặc `WHEEL_TRACKING_WARNING`; không cần reset
   vì các warning đó. Lỗi đã có ở firmware cũ chỉ hết sau reset hợp lệ/reboot
   và cập nhật firmware; không tự arm lại sau lỗi.

Log gốc và kết quả chi tiết nằm cạnh báo cáo. `runtime_check.py` là bộ kiểm tra
cục bộ dùng PTY; không mở USB thật. Để chạy lại cần source ROS/workspace và đặt
ROS_DOMAIN_ID riêng, ví dụ 214, ROS_LOCALHOST_ONLY=1.
