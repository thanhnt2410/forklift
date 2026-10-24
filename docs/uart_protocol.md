# P3 — Giao thức MCU giả qua PTY, phiên bản 1

Phạm vi hiện tại: **chỉ mô phỏng**. `forklift_hardware/PtySystem` chỉ chấp nhận
`/dev/pts/<số>` do `mock_mcu.py` tạo. Không mở `/dev/ttyUSB*`, `/dev/ttyACM*`
hoặc UART GPIO. Chưa có firmware ESP32 hay cấu hình pin/baud cho robot thật.

## Framing

ASCII, một frame kết thúc LF. Dạng `payload*CRC32\n`, checksum 8 ký tự hex,
CRC-32/ISO-HDLC tương thích `zlib.crc32`. Tối đa 256 byte/frame; parser có buffer
giới hạn, nhận được frame bị chia nhỏ hoặc nhiều frame trong một lần đọc.

Lệnh host → MCU giả:

```text
C,1,command_seq,w_fl,w_fr,w_rl,w_rr,lift_target_m,enabled*crc32\n
```

Phản hồi MCU giả → host:

```text
S,1,boot_id,state_seq,mcu_time_us,ack_command_seq,t_fl,t_fr,t_rl,t_rr,lift_steps,fault*crc32\n
```

- Bánh luôn theo thứ tự **FL, FR, RL, RR**. Tốc độ rad/s, encoder tích lũy có dấu.
- `lift_target_m`: mét, miền vận hành mô phỏng 0–0,25; `lift_steps` là số bước giả
  quy đổi theo `lift_steps_per_m`, chưa phải encoder tuyến tính thật.
- `enabled=0`: không chạy bánh và giữ mục tiêu nâng tại vị trí hiện tại ở MCU giả.
- `boot_id` đổi khi reset; `state_seq` tăng mỗi phản hồi, `command_seq` tăng mỗi lệnh.
- Frame sai CRC, độ dài, version, số trường hoặc dữ liệu không hợp lệ bị từ chối.
- STATE trùng/cũ không gia hạn thời hạn phản hồi; reboot được phát hiện và giữ lỗi.

## Chu kỳ và lỗi

- Lệnh và STATE: khoảng 50 Hz; controller manager 100 Hz.
- Worker serial dùng I/O nonblocking; `read()`/`write()` của controller chỉ truy cập
  cache được bảo vệ bằng critical section ngắn, không chờ UART.
- Host không nhận STATE hợp lệ trong 250 ms: trả lỗi hardware, vô hiệu lệnh.
- Cache lệnh host không cập nhật trong 150 ms: worker gửi lệnh disabled.
- MCU giả không nhận lệnh mới hợp lệ trong 300 ms: watchdog đưa motor về dừng.
- Lỗi kết nối, frame hỏng kéo dài hoặc boot đổi được giữ; dữ liệu trở lại không
  tự phát lại lệnh cũ. Bản P3 được nghiệm thu theo quy trình restart bringup chủ động.
- Diagnostic `forklift_hardware/pty` và `/mock_mcu/status` chỉ rõ trạng thái ảo.

## Mô hình được dùng

Motor giả là hệ bậc một với hằng số thời gian 0,08 s. Encoder giả 4096 counts/vòng;
nâng giả 10000 bước/mét. Đây là giá trị thử giao thức, không phải thông số GA25A370
hoặc NEMA23 đã hiệu chuẩn. Odometry đọc chênh encoder theo thời gian MCU,
không tích phân trực tiếp tốc độ đặt.

PTY không mô phỏng băng thông, nhiễu điện hoặc mức logic UART thật. Trước khi làm
REAL phải tính lại thông lượng theo độ dài frame cực đại/tần số, chọn baud phù hợp,
đo CPR thực, bổ sung firmware/PID, homing và mạch an toàn theo P2/P5.

## Tạo lỗi để kiểm tra

Trong phiên `mock_hardware.launch.py`:

```bash
ros2 service call /mock_mcu/drop_telemetry std_srvs/srv/SetBool '{data: true}'
ros2 service call /mock_mcu/corrupt_telemetry std_srvs/srv/SetBool '{data: true}'
ros2 service call /mock_mcu/freeze_encoders std_srvs/srv/SetBool '{data: true}'
ros2 service call /mock_mcu/reset std_srvs/srv/Trigger '{}'
ros2 service call /mock_mcu/disconnect std_srvs/srv/Trigger '{}'
```

Các lệnh này chỉ tác động MCU giả của phiên hiện tại. Dùng `false` để ngừng drop,
corrupt hoặc freeze; fault hardware đã phát sinh vẫn cần quy trình phục hồi chủ động.
