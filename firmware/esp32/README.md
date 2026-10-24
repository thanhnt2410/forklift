# Firmware ESP32 xe nâng

Bản khung cho **ESP32 DevKit 38 chân, WROOM**, 4 GA25-370 có encoder,
4 BTS7960 và TB6600/NEMA17. MPU6050 được chọn nối **Raspberry Pi qua I2C**
để dành GPIO ESP32 cho encoder và cơ cấu chấp hành. Chưa triển khai driver MPU6050
thật hoặc backend ROS serial thật trong bước này.

**Bản phân phối khóa đầu ra. Chưa nạp firmware, chưa thử bàn hoặc chạy xe thật.**
`board_config.h` dùng CPR danh nghĩa 937,2; PID và thông số nâng còn bằng 0; các cờ
xác nhận đấu dây/mức logic/strapping đều `false`. Firmware vẫn có thể phát chẩn đoán UART, nhưng không khởi tạo
GPIO chấp hành, báo `Configuration`, capability 0. Đây là bản để hoàn thiện và
kiểm thử phần mềm; không phải bản hiệu chuẩn cho xe hiện tại.

**Lựa chọn mới của người dùng — ngắt nguồn thủ công cho bánh:**
`base_enable_hardwired_high=true`, `stop_base_on_fault=false` trong `board_config.h`.
R_EN/L_EN kéo HIGH ngoài, không dùng GPIO EN. Khi lỗi hoặc mất liên lạc, firmware
báo fault và **giữ PWM cuối của bốn bánh**, không tiếp tục PID với dữ liệu lỗi.
`Faulted` không có nghĩa bánh đã dừng. Lệnh `D` hoặc `C enabled=0` vẫn đưa PWM về 0;
sau đó phải xử lý lỗi và reset trước khi nhận lệnh chạy mới. Cơ cấu nâng vẫn dừng
STEP khi lỗi và giữ homing/giới hạn. Chưa nạp hoặc kiểm chứng phần cứng.

## Thành phần đã có

- Bộ đếm PCNT cho 4 encoder quadrature A/B, mặc định giải mã x4 theo yêu cầu.
  User cung cấp 11 PPR và tỉ số truyền 21,3:1. Với cách hiểu PPR là chu kỳ trên
  một kênh/vòng trục motor, CPR danh nghĩa là **11 × 4 × 21,3 = 937,2** trên một
  vòng trục ra. Cần kiểm chứng quy ước PPR và số đếm thực; chiều encoder và motor độc lập.
- PID vận tốc từng bánh 100 Hz, chống tích phân bão hòa, ramp vận tốc, chờ bánh
  dừng khi đổi chiều. Khi vượt vận tốc đặt, PWM về 0; không đảo cực để phanh.
  Với EN kéo HIGH, PWM=0 không tương đương vô hiệu hóa cầu H; trạng thái phanh/thả
  phụ thuộc mạch driver, không coi đây là cách ly nguồn.
- PWM BTS7960 20 kHz, 10 bit; giới hạn ban đầu 25% duty, 2 rad/s, chỉ là phạm vi
  thử bàn dự kiến. 280 RPM do người dùng cung cấp không được coi là vận tốc tải đo được.
- Xung STEP/DIR qua timer, giới hạn 5000 xung/s; giữ dòng nâng sau khi dừng.
  Biên dạng tốc độ/giảm tốc và chặn nâng đồng thời với chạy bánh.
- Hai công tắc NC dự kiến: **dưới để homing và giới hạn dưới**, **trên giới hạn trên**.
  Homing tìm dưới → đi lên cho công tắc nhả → đi thêm khoảng backoff → chạm lại chậm.
  Không nhả công tắc hoặc không hoàn tất đúng thời gian sẽ lỗi.
- CRC32, ID phiên boot, số thứ tự tăng, handshake vô hiệu hóa trước khi cho phép
  chạy; chẩn đoán timeout lệnh 300 ms, fault latch và reset sau khi có lệnh dừng.
- Kiểm tra chu kỳ điều khiển quá 50 ms. Timer ISR chặn STEP khi trễ; latch yêu cầu
  reboot. Trong profile hiện tại, lỗi timeout/encoder/IO/deadline không tự ngắt PWM bánh.
  Các thời gian trên là thiết kế phần mềm, **chưa đo trên ESP32 thật**.

Chưa có đo dòng, xác nhận tải thật, cảm biến vị trí nâng, hoặc brake cơ khí.
`lift_steps` đếm xung đã phát, không phát hiện trượt bước hay xác nhận tiếp xúc tải.
Giữ dòng TB6600 không bảo đảm giữ tải khi mất điện. Cần xác minh cơ cấu vít me và
mạch dừng vật lý trước khi nâng tải; mã này không chứng nhận chức năng an toàn.

## Bảng chân người dùng cung cấp — chưa kiểm chứng điện

Thứ tự bánh: FL, FR, RL, RR nhìn theo hướng tiến của xe.

| Chức năng | GPIO ESP32 |
|---|---|
| FL RPWM / LPWM | 23 / 22 |
| FR RPWM / LPWM | 18 / 19 |
| RL RPWM / LPWM | 25 / 26 |
| RR RPWM / LPWM | 21 / 27 |
| FL encoder A / B | 34 / 35 |
| FR encoder A / B | 12 / 14 |
| RL encoder A / B | 5 / 17 |
| RR encoder A / B | 16 / 4 |
| R_EN và L_EN của mỗi BTS7960 | Kéo HIGH ngoài theo user, không dùng GPIO; EN pin `-1` |
| Phản hồi E-stop | Chưa có; `-1`, được bỏ qua ở chế độ base-only ngắt nguồn thủ công |
| Limit dưới / trên | Chưa cung cấp; `-1` / `-1` |
| TB6600 PUL+ / DIR+ | 13 / 33 |
| TB6600 PUL− / DIR− | GND / GND |
| TB6600 ENA | Chưa gán; phải xác minh cực tính và cách giữ dòng của module |
| USB–UART | TX0 GPIO1 / RX0 GPIO3; không dùng cho chức năng khác |

Hiểu nhãn user “RHW/LPW” là RPWM/LPWM, “DTR+” là DIR+. Mảng firmware giữ thứ tự
FL/FR/RL/RR; không thay thứ tự joint ROS theo thứ tự liệt kê FR/RR/FL/RL của user.
Không giữ GPIO5/23/4/13 ở chức năng EN/E-stop/limit của bảng dự kiến cũ.

GPIO34/35 chỉ vào và không có pull-up/pull-down nội, cần mạch ngoài nếu encoder
yêu cầu kéo mức. GPIO12 (FR A) và GPIO5 (RL A) là chân strapping. GPIO12 HIGH lúc
reset có thể chọn VDD_SDIO 1,8 V thay vì 3,3 V và làm WROOM không boot.
[Giải thích boot của Espressif](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html).
Nên chuyển FR A từ GPIO12 sang GPIO32 đang trống nếu có thể đổi dây; code vẫn
ghi GPIO12 đúng sơ đồ user, chưa tự đổi chân. `strapping_levels_reviewed=false`
chặn cho phép chạy; chỉ xác nhận sau khi xử lý mức boot thực tế.
Firmware không thể ngăn tín hiệu của boot ROM trước `setup()`. Với EN kéo HIGH,
cần giữ RPWM/LPWM ở LOW bằng mạch phù hợp lúc boot/reset, không để ngõ vào trôi;
khóa phần mềm không cách ly được driver đã được bật bằng dây ngoài.
Tín hiệu STEP cũng cần trạng thái nghỉ xác định.
Không dùng GPIO6–11 của flash trong profile này.
[Giới hạn GPIO chính thức của Espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html).

Chưa xác minh điện áp encoder và mạch input trên các module BTS7960/TB6600 đang
dùng. Không suy ra khả năng nhận 3,3 V của cả module từ tên IC. Các module TB6600
có thể dùng giao diện khác nhau: xem ví dụ
[DFRobot DRI0043](https://wiki.dfrobot.com/dri0043/) và
[Joy-IT](https://joy-it.net/files/files/Produkte/SBC-MD-TB6600/SBC-MD-TB6600_Datasheet-EN_2024-04-04.pdf).
Phải xác nhận hoặc bổ sung chuyển mức/tầng đệm trước khi bật cờ cấu hình.

Thiết kế input NC dự kiến: tiếp điểm khỏe nối về GND, hở/nhấn đọc HIGH.
E-stop cần mạch dừng vật lý độc lập; GPIO là phản hồi bổ sung. Cần xác minh nhiễu,
dội tiếp điểm và thời gian dừng khi thử bàn. Chưa có lọc dội công tắc trong profile.

## Thông số còn thiếu

1. Mức điện áp encoder và kiểm chứng **số đếm x4 trên một vòng trục ra hộp số**.
   Đã cấu hình danh nghĩa 937,2 từ 11 PPR user cung cấp, chưa phải số đo hiệu chuẩn.
   Nếu tài liệu ghi PPR ở trục motor, phải làm rõ số cạnh và tỉ số truyền.
   Có thể đo bằng cách đánh dấu bánh và đếm khi quay tay; không lấy 4096 từ MCU giả.
2. NEMA17: góc bước, dòng pha; TB6600: mức vi bước và dòng DIP.
   Bước vít me: **quãng dịch chuyển trên một vòng**, không nhầm bước ren với lead.
   `steps_per_m = full_steps_per_rev × microsteps / lead_m_per_rev` nếu truyền trực tiếp.
3. Hành trình hữu dụng, backoff homing, tốc độ/gia tốc nâng và timeout đo được.
   Không lấy 10000 step/m hoặc hành trình CAD làm hiệu chuẩn thật.
4. Mạch nguồn, tầng đệm, HIGH cho EN và ngắt nguồn thủ công; PID và dấu từng bánh.
   Nâng vẫn cần E-stop và 2 limit được gán/xác minh trước khi bật.

Chỉ bật `base_ready` khi wiring/mức logic, CPR và PID đã có; cơ cấu nâng có cờ riêng.
Không bật nâng bằng cách chỉ đổi `enable_lift` khi các trường nâng còn 0.

## UART v2

USB–UART, 230400 baud, 8N1; mỗi frame ASCII:

```text
PAYLOAD*crc32_8_hex\n
C,2,boot,seq,wFL,wFR,wRL,wRR,lift_m,enabled
H,2,boot,seq
D,2,boot,seq
R,2,boot,seq
S,2,boot,state_seq,time_us,ack,tFL,tFR,tRL,tRR,lift_steps,fault,mode,homed,low,high,estop,capabilities
```

CRC32/ISO-HDLC tính trên PAYLOAD, không gồm `*`, CRC, newline. Không có CR.
Tối đa 256 byte/frame kể cả LF. Vận tốc bánh rad/s, vị trí nâng m, timestamp µs
64 bit từ MCU; tick encoder là số đếm x4 có dấu, không phải vị trí mô phỏng.
`boot` là ID boot 64 bit ngẫu nhiên phát trong S, host phải đọc trước khi gửi lệnh.
`seq` > 0 và tăng nghiêm ngặt; frame sai boot/lặp/cũ không ACK, không nuôi watchdog.
Đây là chống nhầm phiên/lệnh cũ, không phải cơ chế xác thực bảo mật.

| Lệnh/trạng thái | Nghĩa |
|---|---|
| C, enabled=0 | Vô hiệu hóa; trong homing giữ vai trò heartbeat để tiếp tục H |
| C, enabled=1 | Chấp nhận sau handshake disabled và không lỗi; nâng cần homed |
| H | Chỉ từ Disarmed, bánh dừng; tìm home, cần tiếp tục C disabled đều đặn |
| D | Dừng/hủy, kể cả homing; không tự xóa lỗi |
| R | Reset khi input khỏe và bánh đã dừng; xóa homed, phải home lại trước nâng |
| mode | 0 Locked, 1 Disarmed, 2 Active, 3 Homing, 4 Faulted |
| capabilities | 0 cấu hình chưa đủ, 1 hỗ trợ base, 3 hỗ trợ base+nâng; xem fault/mode để biết sẵn sàng |
| fault bitmask | 1 Config, 2 E-stop, 4 watchdog, 8 encoder, 16 limit, 32 home timeout, 64 control deadline, 128 peripheral |

Sau fault, C enabled mới bị từ chối. Profile ngắt nguồn thủ công vẫn giữ PWM cũ,
nên host cần gửi D/C-disabled để dừng trước khi R có thể được chấp nhận. Với lỗi
khôi phục được, kiểm tra ACK/trạng thái sau R trước khi gửi lệnh tiếp. Timer deadline latch hoặc
khởi tạo peripheral hỏng yêu cầu reboot sau xử lý nguyên nhân.
Vận tốc/lift ngoài giới hạn bị từ chối nguyên frame. Bản base-only cho phép thử
bánh khi chưa homing với lift target = 0. Giá trị setpoint vượt khả năng tải thực
không được tự hiệu chỉnh bằng thông số 280 RPM.

**Không tương thích trực tiếp với UART v1 của `PtySystem`/MCU giả.** Backend đó
vẫn chỉ mở `/dev/pts/*`, được giữ nguyên cho SIM. Cần backend serial v2 và bringup
REAL riêng trước khi kết nối ros2_control; chưa có trong thay đổi này.

## Build và kiểm thử

Lõi C++17 chạy native trên máy phát triển (CMake, GTest):

```bash
cmake -S firmware/esp32 -B build/esp32_native -DCMAKE_BUILD_TYPE=Debug
cmake --build build/esp32_native -j4
ctest --test-dir build/esp32_native --output-on-failure
```

Có thể bật `-DFIRMWARE_SANITIZE=ON` cho ASan/UBSan. Trong môi trường ptrace của
phiên phát triển, chạy build và test với `ASAN_OPTIONS=detect_leaks=0`; LeakSanitizer
không chạy được trong môi trường này, không coi đó là kiểm tra leak đã đạt.

Biên dịch sketch bằng Arduino CLI và core **esp32:esp32 3.3.8** đã cài:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
  --build-path "$PWD/build/esp32" firmware/esp32/forklift_esp32
```

Không có lệnh upload trong tài liệu hoặc script build. Kiểm thử native chỉ kiểm
tra logic lõi; không kiểm chứng PCNT, tín hiệu PWM/STEP, dừng tải, logic voltage hoặc
timing thực trên board. API dùng theo tài liệu
[PCNT](https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32/api-reference/peripherals/pcnt.html),
[LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html),
[timer](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html).

Bằng chứng build, kiểm thử và hash nguồn tại
[`results/esp32_firmware`](../../results/esp32_firmware/README.md).
Không chạy thêm kiểm thử tránh vật cản hoặc chu trình Gazebo trong bước firmware.
