# Firmware ESP32 xe nâng

Bản khung cho **ESP32 DevKit 38 chân, WROOM**, 4 GA25-370 có encoder,
4 BTS7960 và TB6600/NEMA17. MPU6050 được chọn nối **Raspberry Pi qua I2C**
để dành GPIO ESP32 cho encoder và cơ cấu chấp hành. Chưa triển khai driver MPU6050
thật. Backend ROS USB/UART CSV đơn giản đã có ở `forklift_hardware`, xem
[hướng dẫn điều khiển ESP32](../../docs/esp32_manual_control.md).

Profile mặc định `encoder_spikes_warn_only=true`: bỏ mẫu encoder nhảy phi lý
và báo warning theo bánh, không khóa bộ điều khiển. Stall/encoder ngược chiều
cũng chỉ warning. Vẫn giữ lỗi khi bánh còn chuyển động trong lúc nâng/hạ và
các lỗi E-stop, I/O, công tắc, cấu hình, deadline. Cần nạp lại sketch để áp dụng;
bridge ROS không che `fault=8` từ firmware cũ đã FAULTED.

**Mặc định bật profile thử bánh + nâng một công tắc dưới. Chưa nạp hoặc thử xe thật.**
Theo yêu cầu dùng giá trị khởi đầu để tuning sau, `base_bench_profile=true`
cho phép khởi tạo GPIO bánh với PI dự kiến. Các cờ xác nhận điện vẫn `false`;
profile thử không có nghĩa đã kiểm chứng đấu dây/mức logic/strapping.
Khởi động PWM=0, STEP=0, chưa homed, mode `Disarmed`, capability **3** nếu HAL thành công;
nhận ngay lệnh `C,...,1`, không cần gửi `D` trước hoặc gửi lặp liên tục. Nếu khởi tạo lỗi,
firmware vẫn khóa/capability 0. Đặt `base_bench_profile=false` để trở về yêu cầu
đủ ba cờ xác nhận trước khi khởi tạo bánh.

| Thông số khởi đầu | Giá trị |
|---|---|
| PID FL/FR/RL/RR | Kp=0,08; Ki=0,12; Kd=0,05 |
| Đơn vị | Đầu vào rad/s, đầu ra duty −1…1, dt giây |
| Tần số vòng điều khiển | 100 Hz |
| Trần PWM | 50% |
| Vận tốc MCU / gia tốc bánh | 10 rad/s / 2 rad/s² |
| Giới hạn bánh phía ROS | 6 rad/s (0,30 m/s với bán kính 0,05 m) |
| CPR | 937,2 danh nghĩa |

Tuning tại `wheel_pi[4]` trong `board_config.h`, theo thứ tự **FL, FR, RL, RR**.
Đây là lựa chọn khởi đầu của dự án, không phải bộ GA25-370 đã được đo. Ví dụ
[PID motor chính thức của Espressif](https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control/main/mcpwm_bdc_control_example_main.c)
dùng thang đo encoder/PWM riêng, không sao chép trực tiếp hệ số sang rad/s/duty.
Kd=0 để bắt đầu bằng PI và tránh khuếch đại nhiễu tốc độ encoder.

`enable_lift=true`, `single_lower_bench_profile=true`: dùng một công tắc NC,
**COM → GND, NC → GPIO32**, `INPUT_PULLUP`, không nối tín hiệu lên 5 V.
Bình thường LOW; chạm đáy/hở mạch HIGH. Không có công tắc trên, `upper_pin=-1`;
không đọc GPIO âm. Phản hồi E-stop chưa gán trong profile thử thủ công này.

- F/H sau arm: hạ tối đa **3 giây**, hoặc dừng sớm khi công tắc dưới tác động.
  Không nhả/chạm lại; công tắc đã tác động thì dừng ngay. Hết thời gian là hoàn tất
  bình thường, không fault 32. Chỉ đặt bước về 0 và homed khi thực sự chạm công tắc.
- R trên ROS (U trên Serial): nâng **thêm 2.000 xung từ vị trí hiện tại**, không
  cần home. Bấm lại sau khi hoàn tất sẽ nâng thêm; giữ phím không xếp hàng lệnh.
- Không còn giới hạn tổng 2.000 xung. Timer chặn chính xác tại đích tương đối
  của mỗi lần nâng; profile không có công tắc trên nên cần quan sát hành trình.
- Tốc độ nâng/hạ tối đa 1000 xung/s, gia tốc 4000 xung/s².
- Space dừng STEP/hủy action. Lỗi phần cứng vẫn dừng STEP; giữ chính sách PWM bánh.

Quãng đường vật lý chưa biết. Giả định 200 bước/vòng, vi bước 1/8, lead 8 mm/vòng
chỉ tạo **thang tọa độ UART danh nghĩa 200000 step/m**. Giá trị 0,01 trong goal
hiện nghĩa là 2.000 xung, **chưa xác nhận nâng 1 cm**. ROS dùng cùng thang để đổi
tọa độ ra xung; status hiển thị `steps`, chưa coi đó là chiều cao đo thực. Chưa chọn dòng DIP
TB6600 khi không biết dòng pha NEMA17; cần chọn dòng phù hợp động cơ thật.
Xem [cách đo và hiệu chỉnh](../../docs/esp32_manual_control.md#đo-quãng-nâng-và-hiệu-chỉnh).
Đặt `enable_lift=false` và cấu hình ROS phù hợp nếu chỉ thử bánh.

**Lựa chọn mới của người dùng — ngắt nguồn thủ công cho bánh:**
`base_enable_hardwired_high=true`, `stop_base_on_fault=false` trong `board_config.h`.
R_EN/L_EN kéo HIGH ngoài, không dùng GPIO EN. Khi lỗi hoặc mất liên lạc, firmware
báo fault và **giữ PWM cuối của bốn bánh**, không tiếp tục PID với dữ liệu lỗi.
Riêng mất lệnh Serial không còn là lỗi: firmware tiếp tục PID theo vận tốc đặt cuối.
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
- PWM BTS7960 20 kHz, 10 bit; giới hạn 50% duty, MCU 10 rad/s, ROS 6 rad/s, chỉ là phạm vi
  thử bàn dự kiến. 280 RPM do người dùng cung cấp không được coi là vận tốc tải đo được.
- Xung STEP/DIR qua timer, giới hạn 5000 xung/s; giữ dòng nâng sau khi dừng.
  Biên dạng tốc độ/giảm tốc và chặn nâng đồng thời với chạy bánh.
- Profile hiện tại dùng **một công tắc NC dưới**, không yêu cầu công tắc trên.
  Hạ tối đa 3 s hoặc tới công tắc; nâng thêm một số xung, không cần home.
- CSV từng dòng đơn giản, kiểm tra kiểu số/giới hạn và bắt đầu disarm;
  không timeout lệnh Serial, không yêu cầu handshake disabled. Lỗi phần cứng
  vẫn được chốt và cần dừng/reset. Giao thức v2 cũ
  vẫn có thể chọn riêng để đối chiếu.
- Kiểm tra chu kỳ điều khiển quá 50 ms. Timer ISR chặn STEP khi trễ; latch yêu cầu
  reboot. Trong profile hiện tại, lỗi encoder/IO/deadline không tự ngắt PWM bánh.
  Các thời gian trên là thiết kế phần mềm, **chưa đo trên ESP32 thật**.

Chưa có đo dòng, xác nhận tải thật, cảm biến vị trí nâng, hoặc brake cơ khí.
`lift_steps` đếm xung đã phát, không phát hiện trượt bước hay xác nhận tiếp xúc tải.
Giữ dòng TB6600 không bảo đảm giữ tải khi mất điện. Cần xác minh cơ cấu vít me và
mạch dừng vật lý trước khi nâng tải; mã này không chứng nhận chức năng an toàn.

## Bảng chân người dùng cung cấp — chưa kiểm chứng điện

Thứ tự bánh: FL, FR, RL, RR nhìn theo hướng tiến của xe.

| Chức năng | GPIO ESP32 |
|---|---|
| FL RPWM / LPWM | 25 / 26 |
| FR RPWM / LPWM | 18 / 19 |
| RL RPWM / LPWM | 23 / 22 |
| RR RPWM / LPWM | 21 / 27 |
| FL encoder A / B | 34 / 35 |
| FR encoder A / B | 2 / 14 |
| RL encoder A / B | 5 / 17 |
| RR encoder A / B | 16 / 4 |
| R_EN và L_EN của mỗi BTS7960 | Kéo HIGH ngoài theo user, không dùng GPIO; EN pin `-1` |
| Phản hồi E-stop | Chưa có; `-1`, bỏ qua trong profile thử thủ công |
| Limit dưới / trên | GPIO32 (COM–GND, NC–GPIO32) / không có (`-1`) |
| TB6600 PUL+ / DIR+ | 13 / 33 |
| TB6600 PUL− / DIR− | GND / GND |
| TB6600 ENA | Chưa gán; phải xác minh cực tính và cách giữ dòng của module |
| USB–UART | TX0 GPIO1 / RX0 GPIO3; không dùng cho chức năng khác |

Hiểu nhãn user “RHW/LPW” là RPWM/LPWM, “DTR+” là DIR+. Mảng firmware giữ thứ tự
FL/FR/RL/RR; không thay thứ tự joint ROS theo thứ tự liệt kê FR/RR/FL/RL của user.
Đã đồng bộ cấu hình sau test từng bánh với `firmware/esp32_test`: đổi cặp PWM
FL/RL, `motor_sign={1,-1,1,-1}`, `encoder_sign={1,1,1,-1}`.
Encoder A của FR đã chuyển từ GPIO12 sang GPIO2; B vẫn GPIO14.
PCNT dùng HIGH=INVERSE, LOW=KEEP để cùng quy ước với bộ giải mã ngắt của bản test:
chuỗi AB `00 → 01 → 11 → 10 → 00` đếm +4 trước khi nhân `encoder_sign`.
Không giữ GPIO5/23/4/13 ở chức năng EN/E-stop/limit của bảng dự kiến cũ.

GPIO34/35 chỉ vào và không có pull-up/pull-down nội, cần mạch ngoài nếu encoder
yêu cầu kéo mức. FR A đã chuyển khỏi GPIO12 để tránh chọn sai điện áp flash lúc reset.
GPIO2 (FR A) và GPIO5 (RL A) vẫn là chân strapping; GPIO2 cần LOW hoặc thả nổi
khi vào chế độ nạp UART. Nếu encoder giữ GPIO2 HIGH và không nạp được, tắt nguồn
rồi tháo dây tín hiệu GPIO2 trong lúc nạp; không nối tắt ngõ ra encoder xuống GND.
[Giải thích boot của Espressif](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html).
GPIO32 hiện dành cho công tắc dưới; không dùng đồng thời làm FR encoder A.
`strapping_levels_reviewed=false`
vẫn ghi nhận chưa kiểm chứng; profile thử bánh hiện bỏ qua khóa xác nhận này,
không xử lý được mức điện lúc boot. Chỉ đặt cờ `true` sau khi kiểm chứng thực tế.
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
   Profile thử hiện chỉ dùng limit dưới GPIO32 và giới hạn xung, chưa có E-stop GPIO.

Profile thử bánh bỏ qua yêu cầu xác nhận điện theo lựa chọn người dùng, giữ bảng
chân đã cung cấp. Khi thử, kê bánh khỏi mặt đất, chưa mang tải; kiểm tra chiều
motor/encoder rồi mới tuning PI. Nếu đếm ngược, sửa `encoder_sign`/`motor_sign`,
không tăng gain để bù sai dấu. GPIO2/5 và mức điện vẫn cần xử lý như mục trên.
Profile nâng một công tắc dùng giả định số xung theo yêu cầu; đấu công tắc dưới,
kiểm tra chiều nâng/hạ, mức điện và dòng DIP trước khi thử không tải. Các bảo vệ
phần mềm và công tắc dưới không thay cho công tắc trên hoặc mạch dừng vật lý.

## Serial CSV đơn giản — mặc định

USB–UART **115200 baud, 8N1**. Mỗi bản tin ASCII kết thúc bằng newline, không CRC,
không boot ID, không sequence hoặc ACK trên dây. Serial Monitor chọn 115200 và
New Line (Both NL & CR cũng được). ESP32 tự phát trạng thái **20 lần/giây**:

```text
S,ms,tFL,tFR,tRL,tRR,lift_steps,mode,homed,lower,fault,capabilities,warnings
S,1200,0,0,0,0,0,1,0,1,0,3,0
```

- `ms`: thời gian ESP32 chạy, ms, dùng tính vận tốc từ số đếm encoder.
- `tFL…tRR`: encoder có dấu; `lift_steps`: xung nâng tính từ home.
- `mode`: 0 khóa, 1 disarmed, 2 active, 3 đang nâng/hạ (nhãn cũ HOMING), 4 fault.
- `homed`, `lower`: 0/1; `lower=1` chỉ nói input công tắc đang tác động, chưa
  tự xác nhận vị trí home cho tới khi lệnh hạ nhận công tắc.
- `fault`: bitmask 1 config, 2 E-stop, 8 encoder, 16 limit,
  32 home timeout, 64 deadline, 128 peripheral.
  Bit 4 của watchdog Serial cũ được để trống, không còn phát sinh.
- `capabilities`: 0 cấu hình lỗi, 1 bánh, 3 bánh+nâng.
- `warnings`: bitmask FL=1, FR=2, RL=4, RR=8. Bánh không quay dù PWM cao hoặc
  encoder ngược chiều lệnh trên 0,5 s chỉ phát warning, không latch fault hay
  chặn điều khiển. Warning tự hết khi điều kiện hết hoặc nhận D. Các lỗi encoder
  nhảy số phi lý và bánh còn chạy khi nâng vẫn là fault 8.

Trường `warnings` cũng được thêm cuối telemetry UART v2 trước checksum.
Bridge ROS mới đọc được bản tin cũ lẫn mới; cập nhật/build bridge trên Pi trước
khi dùng firmware mới. Phải nạp lại sketch chính để đổi stall từ fault sang
warning; chỉ cập nhật Python không đủ.

Các lệnh ROS gửi xuống:

```text
C,wFL,wFR,wRL,wRR,lift_steps,enabled
C,0,0,0,0,0,0
C,0,0,0,0,0,1
H
U
D
R
```

`C` đặt vận tốc bánh **rad/s**, trường lift để **0** trong profile thủ công,
enabled 0/1. `H` hạ tối đa 3 s hoặc tới công tắc, `U` nâng thêm 2.000 xung,
`D` dừng, `R` reset fault (khác phím R bàn phím nâng; node chuyển phím thành U).
Khởi động vẫn disarm/PWM=0; nhận ngay `C,...,1` mà không cần `D` trước.
Mỗi lệnh chỉ cần gửi một lần; PID tiếp tục chạy theo mục tiêu cuối cho đến
khi có lệnh mới hoặc `D`. `H` cũng không cần heartbeat. Bộ điều khiển vẫn
kiểm tra tốc độ, chiều encoder và đích xung của mỗi lần nâng. Lỗi phần cứng bánh
vẫn giữ PWM theo lựa chọn ngắt nguồn thủ công.

Thử trực tiếp bằng Serial Monitor: gửi `C,1,0,0,0,0,1` để chạy bánh trước trái
1 rad/s, gửi `D` để dừng. Không còn lỗi `fault=4` khi gửi bằng tay.
Đóng Serial Monitor trước khi dùng ROS. Mất kết nối Serial không tự dừng bánh.

Sketch được chia thành các hàm theo kiểu Arduino: `readSerialCommands()` đọc
lệnh không chặn, `updateMotors()` cập nhật PI mỗi 10 ms, `sendStatus()` gửi
trạng thái mỗi 50 ms; `setup()` và `loop()` điều phối các hàm này bằng `millis()`.
Cấu hình chân/gain nằm ở `board_config.h`, đọc encoder/PWM/STEP ở `board_hal.cpp`,
parser và PI/homing ở `firmware_core.cpp` để kiểm thử được trên máy tính.

Bản tin lỗi định dạng bị bỏ qua. Vì không có CRC/ID/ACK, định dạng này dễ xem
bằng Monitor nhưng không kiểm tra được lỗi bit tạo thành số hợp lệ hoặc xác
nhận riêng từng lệnh. ROS đợi telemetry mới và kiểm tra mode/vị trí/công tắc,
không coi việc gửi lệnh là hoàn tất. Uptime giảm được coi là MCU khởi động lại,
không tự tiếp tục nhiệm vụ.

**Phải cập nhật cả firmware và ROS cùng lúc**: `simple_serial=true` trong
`board_config.h` và `serial_protocol: simple` trong `esp32.yaml`. Bản firmware
v2 cũ (`S,2,...*CRC`) không được parser simple chấp nhận. Có thể dùng lại v2
bằng `simple_serial=false`, `serial_protocol: v2`; mã v2 giữ cho đối chiếu và
MCU PTY cũ vẫn riêng biệt. Backend ros2_control PTY v1 không đổi.

## Build và kiểm thử

Lõi C++17 chạy native trên máy phát triển (CMake, không cần GTest cho bộ test Serial):

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
