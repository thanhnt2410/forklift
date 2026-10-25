# Điều khiển bàn phím qua ESP32 USB/UART

Backend này nối ROS với **Serial CSV đơn giản của firmware trong repo**, không cần Gazebo.
Đã kiểm tra phần mềm qua PTY với lõi C++ của firmware; chưa nạp firmware hoặc
kiểm chứng trên xe thật. Firmware hiện bật **profile thử bánh** theo yêu cầu dùng
PID hiện tại để tuning: Kp=0,08, Ki=0,12, Kd=0,05; PWM tối đa 50%, MCU tối đa
10 rad/s, phía ROS giới hạn 6 rad/s cho lệnh tịnh tiến 0,30 m/s.
Firmware khởi động disarm; ROS launch mặc định tự arm sau khi kiểm tra telemetry
và nhận zero từ command gate, không cần gọi service. Serial Monitor có thể gửi thẳng
`C,...,1`, không cần gửi `D` trước hoặc gửi lệnh lặp. Profile không xác nhận mức điện/đấu dây.
Nâng hiện dùng **một công tắc NC dưới GPIO32**, điều khiển theo số xung, chưa
hiệu chuẩn quãng đường thực. Không có công tắc trên hoặc phản hồi E-stop GPIO.


## Định dạng Serial hiện tại

Hai phía dùng `serial_protocol: simple` / `simple_serial=true`. Không có CRC,
boot ID hoặc ACK trên dây. Phản hồi đọc được bằng Monitor 115200:

```text
S,ms,tFL,tFR,tRL,tRR,lift_steps,mode,homed,lower,fault,capabilities,warnings
S,1200,0,0,0,0,0,1,0,1,0,3,0
```

Lệnh `C,wFL,wFR,wRL,wRR,0,enabled`, `H` hạ tối đa 3 s/công tắc,
`U` nâng thêm 2.000 xung, `D` dừng, `R` reset;
mỗi lệnh một dòng. Firmware phát trạng thái 20 Hz ngay khi boot, không cần ROS.
Sau khi pull bản chuyển giao thức, **nạp lại sketch ESP32 và restart ROS**; không
trộn bản firmware v2 cũ với bridge simple. Giữ 115200 ở cả hai phía.

### Thử bánh trực tiếp bằng Serial Monitor

Chọn **115200 baud**, kết thúc dòng **Newline** hoặc **Both NL & CR**. Không chạy
ROS bridge đồng thời vì cùng dùng cổng Serial. Gửi một lần:

```text
C,1,0,0,0,0,1
```

Bánh trước trái nhận vận tốc 1 rad/s. Thứ tự bốn bánh là FL, FR, RL, RR;
trường kế tiếp để 0 trong profile nâng thủ công, cuối cùng enabled=1.
Gửi `D` để dừng; `C,0,0,0,0,0,0` cũng dừng khi không đang home.
Không cần gửi D trước, không cần home để thử bánh với đích nâng 0.
Firmware tiếp tục PID theo lệnh cuối, **không còn timeout Serial 300 ms**;
ngừng gửi/rút USB không tự dừng bánh. Lệnh `H` cũng chỉ cần gửi một lần.
`R` trên Serial là reset lỗi, khác phím R nâng trong chương trình bàn phím ROS.

Telemetry giữ nguyên: tính cả chữ S, trường thứ 8 là mode (1 dừng, 2 chạy,
3 đang nâng/hạ (nhãn cũ HOMING), 4 lỗi), trường thứ 11 là fault (0 không lỗi). Bit lỗi 4 của watchdog
Serial cũ không còn dùng. Khi có lỗi phần cứng, gửi D, đợi bánh dừng, xử lý
nguyên nhân rồi R; các kiểm tra encoder/công tắc/giới hạn nâng vẫn hoạt động.

### Cảnh báo bánh không quay ở tốc độ thấp

Bánh có lệnh trên 0,5 rad/s, PWM trên 20% nhưng encoder dưới 0,05 rad/s, hoặc
encoder ngược chiều lệnh, kéo dài trên 0,5 giây sẽ tạo **warning**, không còn
chuyển MCU sang FAULTED hay tự disarm. PI tiếp tục chạy và nhận lệnh mới, nên có
thể tăng/giảm tốc hoặc gửi dừng mà không reset. Warning tự hết khi điều kiện
không còn hoặc nhận lệnh dừng chủ động.

Trường cuối `warnings` là bitmask bánh: FL=1, FR=2, RL=4, RR=8; cộng các bit nếu
nhiều bánh cùng cảnh báo. Bridge in `WHEEL_TRACKING_WARNING [tên bánh]`, thêm
`wheel_warning=...` vào `/esp32/status`, và đặt `/diagnostics` mức WARN.
Warning không khóa command gate và không được đưa vào `fault`.

`MCU_FAULT_8` vẫn được dùng cho số đếm encoder nhảy tới vận tốc phi lý hoặc
phát hiện bánh còn chạy khi đang nâng; đây là các điều kiện khác với thiếu
mô-men khởi động. Không bỏ qua bit lỗi 8 từ firmware cũ ở phía ROS vì MCU cũ
đã tự khóa nhận lệnh.

Để áp dụng: cập nhật/build bridge trên Pi **và nạp lại firmware chính**
`firmware/esp32/forklift_esp32/forklift_esp32.ino` lên ESP32. Chỉ restart ROS
không thay đổi logic MCU. Bridge mới đọc được cả telemetry cũ (không có trường
warning) và mới; firmware mới cần bridge mới. Thay đổi này không tăng trần PWM
hoặc tự bù mô-men khởi động; tốc độ rất thấp vẫn cần tuning PID/phần cứng.

## Luồng điều khiển

```text
arrow_teleop → /cmd_vel_teleop → command_gate → /cmd_vel
                                             ↓
                                       esp32_bridge
                                 C / H / U / D / R dạng CSV
                                             ↓
                                ESP32: PID bánh + STEP/DIR
                                             ↓
                          encoder / trạng thái → ROS
```

Bridge tính vận tốc bánh theo thứ tự **FL, FR, RL, RR**, nhận số đếm encoder,
phát `/joint_states`, `/odom` và tùy chọn TF `odom → base_link`. Odometry này chỉ
sử dụng encoder, **chưa kết hợp MPU6050 thật**. Launch hiện chỉ phục vụ điều khiển
thủ công; không khởi chạy Nav2, Gazebo, ros2_control hoặc supervisor nâng SIM.
ESP32 đảm nhiệm PID, hạ theo thời gian/công tắc và tạo xung nâng; bridge cung cấp cùng các
action `/lift/home`, `/lift/move` và service `/lift/stop` cho bàn phím.
Không chạy launch SIM, node tay cầm hay backend khác cùng ROS domain với launch này.

## Cấu hình

[esp32.yaml](../src/forklift_hardware/config/esp32.yaml) dùng baud **115200, 8N1**.
Hình học CAD và CPR **937,2** là giá trị ban đầu, chưa phải hiệu chuẩn xe thật.
`max_wheel_speed: 6.0` phải khớp hoặc thấp hơn giới hạn trong firmware (10 rad/s).
Bàn phím mặc định tiến/lùi/đi ngang 0,30 m/s; bánh bán kính 0,05 m cần 6 rad/s
để nhận đủ lệnh này khi tịnh tiến. Cần cập nhật YAML, nạp firmware và khởi động
lại bridge lẫn bàn phím để áp dụng; tốc độ thực còn phụ thuộc PI, tải và trần PWM.
Với bánh bán kính 0,05 m, chuyển động thẳng bị giới hạn khoảng **0,30 m/s**;
chuyển động kết hợp được scale chung bốn bánh để giữ hướng vận tốc.

Firmware dùng dấu `motor_sign` và `encoder_sign` trong
[board_config.h](../firmware/esp32/forklift_esp32/board_config.h). Không đảo dấu
lần nữa trong bridge. Kiểm tra chiều từng bánh khi hoàn tất cấu hình thử bàn.
Sau test từng motor, cấu hình đã đồng bộ với `firmware/esp32_test`: PWM theo thứ tự
FL/FR/RL/RR là `25/26`, `18/19`, `23/22`, `21/27`; dấu motor `{1,-1,1,-1}`,
dấu encoder `{1,1,1,-1}`. Chân encoder là `34/35`, `2/14`, `5/17`, `16/4`;
encoder A của FR đã chuyển từ GPIO12 sang GPIO2.

Profile mặc định là `lift_preset_mode: true`, `lift_raise_steps: 2000`,
`lift_max_steps: 2000`. Giá trị max này giữ thang goal ROS cũ, không phải giới hạn
tổng số xung nâng. Firmware và ROS phải dùng cùng `lift_raise_steps`.
Nối **COM công tắc → GND, NC → GPIO32**. Input kéo lên 3,3 V nội; trạng thái
bình thường LOW, chạm đáy/hở HIGH. Không nối GPIO32 lên 5 V.

Sau khi arm có thể lái và nâng ngay, không cần home trước.
**F hoặc H** hạ tối đa 3 giây, hoặc dừng sớm khi công tắc tác động. Nếu công tắc
đã tác động thì không hạ. Không còn pha đi lên nhả công tắc/chạm lại, không báo
fault 32 vì hết 3 giây. **R** nâng thêm 2.000 xung từ vị trí hiện tại mỗi lần bấm;
nhả rồi bấm lại để nâng thêm. Giữ phím không tự xếp hàng các lần nâng.
Space/S dừng giữa chừng. Không tự chạy càng khi vừa mở kết nối.
Chỉ khi hạ chạm công tắc mới đặt bước về 0 và `homed=1`; dừng do hết thời gian
giữ `homed=0`. Thiếu/hở dây công tắc NC vẫn đọc là đang chạm và sẽ không hạ.
Không có công tắc trên hoặc giới hạn tổng hành trình trong profile này; mỗi lần
nâng thêm là một thao tác tương đối, cần quan sát hành trình cơ khí.

Giá trị `lift_steps_per_m: 200000.0`, `lift_max_height: 0.01` hiện là **quy đổi
tọa độ UART danh nghĩa**, không xác nhận chiều cao 1 cm. Bridge kiểm tra hoàn tất
R bằng **tăng đúng 2.000 xung so với lúc bắt đầu**, F/H bằng MCU về mode dừng.
Kết quả hạ là `LOWER_LIMIT` hoặc `LOWER_TIME_DONE`; tăng xong là `RAISED`.
Quãng đường vẫn chưa hiệu chuẩn; `/lift/state.height` và
`/joint_states` phần nâng vẫn là khoảng cách ước tính từ thang danh nghĩa.

Bridge phát `/lift/preset_target` để bàn phím tự nhận chế độ; không cần đổi lệnh
chạy bàn phím. Với SIM không có topic này, R/F vẫn nâng/hạ từng nấc như trước.
Trong pulse mode, action `/lift/move` nhận goal 0 (hạ tối đa 3 s/công tắc) hoặc
goal preset (nâng thêm một lần); goal ở giữa bị từ chối. `/lift/home` hiện là
lệnh hạ theo thời gian/công tắc, không bảo đảm đã về gốc nếu chỉ hết thời gian.

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
cd /home/thanh/workspace/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to forklift_bringup
source install/setup.bash
ls -l /dev/serial/by-id/
ros2 launch forklift_bringup real.launch.py
```

`port` mặc định là `auto`: trên Pi hoặc laptop, bridge dò các cổng USB serial,
bỏ qua cổng đang được tiến trình khác dùng (kể cả driver LiDAR), rồi chỉ đọc
telemetry theo giao thức firmware đã cấu hình. Hai bản tin hợp lệ với timestamp
tăng mới xác nhận ESP32; không gửi lệnh điều khiển để dò và không dựa riêng vào
tên CP2102. Mỗi cổng trống được nghe tối đa 3 giây. Nếu không tìm thấy hoặc có
nhiều ESP32 chạy firmware này, launch dừng và yêu cầu chỉ định cổng.
Auto chỉ dò lúc khởi động, không tự reconnect sau khi rút/cắm USB; chạy lại
launch để dò tên cổng mới. Không tự arm lại sau khi nối USB.

Chế độ auto cần `fuser` từ gói `psmisc` (`sudo apt install psmisc` nếu thiếu).
Đóng launch bridge cũ và Serial Monitor trước khi chạy; nên giữ driver LiDAR
chạy để cổng của nó được bỏ qua. Mở cổng USB có thể reset một số board như khi
chỉ định cổng thủ công. Sau nhận diện, bước auto arm riêng của bridge chờ
telemetry hợp lệ, bánh đứng yên, handshake và lệnh zero mới từ command gate.

Muốn chỉ định thủ công, dùng
`ros2 launch forklift_bringup real.launch.py port:=/dev/serial/by-id/TEN_THIET_BI_ESP32`,
thay tên thiết bị bằng đường dẫn thực; cũng có thể dùng `/dev/ttyUSB0` nếu đó đúng là
ESP32. Tài khoản cần quyền đọc/ghi cổng serial. Bridge chiếm độc quyền cổng;
đóng Serial Monitor trước khi launch. Một số board reset khi mở USB serial;
bridge đợi telemetry mới rồi mới cho arm; uptime giảm sẽ khóa phiên và yêu cầu dừng/arm lại.

Muốn dùng cấu hình hiệu chuẩn riêng:

```bash
ros2 launch forklift_bringup real.launch.py port:=/dev/ttyUSB0 config:=/duong/dan/esp32.yaml
```

Mở terminal khác, source workspace như trên rồi xem trạng thái:

```bash
ros2 topic echo /esp32/status
```

Launch mặc định `auto_arm:=true`: tự chuyển từ DISARMED sang ARMING rồi ARMED,
không cần gọi service. Chờ `ARMED; host=OK; mcu=ACTIVE; fault=0`, rồi chạy:

```bash
ros2 run forklift_control arrow_teleop.py
```

Auto arm chỉ chạy một lần lúc khởi động, gửi vận tốc bánh bằng 0 và chờ telemetry
ACTIVE xác nhận. Lệnh di chuyển trong lúc arm không được phát lại; bridge cần
một lệnh zero mới sau khi ARMED trước khi nhận chuyển động. Command gate trong
launch phát zero khi chưa có đầu vào điều khiển.

Disarm chủ động, lỗi hoặc E-stop hủy auto arm đang chờ. Auto arm không tự bật
lại sau disarm hoặc lỗi. Sau khi xử lý nguyên nhân, khởi động lại launch để
dùng auto arm ở phiên mới, hoặc arm thủ công bằng service hiện có.

Muốn giữ quy trình arm thủ công, chạy Pi với:

```bash
ros2 launch forklift_bringup real.launch.py auto_arm:=false
```

Sau khi có telemetry DISARMED hợp lệ, có thể gọi
`ros2 service call /esp32/arm std_srvs/srv/SetBool '{data: true}'`.
`success: true` chỉ xác nhận đã gửi yêu cầu; vẫn chờ ARMED trước khi điều khiển.

Chọn bộ gõ tiếng Anh, focus terminal bàn phím và chờ `Manual mode ready`.

| Phím | Chức năng |
|---|---|
| ↑ / ↓ | Tiến / lùi |
| ← / → | Quay trái / phải |
| A / D, hoặc Shift + ← / → | Đi ngang trái / phải |
| H / Home, F / Page Down | Hạ tối đa 3 s hoặc tới công tắc dưới |
| R / Page Up | Nâng thêm 2.000 xung, xe phải dừng; không cần home |
| Space / S | Gửi zero bánh và D dừng/hủy nâng |
| Q / Ctrl+C | Dừng và thoát bàn phím |

Phím chữ nhận cả hoa/thường. Nhả phím hết timeout bàn phím vẫn là lệnh dừng chủ
động như SIM; command gate giữ giới hạn gia tốc và khóa lái khi nâng/homing
hoặc nâng quá chiều cao vận chuyển. Chưa home không còn khóa lái lúc khởi động.
Một goal nâng chạy đến đích, không cần giữ R/F; Space hủy giữa chừng.

## Dừng, lỗi và nối lại

Disarm chủ động:

```bash
ros2 service call /esp32/arm std_srvs/srv/SetBool '{data: false}'
```

Bridge gửi D, trạng thái sau đó cho biết mode MCU và số xung thực tế. Khi cần reset, xử lý
nguyên nhân, đợi bánh dừng và telemetry dừng sau D rồi gọi:

```bash
ros2 service call /esp32/reset std_srvs/srv/Trigger '{}'
```

`/lift/reset` là alias cùng chức năng reset MCU, xóa homed; không bắt buộc home để nâng/lái.
Phản hồi service chỉ xác nhận **đã gửi R**. Chờ telemetry fault=0, DISARMED và
host=OK trước khi arm lại. Reset bị firmware từ chối sẽ không được coi là hoàn tất.

Theo lựa chọn đã có của dự án, **fault/mất liên lạc không tự ngắt PWM bánh**.
Firmware không còn watchdog lệnh Serial: khi không có lệnh mới, PID tiếp tục
điều khiển theo mục tiêu cuối. Timeout phía ROS bridge vẫn giữ nguyên.
Khi telemetry/lệnh ROS mất quá hạn hoặc MCU báo lỗi/khởi động lại, bridge khóa phiên điều
khiển, ngừng tự gửi frame; không tự gửi D/C-disabled để thay đổi chính sách giữ
PWM của firmware. PWM thực tế có thể vẫn còn cho đến khi ngắt nguồn hoặc lệnh D
chủ động được nhận. `DISARMED` phía host khi lỗi **không xác nhận bánh đã dừng**.
Cơ cấu nâng vẫn dừng khi lỗi phần cứng, quá hạn vòng điều khiển hoặc quá hạn
homing; mất lệnh Serial không còn làm dừng nâng.

Dừng bằng Space, service disarm, E-stop ROS do người vận hành kích hoạt, hoặc
Ctrl+C launch là lệnh dừng chủ động, vẫn gửi D. Phản hồi GPIO E-stop của MCU được
báo lỗi theo chính sách firmware; E-stop ROS không thay cho mạch ngắt nguồn.

Nối lại telemetry không tự tiếp tục lệnh cũ. Cần dừng chủ động, reset nếu MCU có
fault rồi arm lại. Nếu USB bị rút, lỗi đọc/ghi hoặc ghi dở frame: khởi động lại
launch sau khi nối lại. Không tự reconnect hoặc mở khóa firmware.

`/esp32/status` và `/diagnostics` hiển thị trạng thái, số xung, bộ đếm RX và lỗi và lỗi
host như `TELEMETRY_TIMEOUT`, `ROS_COMMAND_TIMEOUT`, `MCU_REBOOT` (ACK_TIMEOUT chỉ ở v2 cũ).
Lỗi kết nối không làm bridge phát lại odometry cũ thành dữ liệu mới.

## Khi báo NO_TELEMETRY trên Linux

Hai bên mặc định **115200 baud**. `invalid_frames=0` chỉ nói chưa có dòng lỗi được
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

- `port`, `baud`: xác nhận đúng thiết bị và 115200.
- `modem_lines=released`: driver đã chấp nhận lệnh nhả DTR/RTS.
  `unsupported` dùng cho PTY hoặc thiết bị không hỗ trợ ioctl modem.
- `rx_bytes=0`: chưa nhận byte; kiểm tra đúng cổng, board boot/EN, cáp USB và
  Serial Monitor/phần mềm khác đang chiếm hoặc đọc cùng cổng.
- `rx_bytes` tăng, `rx_lines=0`: có byte nhưng thiếu newline hoàn chỉnh.
- `rx_lines` và `invalid_frames` tăng: có dòng, cần kiểm tra nội dung/giao thức/CRC.
- `discarded_frames` tăng: nhận dòng quá dài, bị bỏ đến newline tiếp theo.

Sau 5 giây chưa có telemetry hợp lệ, terminal launch sẽ báo các bộ đếm này.
Nếu `rx_bytes` vẫn bằng 0, nhấn EN/RESET một lần (không giữ BOOT), xem bộ đếm
có tăng không. Encoder A FR hiện dùng GPIO2, không dùng GPIO12. GPIO2 cần LOW
hoặc thả nổi khi vào chế độ nạp UART; chỉ thay đổi dây khi đã tắt nguồn.
Riêng thay đổi RTS/DTR không cần nạp lại; đổi chân encoder cần nạp lại firmware
và chuyển dây tương ứng.
