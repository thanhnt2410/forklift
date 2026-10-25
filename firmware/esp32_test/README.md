# Test riêng 4 động cơ ESP32

Mở `esp32_test.ino` trong Arduino IDE, chọn **ESP32 Dev Module (ESP32-WROOM)**,
core **Arduino ESP32 3.x**, nạp sketch. Không cần thư viện ngoài. Firmware này
độc lập với `firmware/esp32`, không dùng ROS và không điều khiển nâng.

Serial Monitor: **115200 baud**, **Newline** hoặc **Both NL & CR**.
Sau khởi động sẽ thấy `READY motors stopped`, hướng dẫn lệnh và trạng thái mỗi 250 ms.

## Lệnh test

| Lệnh | Tác dụng |
|---|---|
| `FL,25` | Chỉ bánh trước trái chạy PWM +25%, ba bánh còn lại dừng |
| `FR,25` | Chỉ bánh trước phải chạy PWM +25% |
| `RL,25` | Chỉ bánh sau trái chạy PWM +25% |
| `RR,25` | Chỉ bánh sau phải chạy PWM +25% |
| `RL,-25` | Chỉ bánh sau trái chạy chiều ngược, PWM 25% |
| `ALL,25` | Cả bốn bánh cùng PWM +25% |
| `C,25,0,-25,0` | Đặt PWM riêng cho FL, FR, RL, RR |
| `D` | Dừng tất cả PWM ngay khi nhận đủ dòng |
| `Z` | Đưa số đếm encoder về 0, giữ nguyên PWM |
| `?` | In hướng dẫn |

PWM là **số nguyên từ -100 đến 100 phần trăm**, không phải rad/s.
Mỗi lệnh chỉ cần gửi một lần. Lệnh từng bánh thay thế toàn bộ lệnh cũ, ba bánh
còn lại được đặt về 0. Lệnh hợp lệ có phản hồi `OK PWM ...`; lệnh sai có `ERR`
và giữ nguyên PWM trước đó. Không có PID, arm, timeout Serial hay khóa lỗi encoder.
Ngừng gửi lệnh hoặc rút USB không tự dừng PWM; gửi `D` hoặc ngắt nguồn motor.

## Đấu chân

| Bánh | RPWM | LPWM | Encoder A | Encoder B |
|---|---:|---:|---:|---:|
| FL — trước trái | 25 | 26 | 34 | 35 |
| FR — trước phải | 18 | 19 | 2 | 14 |
| RL — sau trái | 23 | 22 | 5 | 17 |
| RR — sau phải | 21 | 27 | 16 | 4 |

Đã cập nhật theo các lần thử thực tế: hai cặp PWM FL/RL hoán đổi so với bảng
ban đầu; encoder A FR đã chuyển từ GPIO12 sang GPIO2, B vẫn GPIO14.
`MOTOR_SIGN = {1,-1,1,-1}` để lệnh dương
quay tiến, `ENCODER_SIGN = {1,1,1,-1}` để encoder RR tăng khi tiến.
Encoder FR chỉ dao động 0–1 khi bánh quay: dấu FR hiện tạm giữ +1, cần kiểm tra
tín hiệu A/B ở GPIO2/14 và thử lại trước khi kết luận dấu encoder.
Cấu hình chân và dấu này đã được đồng bộ sang firmware chính trong `firmware/esp32`.

Theo cấu hình hiện tại, R_EN/L_EN mỗi BTS7960 kéo HIGH ngoài; nối chung GND
giữa ESP32, driver và encoder. GPIO13 (STEP), GPIO33 (DIR) được giữ LOW.
GPIO34/35 không có điện trở kéo nội; cần kéo ngoài nếu ngõ ra encoder yêu cầu.
GPIO2/5 là chân strapping. GPIO2 cần LOW hoặc thả nổi khi vào chế độ nạp UART;
nếu encoder giữ HIGH và không nạp được, tắt nguồn rồi tháo dây GPIO2 trong lúc nạp.
Tín hiệu encoder vào ESP32 cần tương thích 3,3 V.

## Đọc và chỉnh encoder

Ví dụ một dòng (giá trị minh họa):

```text
T=1500 ms | FL pwm=25% enc=450 rpm=96.0 | FR pwm=0% enc=0 rpm=0.0 | RL pwm=0% enc=0 rpm=0.0 | RR pwm=0% enc=0 rpm=0.0
```

- `pwm`: lệnh PWM có dấu; `enc`: số đếm x4 tích lũy, có dấu.
- `rpm`: tốc độ trục ra hộp số/bánh trong khoảng lấy mẫu vừa qua, có dấu.
- CPR danh nghĩa **11 × 21,3 × 4 = 937,2** count/vòng trục ra, chưa hiệu chuẩn.
- Encoder được đọc bằng ngắt CHANGE ở cả A và B. Quy ước giải mã thô giống firmware chính:
  chuỗi AB `00 → 01 → 11 → 10 → 00` đếm +4 trước khi nhân `ENCODER_SIGN`.
  Số đếm âm không làm dừng motor.

Kê bánh khỏi mặt đất, bắt đầu PWM 15–25%, thử từng bánh rồi gửi `D` trước khi
đổi chiều. Nếu PWM thấp mà bánh chưa quay, tăng dần theo quan sát; PWM mở vòng
không bảo đảm một tốc độ cố định.

Nếu lệnh dương quay đúng chiều mong muốn nhưng encoder âm, sửa phần tử tương ứng
của `ENCODER_SIGN` trong `motor_config.h` từ `1` sang `-1`. Nếu chiều quay thực tế
sai, sửa `MOTOR_SIGN`, rồi kiểm tra lại chiều encoder. Ví dụ đảo thêm dấu encoder RL
và giữ dấu RR hiện tại:

```cpp
const int ENCODER_SIGN[MOTOR_COUNT] = {1, 1, -1, -1};
```

Để kiểm tra CPR: gửi `D`, `Z`, quay tay bánh đúng một vòng rồi xem thay đổi `enc`.
Không có lệnh chạy tự động khi bật nguồn. Đếm bằng ngắt phần mềm phục vụ test;
ở tốc độ cao hoặc khi có nhiễu, cần đối chiếu số đếm với phép đo thực.

## Biên dịch bằng CLI

Từ thư mục gốc repo:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
  --build-path /tmp/forklift-esp32-test-build firmware/esp32_test
```

Biên dịch chỉ kiểm tra phần mềm, chưa xác nhận chiều quay/encoder trên motor thật.
