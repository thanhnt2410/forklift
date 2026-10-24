# Điều khiển mô phỏng bằng tay cầm PS4

Launch PS4 là bộ nhập lệnh cho phiên SIM đang chạy, không mở Gazebo và không
kết nối ESP32. Nó chuyển command gate sang thủ công và phát `/cmd_vel_teleop`.
Không chạy cùng bàn phím hoặc nhiệm vụ tự động đang hoạt động.

## Kết nối và chạy

Cài driver và build các package đã thay đổi:

```bash
sudo apt install ros-humble-joy
cd ~/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select forklift_control forklift_bringup
source install/setup.bash
```

Kết nối PS4 bằng cáp USB có truyền dữ liệu, hoặc ghép đôi qua Bluetooth trong
cài đặt Ubuntu trước. Kiểm tra tay cầm mà SDL nhận được:

```bash
ros2 run joy joy_enumerate_devices
```

Terminal 1, đã source ROS/workspace:

```bash
ros2 launch forklift_bringup sim.launch.py auto_home:=true
```

Terminal 2, đã source ROS/workspace:

```bash
ros2 launch forklift_bringup ps4.launch.py
```

Nếu có nhiều thiết bị, chọn ID SDL trong kết quả enumerate (không suy từ `js0`):

```bash
ros2 launch forklift_bringup ps4.launch.py device_id:=1
```

Cũng có thể dùng `device_name:="tên chính xác từ enumerate"`; tên được ưu tiên
hơn ID. Nếu không có dữ liệu, xem log driver và quyền truy cập `/dev/input/event*`.
Không cần thay đổi quyền toàn bộ `/dev/input`.

## Các nút mặc định

Trước khi chạy, **thả L1 và đưa hai cần về giữa**, sau đó giữ L1 lúc cần vẫn ở
giữa rồi mới đẩy cần. Cần làm lại trình tự này sau timeout hoặc mất kết nối.

| Thao tác | Chức năng |
|---|---|
| Giữ L1 + cần trái lên/xuống | Tiến/lùi, tối đa 0,15 m/s |
| Giữ L1 + cần trái trái/phải | Đi ngang, tối đa 0,12 m/s |
| Giữ L1 + cần phải trái/phải | Xoay, tối đa 0,4 rad/s |
| Giữ L1 + D-pad lên | Nâng thêm 0,01 m mỗi lần nhấn |
| Giữ L1 + D-pad xuống | Hạ 0,01 m mỗi lần nhấn |
| Giữ L1 + Triangle | Homing càng |
| Circle | Gửi zero cho bánh, yêu cầu dừng nâng và khóa lại L1 |
| Nhả L1 | Gửi zero cho bánh; dừng yêu cầu nâng đang do PS4 quản lý |

Nâng/home chỉ được gửi khi cần đã về giữa, odometry cho thấy xe dừng và trạng
thái nâng còn mới. Homing có thể dùng khi chưa home; nâng/hạ theo bước cần đã
home. Giữ nút D-pad không lặp lệnh. Cần tiếp tục giữ L1 đến khi thao tác nâng/home
hoàn tất; nhả L1 sẽ yêu cầu hủy. Trong lúc chờ phản hồi action, bánh luôn nhận zero.
Circle không yêu cầu giữ L1 và không phải E-stop vật lý.

Nấc nâng bị giới hạn trong 0–0,115 m theo supervisor SIM hiện tại. Các khóa của
command gate vẫn áp dụng, gồm homing, càng đang di chuyển và độ cao vận chuyển.
Vì thế khi nâng cao, cần hạ càng về vùng vận chuyển trước khi lái.

## Ánh xạ và mất kết nối

Driver là `joy/game_controller_node` với ánh xạ SDL chuẩn, không phải `joy_node`
đọc thứ tự trục tùy thiết bị. Theo [tài liệu ROS joy](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md),
cần trái dùng trục 0/1, cần phải ngang trục 2; L1=9, Triangle=3, Circle=1,
D-pad lên/xuống=11/12. Bản sao PS4 cần được SDL nhận dạng đúng; phải xác minh trên
tay cầm thực tế trước khi dùng các nút nâng.

Cấu hình ở [`ps4.yaml`](../src/forklift_control/config/ps4.yaml). Driver phát lại
Joy ở 20 Hz, deadzone 10%; bộ teleop hết hạn dữ liệu sau 0,25 s bằng wall clock.
Nhả L1, dữ liệu không hợp lệ hoặc hết hạn đều gửi zero qua command gate; quá trình
dừng còn chịu giới hạn gia tốc của gate. Timeout cũng yêu cầu dừng action nâng
đang do node này quản lý, kể cả goal được chấp nhận sau yêu cầu hủy.

Xem dữ liệu khi kiểm tra ánh xạ:

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel_teleop
```

Khi quay lại Nav2/nhiệm vụ, dừng launch PS4 bằng Ctrl+C rồi khởi động lại bringup
navigation/P6 để chọn nguồn tự động. Không tự chuyển sang Nav2 khi nhả L1.

Phần tích hợp được kiểm tra bằng Joy và action/service ROS giả. Chưa có tay cầm
PS4 thật kết nối trong phiên phát triển; chưa xác nhận USB/Bluetooth hoặc cảm giác
lái trên thiết bị thật. Không thay đổi cấu hình giữ PWM khi lỗi của firmware ESP32.
