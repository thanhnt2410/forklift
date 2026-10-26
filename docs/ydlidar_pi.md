# Chạy LiDAR trên Pi, nhận dữ liệu trên laptop

Driver `ydlidar_ros2_driver` nhánh humble và YDLidar-SDK phải được cài trên Pi.
Launch này chỉ chạy LiDAR, không mở cổng ESP32, không arm hoặc chạy motor bánh xe.

## Cấu hình và chạy

Sửa `src/forklift_hardware/config/ydlidar.yaml` trên Pi. Cấu hình đã thử trực tiếp
với thiết bị hiện tại: 115200 baud, giao tiếp một chiều, motor quay độc lập,
Trong lần thử ban đầu với `fixed_resolution=false` đã nhận 10 LaserScan khoảng 6,17 Hz,
khoảng 649 điểm/lượt, frame `laser_frame`; đây không phải profile X4 mặc định
128000 baud. Chưa xác nhận model bằng phản hồi thông tin thiết bị.

Cấu hình hiện tại giữ `fixed_resolution=true`: người dùng đã xác nhận map
cập nhật với giá trị này trên xe thật.

## Tự chọn cổng khi cả hai thiết bị dùng CP2102

LiDAR dùng `port` theo `by-path` trong YAML, tự theo đúng ổ USB dù số
`ttyUSB0/1` thay đổi. `real.launch.py` đọc cùng YAML và truyền `lidar_port`
cho bridge: ESP32 auto bỏ qua cổng LiDAR cùng mọi alias của nó trước khi mở
cổng, kể cả khi driver LiDAR chưa chạy. Các cổng còn lại được nhận diện bằng
hai bản tin telemetry ESP32 có timestamp tăng; không chọn chỉ theo CP2102.
Giữ đúng ổ USB vật lý của LiDAR. Nếu đổi ổ, cập nhật `port` trong YAML.

Chạy hai terminal như thường lệ, không cần chỉ định cổng ESP32:

```bash
ros2 launch forklift_bringup lidar.launch.py
ros2 launch forklift_bringup real.launch.py
```

Nếu LiDAR dùng file cấu hình riêng, truyền cùng file cho cả hai launch:
`lidar.launch.py params_file:=/duong/dan/lidar.yaml` và
`real.launch.py lidar_params_file:=/duong/dan/lidar.yaml`.
Không khởi động lại hai launch phần cứng đồng thời; chờ một launch dò xong
rồi chạy launch còn lại. Cơ chế này không tự dò lại khi rút/cắm USB lúc đang chạy.

Build sau khi thêm file lần đầu:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_ws/forklift
colcon build --symlink-install --packages-select forklift_hardware forklift_bringup
```

Mỗi lần chạy trên Pi:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/forklift/install/local_setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch forklift_bringup lidar.launch.py
```

Giữ terminal chạy. Không chạy thêm driver hoặc SDK test trên cùng cổng Serial.
Cổng mặc định là
`/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0`,
ứng với ổ USB vật lý trên Pi đang nối LiDAR. Giữ LiDAR cắm đúng ổ này.
ESP32 và LiDAR hiện cùng báo CP2102 serial `0001`, nên liên kết `by-id` trùng
nhau và có thể trỏ nhầm sang ESP32. Dùng `by-path` để phân biệt theo ổ USB,
không phụ thuộc thiết bị được gán `ttyUSB0` hay `ttyUSB1` sau khi cắm lại.
Nếu đổi ổ USB, xem `ls -l /dev/serial/by-path/`, đối chiếu thiết bị thực rồi
sửa `port` trong YAML. Không chọn liên kết `by-id` trùng `0001` cho LiDAR.

Có thể dùng cấu hình khác mà không sửa launch:

```bash
ros2 launch forklift_bringup lidar.launch.py params_file:=/duong/dan/lidar.yaml
```

Không sửa file YAML sinh ra trong `install/`; sửa file trong `src/`, build lại
và khởi động lại launch để áp dụng chắc chắn.

## Kiểm tra

Trong terminal khác trên Pi hoặc laptop (cùng mạng/domain):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic hz /scan --window 30
```

Nhấn Ctrl+C để thoát phép đo, sau đó xem frame:

```bash
ros2 topic echo /scan --once --field header
```

SDK còn có thể báo không đọc được thông tin thiết bị và checksum ở đầu phiên.
Kiểm tra scan liên tục và hình quét trong RViz; nếu lỗi checksum tiếp tục xuất
hiện hoặc có khoảng mất dữ liệu, xử lý trước khi mapping.
Launch chưa phát TF `base_link -> laser_frame`: cần cấu hình theo vị trí/góc lắp
thật khi chạy SLAM trên laptop. Không dùng launch mapping Gazebo cho robot thật.

## Mapping robot thật với vị trí LiDAR theo CAD SolidWorks

Khi LiDAR lắp đúng mô hình CAD, dùng `mapping_real.launch.py` trên laptop.
Launch đọc Xacro hiện tại và chạy `robot_state_publisher`, SLAM Toolbox với
`use_sim_time=false`, cùng RViz. Driver LiDAR và ESP32 bridge vẫn chạy riêng;
launch mapping không tự chạy hoặc arm motor.

Trong mô hình đã chuẩn hóa, `base_link` nằm ở mặt đất. Joint từ `base_link`
đến `chassis` cao 0,0704 m; joint CAD từ chassis đến LiDAR cao 0,2242 m.
Frame `laser_frame` vì thế ở `[0, 0, 0.2946]` m, roll/pitch/yaw đều 0,
theo `forklift_description/config/robot.yaml`. Đây là vị trí theo mô hình;
đối chiếu hướng quét thực tế nếu scan không khớp tường khi xe quay.

Trước khi chạy, dừng node `lidar_test_static_tf` hoặc static publisher test
`base_link -> laser_frame` bằng Ctrl+C tại terminal tương ứng. Không chạy thêm
`robot_state_publisher` thứ hai. ESP32 bridge phải phát `/odom`, `/joint_states`
và TF `odom -> base_link`; LiDAR phải phát `/scan` với frame `laser_frame`.

Trên laptop, tại thư mục workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select forklift_description forklift_bringup
source install/local_setup.bash
ros2 launch forklift_bringup mapping_real.launch.py
```

Không dùng `mapping.launch.py` cho bước này vì launch đó khởi động Gazebo.
Nếu không cần GUI, thêm `rviz:=false`. Giữ mạng và ROS_DOMAIN_ID như lúc test.

Kiểm tra trong terminal khác:

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
ros2 topic echo /map --once --field info
```

Sau khi bản đồ xuất hiện, điều khiển xe chậm bằng luồng manual đã test, đi qua
khu vực cần lập bản đồ rồi quay về vùng đã quét. Trong RViz, tường nên đứng yên
và các lượt quét chồng khớp. Nếu tường bị đôi hoặc xoay theo xe, kiểm tra TF,
hướng quét, timestamp và odometry trước khi tiếp tục.

Lưu bản đồ khi SLAM vẫn chạy:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/forklift_real
```

Lệnh tạo `forklift_real.yaml` và ảnh bản đồ trong `~/maps`; dùng tên khác để
giữ lại bản đồ cũ. Bản đồ occupancy này phục vụ định vị; muốn tiếp tục phiên
SLAM sau khi tắt cần lưu thêm pose graph qua dịch vụ serialize của SLAM Toolbox.
