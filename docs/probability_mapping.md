# Vẽ bản đồ xác suất với SLAM Toolbox

`probability_slam` kế thừa async SLAM Toolbox 2.6.10 (ROS 2 Humble) đang cài,
giữ scan matching, solver, loop closure, dịch vụ lưu/nạp pose graph và `/map`.
Không chạy thêm một SLAM Toolbox khác cùng lúc.

Đầu ra bổ sung:

- `/map_probability`: OccupancyGrid `-1` nếu chưa quan sát; `0–100` là
  ước lượng xác suất có vật cản từ thống kê tia của Karto.
- `/mapping/trajectory`: các pose robot ứng với scan SLAM đã chấp nhận,
  được cập nhật sau tối ưu; không phải đường dự kiến của Nav2 hay odom thô.
- `/mapping/scan`: scan thật đã lọc, giữ timestamp và frame gốc.
- `/mapping/scan_status`: số scan nhận/xuất/từ chối, tuổi scan, tỷ lệ tia hợp lệ.

Với `h` tia chạm ô và `n` lượt tia đi qua hoặc chạm ô, dùng prior Beta(1,1):
`p = (h + 1) / (n + 2)`. Ví dụ chỉ thấy trống: `33 → 25 → 20 → … → 0`;
chỉ thấy vật cản: `67 → 75 → 80 → … → 100`.
Đây là xác suất có làm trơn trên tỷ lệ tia, không phải mô hình log-odds với
FREE_PROB/OCC_PROB của bumperbot. Các tia có tương quan, nên không xem nó là
xác suất đã hiệu chuẩn của cảm biến. Một lượt quét có thể góp nhiều tia vào ô;
không phải mỗi lần quét chỉ cập nhật ô một lần.

Karto tăng bộ đếm pass thêm một lần ở endpoint sau khi đã trace cả đường tia;
extension trừ số hit khỏi bộ đếm pass thô để mỗi lượt tia chỉ tính một lần.

Bản đồ được dựng lại từ toàn bộ scan và pose đã tối ưu mỗi 2 giây, kể cả sau
loop closure, không tích lũy vào vị trí cũ. Việc này tốn thêm CPU và giữ khóa
mapper trong lúc dựng; tăng `probability_update_interval` nếu khu vực lớn
làm xử lý scan chậm. `/map` vẫn dùng phân loại upstream cho AMCL/Nav2.

## Build và chạy

```bash
cd ~/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --packages-select forklift_navigation forklift_bringup --symlink-install
source install/local_setup.bash
ros2 launch forklift_bringup mapping_real.launch.py
```

Phần cứng phải đang phát `/scan`, `/odom`, `/joint_states` theo hướng dẫn
[LiDAR trên Pi](ydlidar_pi.md). Launch mapping cung cấp TF từ URDF.
RViz hiển thị `/map_probability` và đường SLAM màu cam.
Mô phỏng: `ros2 launch forklift_bringup mapping.launch.py` cũng dùng extension,
nhưng giữ relay scan riêng của mô phỏng; `/mapping/scan` và scan_status chỉ có
trong launch thật.

Bộ lọc thật mặc định nhận tia hữu hạn trong khoảng giao giữa giới hạn cảm biến
và 0.12–10 m. Tia NaN/Inf/ngoài giới hạn trở thành NaN để không bịa vùng trống.
Scan thiếu header, quá cũ (>0.5 s), ở tương lai (>0.1 s), sai hình học hoặc không
có tia dùng được sẽ bị loại. Đồng bộ giờ Pi/laptop bằng NTP/chrony; kiểm tra
`timedatectl status` trên cả hai. Có thể đặt `max_scan_age:=1.0` nếu đã đo độ trễ
thực tế; không tăng ngưỡng để che lỗi đồng hồ. Khởi động lại mapping sau khi
đồng hồ bị reset. Không tự thay timestamp để làm scan có vẻ mới.

## Ghi và kiểm chứng một vòng quét thật

Sau khi source ROS/workspace trong các terminal, chạy trước khi bắt đầu lái:

```bash
ros2 bag record -o /tmp/forklift_mapping_trial /scan /mapping/scan /mapping/scan_status /odom /tf /tf_static /joint_states /imu/data /map /map_probability /mapping/trajectory
```

Nếu IMU dùng topic khác, thay `/imu/data` bằng topic đang có. Chạy công cụ chỉ
đọc ở terminal khác trong suốt bài thử (thời gian tính bằng giây):

```bash
python3 scripts/check_mapping.py --duration 120 --output /tmp/mapping_report.json
ros2 topic echo /mapping/scan_status --once
```

Thử xe đứng yên trước tường, đi thẳng 1 m, đi ngang 1 m, quay chậm và đi một
vòng kín quay về vị trí đã đánh dấu. Ghi khoảng cách thực tế để so với bản đồ.
Chỉ điều khiển xe khi phần cứng đã sẵn sàng theo hướng dẫn hiện tại.

Đánh giá report: có scan/odom/map/trajectory; TF tra được tại timestamp scan;
không có timestamp đảo thứ tự; tuổi scan ổn định và thấp hơn ngưỡng lọc;
bản đồ có ô trung gian và ô chưa biết. `start_end_distance_m` chỉ là khoảng
cách hai pose đầu/cuối, có ý nghĩa khi thực sự quay về điểm đầu, không phải
phép đo sai số tuyệt đối. Kiểm tra tường không đôi, không cong; đo kích thước
và độ lệch ở điểm xuất phát. Công cụ không tự kết luận chất lượng hình học.

Để phát lại dữ liệu đầu vào: dừng các launch mapping/driver cũ. Terminal 1:

```bash
ros2 launch forklift_bringup mapping_real.launch.py use_sim_time:=true publish_robot_description:=false relay_replay_tf:=true
```

Terminal 2 (thay đường dẫn bag tương ứng):

```bash
ros2 bag play /tmp/forklift_mapping_trial --clock --topics /scan /odom /tf /tf_static /joint_states /imu/data --remap /tf:=/mapping/replay_tf
```

TF từ bag cung cấp cả extrinsics nên tắt robot_state_publisher khi replay.
`relay_replay_tf` bỏ map→odom cũ rồi phát lại các TF còn lại vào `/tf` để SLAM
mới là nguồn duy nhất của map→odom. Không phát lại `/map`, `/map_probability`,
trajectory hoặc scan đã lọc vào bộ mapping đang tái dựng.

## Lưu bản đồ

Saver được quản lý bởi `lifecycle_manager_mapping` và tự active khi launch.
Chỉ nhận Save Map khi `map_saver_server` ở trạng thái active. Có thể chạy
`autostart:=false` để điều khiển lifecycle thủ công; xem [quản lý lifecycle](lifecycle.md).

Một lần **Save Map** trong panel **SLAM Toolbox** của RViz sẽ lưu cả hai bản
đồ từ cùng một snapshot pose đã tối ưu. Panel được mở sẵn cho mapping thật
và mô phỏng. Nhập prefix, ví dụ `/home/thanh/maps/forklift`, rồi bấm Save Map.
Thư mục cha được tạo tự động. Đường dẫn là trên máy chạy SLAM, kể cả khi RViz
chạy ở máy khác. Đổi prefix để giữ các lần quét trước.

Các file được tạo:

- `forklift.yaml` + `forklift.pgm`: bản đồ phân loại phục vụ AMCL/Nav2.
- `forklift_probability.yaml` + `forklift_probability.png`: bản đồ scale nhiều mức xám.
- `forklift_probability.grid.json`: giá trị ô `-1/0–100` nguyên gốc, metadata và
  timestamp; lưu theo hàng từ dưới lên như OccupancyGrid, không mất giá trị do
  lượng tử hóa ảnh PNG.

Lệnh tương đương nút Save Map, cũng lưu toàn bộ các file trên:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/thanh/maps/forklift'}}"
```

Chỉ trả `result: 0` khi lưu thành công cả hai bản đồ và JSON. Nếu lỗi, xem
log SLAM; có thể một số file đã được tạo trước khi lỗi xảy ra. Nếu chưa có
scan SLAM, trả `result: 1`; lỗi khác trả `255`.

`map_saver_cli` là công cụ riêng, không gọi service này, nên chỉ lưu topic
được chỉ định. Khi muốn lưu riêng ảnh xác suất:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/forklift_probability --mode scale --fmt png --free 0.01 --occ 0.99 --ros-args -r map:=/map_probability
```

Ảnh scale lượng tử hóa và cắt giá trị theo ngưỡng; file `.grid.json` của Save
Map giữ dữ liệu ô gốc. Muốn tiếp tục SLAM sau khi tắt, lưu thêm pose graph
bằng nút Serialize Map hoặc service (Save Map không tự serialize):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/tmp/forklift_posegraph'}"
```

Chưa thể xác nhận chất lượng trên xe thật nếu chưa có bag/bài đo thực tế.
