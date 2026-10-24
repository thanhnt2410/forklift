# P6 — Lấy và giao pallet tại trạm cố định trong Gazebo

> Cấu hình điều hướng mới nhất: [path đã duyệt và FollowPath](curved_navigation.md).
> [Nghiệm thu mới 29/29](../results/approved_route_sim/verified_demo_v2/summary.json),
> [video](../results/approved_route_sim/verified_demo_v2/mission_demo.mp4).
> Footprint cập nhật theo trạng thái tải; Home -75°, rút thoát trạm 0,35 m rồi lùi theo cung.
> Các số liệu cũ bên dưới ghi lại các lần nghiệm thu trước.


> Cập nhật CAD URDF20: mô hình mặc định đã đổi sang kích thước CAD.
> Xem [tích hợp URDF20](forklift_model.md). Kết quả kiểm thử cũ bên dưới thuộc mô hình trước khi đổi CAD.

Phạm vi: ROS 2 Humble / Gazebo Fortress, không firmware/cổng serial thật. Thiết kế xuất phát từ [kế hoạch lấy hàng](p6_pickup_plan.md). Các số đo/tải trong trang này thuộc mô hình SIM, không phải định mức phần cứng.

## Cách lấy hàng

`Nav2 → ALIGN theo mặt kiện → INSERT thẳng → LIFT_PICK → VERIFY_LOAD → RETREAT_PICK → Nav2 tới B → ALIGN/INSERT tại B → LOWER_RELEASE → RETREAT_DROP → RETURN_HOME`.

- Pallet là **model động riêng** tên `p6_pallet`, 0,1 kg gồm pallet và kiện; không có joint gắn với xe, không attach/detach, không teleport trong controller. Trọng lực và collision thực thi bởi Gazebo.
- Pallet sâu 140 mm, rộng 260 mm. Ba thanh chân tạo hai khe thông rộng 100 mm mỗi khe; càng cách tâm 140 mm và rộng 25 mm đi qua các khe. Deck phía dưới cao 50 mm, dày 12 mm. Kiện rộng 240 mm, sâu 60 mm, mặt trên cao 300 mm, cố định trên pallet. Tâm khối kiện nằm phía trước 80 mm tính từ miệng pallet.
- LiDAR P6 đặt `x=0.18, z=0.235 m` để nằm trước mast; các launch P0–P5 giữ vị trí cũ. Vì thay extrinsic cảm biến, các lần nghiệm thu P6 dùng cấu hình này và được thử lại với AMCL/map đã lưu.
- Detector chỉ nhận `/scan_raw` thực: gom cụm, fit mặt phẳng 2D theo các điểm cùng mặt, loại điểm ở cạnh mốc, kiểm tra bề rộng/sai số và hai biên quan sát. AMCL/TF giúp chọn vùng tìm; không nhận pose Gazebo. Bộ điều khiển dùng dung sai ngang thô 20 mm khi còn xa điểm preinsert hơn 100 mm, sau đó siết về 6 mm trước khe; khi INSERT chỉ tiến/lùi thẳng, sai lệch quá ngưỡng thì dừng.
- Điểm chờ A dùng `(-0.05,0.65,0)` để toàn footprint đứng ngoài vùng inflation của kiện; B dùng `(0.45,-0.60,0)`. Docking tiếp tục tiến sát từ điểm chờ, không đổi costmap để bỏ qua vật cản.
- Điểm đầu càng ở `x≈0.377 m` tính từ gốc xe. Khi đã vào, mặt kiện cách LiDAR khoảng 147 mm, lớn hơn range tối thiểu 120 mm; không cần đoạn tiến mù theo thời gian trong mô hình này. Chiều sâu đưa càng vào khoảng 100 mm.
- Home offset được trừ khi gọi `MoveLift`: mức mang tải là **joint thô 16 mm**, không phải action height 16 mm. Mặt trên càng đạt khoảng 58 mm, nhấc pallet khoảng 8 mm; vẫn thấp hơn gate 20 mm. Không thay ngưỡng gate.
- Cảm biến tiếp xúc trên deck phát hiện collision giữa deck và hai càng. C++ `contact_throttle` giới hạn bản tin từ từng bước vật lý xuống tối đa 50 Hz trước bridge; không phát lại heartbeat nếu không có dữ liệu mới. Mất tiếp xúc trong lúc mang tải làm mission FAULT.
- Tại B chưa có kiện để quan sát; thêm mốc cố định bên hông phía sau vùng đặt tải, biến đổi đã biết: `dx=0.45, dy=1.40 m` so với mặt kiện đích. Mốc rộng 400 mm theo trục x, mặt quan sát hướng ngang theo trục y, cao 300 mm, nằm ngoài góc khuất của kiện đang chở. Đây là docking ở trạm cố định có mốc, không phải nhận dạng vị trí giao bất kỳ.

## Luồng lệnh và trạng thái

Package mới `forklift_task_manager` cung cấp action `/mission/run` kiểu `forklift_interfaces/action/RunMission`. Hiện hỗ trợ `mission: a_to_b`, một kiện mỗi lần; chưa có quản lý tồn kho hoặc chiều B→A.

`motion_mux` chọn riêng Nav2 hoặc docking trước velocity smoother → navigation guard → command gate → base controller. Đổi nguồn phát zero và bỏ cache lệnh; lệnh docking quá 200 ms bị dừng. Mission không phát trực tiếp `/cmd_vel`. Supervisor nâng/watchdog/E-stop P5 vẫn hoạt động.

Task manager kiểm tra tuổi scan/odom/trạng thái nâng/navigation readiness, action result, contact tải, timeout, giới hạn quãng đường lùi và vùng phía sau. Fault/cancel giữ vị trí nâng, hủy action đang chạy, giữ nguồn docking zero; không tự hạ tải hoặc tự tiếp tục. Các lỗi được giữ tới phục hồi chủ động.

Footprint P6 bao gồm xe và biên tải: khoảng `x=[-0.1675,0.46], y=[-0.20,0.20] m`. Khi mang tải, `scan_relay` chỉ bỏ điểm tự phản xạ trong hộp tải đã biết bằng NaN, không dùng infinity để xóa vật cản bị che, không tắt obstacle layer. Detector vẫn đọc scan thô. Hàng gây vùng khuất thật phía trước LiDAR: mô hình này chưa cung cấp cảm biến dự phòng để bảo đảm phát hiện mọi vật cản mới nằm trong vùng khuất đó.

## Chạy thử

Launch P6 khởi động server và controller trước, rồi mở GUI client riêng; tránh timeout tạo entity khi Fortress khởi động server/GUI đồng thời trên máy hiện tại. Các launch P0–P5 giữ trình tự cũ.

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_bringup p6.launch.py
```

Chờ home xong và `/navigation/ready` là true; terminal khác đã source cùng workspace/domain:

```bash
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission \
  '{mission: a_to_b, timeout: 300.0}' --feedback
```

Để dừng, dùng API cancel action hoặc:

```bash
ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
```

Ctrl+C client `send_goal` không bảo đảm đã cancel goal trên server. Theo dõi `/mission/status` (JSON gồm state/code/load/face/events) và `/lift/state`; trạng thái lifecycle Nav2 active chưa đủ chứng minh được phép chạy.

Sau lỗi: giải quyết nguyên nhân; nếu cần giữ/đặt tải xuống thì xử lý nâng có giám sát và xác minh vị trí tải trước. Phục hồi các tầng P5/navigation theo [hướng dẫn P5](p5_lift.md) và [P3/P4](p3_p4_simulation.md), sau đó `/mission/reset`. Reset bị từ chối nếu còn active, chưa ready hoặc còn tiếp xúc tải. Không tự reset/home khi đang mang tải.

Sau DONE, nguồn giữ ở docking zero. Nếu chủ động muốn dùng lại Nav2 goal ngoài nhiệm vụ, chỉ đổi nguồn khi xe/tải đã ở trạng thái hợp lệ:

```bash
ros2 service call /mission_motion/select_docking std_srvs/srv/SetBool '{data: false}'
```

Pallet đã ở B sau nhiệm vụ, vì vậy không gửi lại A→B với giả định A vẫn có hàng. Runner kiểm thử đưa pallet trở về A **giữa** các lượt để chuẩn bị fixture; hành vi đó không thuộc controller và được ghi trong mã kiểm thử.

## Kiểm thử và dữ liệu

Runner tạo ROS domain và Gazebo partition riêng, rồi dọn đúng các process của phiên đó:

```bash
python3 scripts/run_p6_checks.py --repetitions 10 --output results/p6/nominal
python3 scripts/run_p6_checks.py --scenario cancel --output results/p6/fault_cancel
python3 scripts/run_p6_checks.py --scenario scan_loss --output results/p6/fault_scan
```

Chạy đầy đủ các ca lỗi theo thứ tự, mỗi ca dùng domain riêng; không mở thêm simulator trong lúc nghiệm thu lặp:

```bash
python3 scripts/run_p6_fault_matrix.py
python3 scripts/record_p6_demo.py
# Plot dùng NumPy/Matplotlib từ apt trên máy Ubuntu hiện tại:
python3 -s scripts/plot_mission.py results/p6/nominal
```

Các scenario khác: `imu_loss`, `estop`, `missing_pallet`, `misaligned`, `blocked`, `lift_fault`, `tf_loss`, `load_loss`, `cancel_loaded`. Thêm `--gui` để xem Gazebo/RViz. Không chạy nhiều phiên cùng ROS domain; chọn `--domain` khác nếu cần.

`checks.json`, `mission.csv`, `summary.json`, `run.json`, snapshot cấu hình + SHA256 trong `config/`, `launch.log` và `test.log` lưu ở thư mục output. Ground truth chỉ đi tới node nghiệm thu `/test/p6_poses` để chấm vị trí hàng, chiều cao thực, độ lệch docking và va chạm footprint; task manager không subscribe kênh này. Thành công cần action DONE, pallet nằm tại B trên nền, xe về Home, contact đã nhả và càng không vượt gate khi vận chuyển.

Các lượt phát triển thất bại được giữ tại các thư mục smoke/contact/drop-marker/candidate; không cộng chúng thành số lượt nghiệm thu thành công. Kết quả nghiệm thu cuối được ghi ở `results/p6/summary.json` sau khi hoàn thành bộ kiểm tra.

Giới hạn: vật có kích thước/khối lượng biết trước, đặt đúng trên pallet; vùng bắt được detector có giới hạn; gá bên hông riêng ở B; bản đồ kho cố định; contact/actuator lý tưởng trong simulator. Chưa chứng minh tải thật, độ chính xác LiDAR X4 thật, trượt/giữ tải vít me thật, hàng lệch trên pallet, nhiều loại kiện hoặc an toàn vận chuyển trong vùng LiDAR bị hàng che.

## Kết quả nghiệm thu cuối

| Bộ kiểm tra | Kết quả | Dữ liệu |
|---|---|---|
| Chu trình A→B→Home | 10/10 lượt, 103/103 kiểm tra | [Nominal](../results/p6/nominal/summary.json) |
| Lỗi/cancel/phục hồi | 11/11 tình huống, 103/103 kiểm tra | [Ma trận lỗi](../results/p6/faults/summary.json) |
| Unit và hồi quy P4/P5 | 46/46 | [Log](../results/p6/unit_tests.log) |
| GUI một chu trình | Đạt | [Video](../results/p6/demo/mission_demo.mp4), [kết quả](../results/p6/demo/summary.json) |

Sai lệch đặt pallet lớn nhất **1.31 cm**; khoảng hở tải nhỏ nhất khi mang **7.97 mm**. Đây là phép chấm bằng pose vật lý Gazebo, không phải số đo xe thật. [Biểu đồ lượt đại diện](../results/p6/nominal/mission_results.png).

[Tổng hợp có kiểm tra gate](../results/p6/summary.json) và [SHA256 mã nguồn](../results/p6/source_manifest.json). Các lần debug và ca bị ngắt không tính vào các số liệu trên.
