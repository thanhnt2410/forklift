# Tích hợp CAD URDF20 vào ROS 2 / Gazebo Fortress

> Điều hướng đã chuyển sang path cong nối đoạn thẳng cuối theo hướng pallet;
> xem [luồng hiện tại](curved_navigation.md). Các kết quả gộp CAD bên dưới được giữ làm lịch sử.

Mô hình mặc định của `forklift_bringup` hiện dùng CAD URDF20. Mesh được giữ tỉ lệ 1:1,
không sửa kích thước để ép về thông số bánh ngang 260 mm / dọc 195 mm trước đây.
Thông số cũ vẫn được lưu trong `config/robot_measurements.yaml` để đối chiếu về sau.

## Cấu trúc thống nhất

Mô hình CAD đã được gộp vào package `forklift_description`; không còn package
`urdf20` riêng hay thư mục `src/URDF20`. Kích thước CAD giữ nguyên.

- `src/forklift_description/urdf/forklift.urdf.xacro`: entry chính, cảm biến và điều khiển.
- `src/forklift_description/urdf/forklift_cad.urdf.xacro`: hình học, inertia CAD và joint.
- `src/forklift_description/urdf/forklift.urdf`: URDF thuần đã expand để đối chiếu;
  dùng Xacro khi chạy để resolve đường dẫn hiện tại.
- `src/forklift_description/meshes/*.stl`: mesh hoạt động, tên đồng nhất với link.
- `src/forklift_description/config/robot.yaml`: kích thước, giới hạn và footprint.
- `src/forklift_description/config/joint_names.yaml`: các joint điều khiển.
- `src/forklift_description/cad_source/forklift.original.urdf`, `forklift.csv`,
  `meshes/*.STL`: nguồn CAD gốc, giữ nguyên nội dung.
- `cad_source/legacy_sim`: các macro hình học đơn giản cũ, chỉ để tham khảo.

Các tên link chính: `base_link`, `chassis`, `lift_carriage`, `lidar_housing`,
`lift_transmission`, `lift_drive_gear`, `lift_screw`, và bốn `*_wheel`.
Các frame giao diện `laser_frame`, `imu_link`, các joint bánh và `lift_joint`
giữ nguyên để tương thích controller, EKF và P6.

Bản sao toàn bộ package trước khi gộp được lưu tại
`results/model_layout/urdf20_before_merge.tar.gz`; bản CAD gốc không phải package ROS.

`base_link` mới ở mặt đất; toàn bộ assembly CAD được tịnh tiến +70,4 mm theo Z
qua `cad_base`. Không đổi vị trí tương đối giữa các chi tiết ở q nâng = 0.

Joint bánh được đưa dọc đúng trục quay tới giữa bề rộng mesh, và bù ngược origin
visual/COM: ngoại hình CAD không thay đổi. Bốn trục quay chuẩn hóa +Y cho controller.
Bao hình mesh toàn xe ở q=0 xấp xỉ 586,4 × 367,14 × 294,49 mm (dài × rộng × cao).
Khoảng cách trước–sau 210 mm; ngang giữa hai mặt phẳng giữa bánh 310,29 mm.
Bán kính tiếp xúc danh định 50 mm; mesh con lăn có biên khoảng 98–98,5 mm do
trạng thái góc quay trong bản xuất CAD, không được co giãn lên 100 mm.

Bản xuất CAD chứa cả mesh càng và LiDAR lặp lại trong `base_link`.
Importer loại các tam giác trùng với link con (sai số tối đa 20 micromet), tạo
`meshes/chassis.stl`; không thay mesh gốc. Điều này tránh càng cố định trùng với càng
đang nâng. Chi tiết trong `results/model_layout/cad_import.json`.

## Vật lý, nâng và cảm biến

Collision khung và càng được đơn giản hóa, tines giữ kích thước CAD
200 × 15 × 2 mm, khoảng cách tâm 145 mm. Collision bánh dùng cầu và ma sát dị hướng
đã dùng trong Fortress, không phải mô phỏng từng con lăn. Footprint bảo thủ có tính
đến mô hình tiếp xúc này. Khối lượng/inertia khai báo CAD được giữ lại; không coi
đó là số đo thực (assembly CAD có chi tiết lặp và có thể thiếu linh kiện điện).

Joint `planar` của càng được đổi thành trượt đứng `lift_joint`.
Bánh răng/vít me được giữ cố định về hình ảnh: chưa biết bước vít để lập quan hệ
truyền động; bộ nâng vẫn là servo vị trí lý tưởng. CAD không cung cấp hard-stop.
Hành trình vận hành tạm thời 120 mm, supervisor nhận tối đa 115 mm, có margin số
5 mm. Đây là giới hạn điều khiển suy ra bảo thủ từ bố trí vít/mast, cần kiểm chứng;
không phải kích thước hành trình đã đo và không sửa chiều dài chi tiết CAD.

LiDAR mô phỏng dùng 720 mẫu/vòng để ổn định việc quan sát mốc trạm B.
LiDAR giữ vị trí CAD: `laser_frame = [0, 0, 0.2946]` so với gốc mặt đất.
MPU6050 không có trong CAD nên vị trí IMU vẫn tạm `[0, 0, 0.12]`.
EKF, joint names và action điều khiển giữ giao diện ROS hiện tại.

## Điều chỉnh fixture P6

Càng CAD khi q=0 có mặt trên cao khoảng 96,13 mm. Pallet mẫu được đổi sang chân
104 mm, đáy deck ở 104 mm; kiện cao tới 354 mm để LiDAR CAD nhìn thấy.
Càng nâng q=16 mm tạo khoảng hở lý thuyết 8,13 mm, dưới gate vận chuyển 20 mm.
Đã cập nhật khoảng cách docking theo đầu càng x=431,3 mm, footprint và vùng tự che
của kiện trong scan relay. Pallet vẫn độc lập, dùng va chạm và trọng lực.

Các kết quả `results/p0`–`results/p6` trước đây thuộc mô hình đơn giản cũ.
Kết quả hồi quy CAD mới đặt riêng trong `results/urdf20`; không cộng dồn hay
suy diễn bộ 10 chu trình / 11 fault cũ sang bản CAD mới.

## Build và chạy

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_bringup sim.launch.py
# hoặc lấy–giao pallet
ros2 launch forklift_bringup p6.launch.py
# terminal khác, đã source:
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission '{mission: a_to_b}' --feedback
```

Tái sinh phần chuyển đổi CAD (cần numpy, scipy; không sửa STL gốc):

```bash
python3 -s scripts/cad/import_forklift_cad.py
xacro src/forklift_description/urdf/forklift.urdf.xacro > src/forklift_description/urdf/forklift.urdf
```

Kiểm thử độc lập:

```bash
python3 -s -m pytest -q tests/test_*.py
python3 scripts/run_p6_checks.py --output results/urdf20/p6 --domain 167
```

## Kết quả kiểm chứng CAD

- Build: 10/10 package; `check_urdf` và chuyển URDF sang SDF thành công.
- Kiểm tra đơn vị/hồi quy: 51/51.
- Chuyển động, odometry, lift endpoint, watchdog và E-stop: 37/37.
- P6 cấu hình cuối: 1 chu trình lấy–giao–về Home, 13/13 kiểm tra.
- Hủy khi mang tải: 10/10, xe dừng, pallet tiếp tục được càng đỡ.
- Một cấu hình CAD trung gian với cùng hình học cũng hoàn thành 1 chu trình;
  không tính chung thành thử nghiệm độ bền của cấu hình cuối.

[Kết quả tổng hợp](../results/urdf20/summary.json),
[chu trình P6](../results/urdf20/p6_final/checks.json),
[chuyển động](../results/urdf20/motion/checks.json),
[hủy có tải](../results/urdf20/cancel_loaded/checks.json),
[hình CAD sau tích hợp](../results/urdf20/cad_preview.png).

Chưa chạy lại toàn bộ ma trận 10 chu trình / 11 tình huống lỗi của mô hình cũ.
Các thư mục có `initial_*_failure` lưu lần thử đầu để đối chiếu, không tính đạt.

## Hiển thị mesh trong Gazebo

Fortress đổi URI `package://forklift_description/...` thành `model://forklift_description/...`.
Launch SIM đã bổ sung thư mục cha của package `forklift_description` vào
`IGN_GAZEBO_RESOURCE_PATH` cho cả server và GUI khởi động riêng.
RViz tự resolve package ROS nên trước khi sửa có thể thấy robot trong RViz
trong khi Gazebo không hiện mesh. Các kiểm tra headless trước đó xác nhận
collision/controller, chưa xác nhận hiển thị CAD. Demo GUI sau sửa nằm tại
`results/urdf20/demo_fixed`, ảnh `gazebo_visible.png` xác nhận robot đã hiện.

Ghi lại một demo GUI riêng, tránh ghi đè bằng chứng cũ:

```bash
python3 scripts/record_p6_demo.py --output results/urdf20/demo_fixed --domain 170
```

Demo GUI sau sửa đường dẫn: hoàn thành 1 chu trình, 13/13 kiểm tra đạt;
[video Gazebo 184 giây](../results/urdf20/demo_fixed/mission_demo.mp4).

## Kiểm chứng sau khi gộp package

Build 9 package, 51/51 unit, GUI P6 hoàn thành 1 chu trình với 13/13 kiểm tra.
Mesh hoạt động giống từng byte so với trước khi đổi tên; expanded URDF giống
mô hình trước sau khi ánh xạ tên và URI. Đã xác nhận robot hiển thị trong Gazebo.
[Kết quả](../results/model_layout/summary.json),
[so sánh mô hình](../results/model_layout/model_equivalence.json),
[video sau khi gộp](../results/model_layout/demo/mission_demo.mp4).
