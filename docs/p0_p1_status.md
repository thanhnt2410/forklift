# Kết quả thực hiện P0 và P1

Ngày: **23/10/2026**. Workspace: `/home/thanh/ros_ws/forklift`.

## Trạng thái

| Phần | Kết quả |
|---|---|
| Nguồn tham chiếu P0 | Đã clone đầy đủ, khóa commit và khảo sát mã; người dùng đã chuyển nội dung repo ra workspace gốc |
| Số đo P0 | Đã nhập bánh 100 mm, tâm ngang 260 mm, tâm dọc 195 mm; thông số khác còn chờ đo thực |
| Gate G0 | Demo Fortress đạt 3/3 phép thử sau khi sửa hướng ma sát cho Gazebo 6 |
| Package P1 | Build thành công 4 package `forklift_*`; `check_urdf` và chuyển URDF → SDF thành công |
| Gate G1 — minimal | **37/37** kiểm tra tích hợp đạt |
| Gate G1 — warehouse | **37/37** kiểm tra tích hợp đạt, có Gazebo GUI và RViz |
| Phần cứng / P2 trở đi | Chưa nạp firmware, chưa nối MCU, chưa kiểm chứng robot thật, Nav2 hoặc SLAM |

P0 đã hoàn thành công việc phần mềm có thể kiểm chứng trên máy phát triển.
Hạng mục đo/khóa toàn bộ yêu cầu cơ khí chưa hoàn thành do thiếu số đo thực tế;
không đánh dấu các dữ liệu chưa biết thành thông số đã xác nhận.

## Dữ liệu nghiệm thu

- [Baseline Fortress](../results/p0/baseline.json): vị trí vật lý sau tiến/ngang/xoay và tốc độ bánh sau timeout.
- [37 kiểm tra minimal](../results/p1_final/checks.json), [log](../results/p1_final/test.log),
  [telemetry CSV](../results/p1_final/telemetry.csv), [đồ thị](../results/p1_final/simulation_results.png).
- [37 kiểm tra warehouse](../results/p1_warehouse/checks.json),
  [telemetry](../results/p1_warehouse/telemetry.csv), [đồ thị](../results/p1_warehouse/simulation_results.png),
  [video Gazebo](../results/p1_warehouse/demo.mp4).
- [Build log](../results/p1_final/build.log), [kiểm tra URDF](../results/p1_final/urdf_check.txt).
- [Ảnh Gazebo](../results/p1_final/gazebo.png), [ảnh RViz](../results/p1_final/rviz.png) sau khi xác minh trực quan.
- [Phiên bản package](../results/p0/packages.tsv), [manifest upstream](../results/p0/upstream_manifest.json).

Các kiểm tra bao gồm: ba controller active; scan 360 tia và frame đúng; xe đứng
yên ổn định; tiến/lùi, trái/phải, xoay, kết hợp; timeout; chế độ manual/auto và
teleop ưu tiên; lệnh NaN và giới hạn tốc độ; nâng/hạ, giới hạn chiều cao, quay lại
từ mức tối đa, cancel; khóa di chuyển khi nâng/càng cao; E-stop phần mềm,
không phát lại lệnh cũ; TF odom, một publisher odom và một publisher lệnh cuối;
odometry đối chiếu với vị trí vật lý; dữ liệu phản hồi còn mới.

Số liệu từ lần chạy minimal lưu tại đây:

- Tiến với 0,10 m/s trong 2,5 giây: khoảng **0,238 m** sau ramp.
- Đi ngang với 0,08 m/s: khoảng **0,192 m**.
- Sai lệch vị trí lớn nhất giữa odom và physics: **0,02371 m**.
- Thời gian từ lệnh cuối tới đầu ra gate bằng zero: khoảng **0,31–0,32 s**.
- Nâng mục tiêu 0,10 m: phản hồi khoảng **0,09991 m**.
- Nâng mục tiêu 0,25 m: phản hồi khoảng **0,24990 m**; hạ về khoảng **0,000097 m**.

Đây là số liệu **mô phỏng**, không phải độ chính xác robot thật hay tải trọng đã
được kiểm chứng. Timeout là độ trễ đầu ra phần mềm, không phải quãng đường dừng cơ học.

## Hai lỗi tương thích đã khắc phục

1. Demo cài sẵn sử dụng `gz:expressed_in` cho ma sát. Physics Fortress trên máy
   nhận `ignition:expressed_in`. Đổi namespace trên bản sao demo và model P1,
   sau đó đo được chuyển động ngang vật lý. Không sửa `/opt/ros`.
2. Khớp nâng bị DART khóa khi trọng lực làm vượt giới hạn, hoặc servo chạm đúng
   giới hạn vận tốc. Mô hình P1 dùng giới hạn cơ học SIM `[-0,005; 0,255] m`,
   giữ miền lệnh `[0; 0,25] m`, vận tốc trajectory 0,04 m/s và headroom vật lý
   0,10 m/s. Đã thử cả hai đầu hành trình và quay lại. Tham khảo
   [issue #192](https://github.com/ros-controls/gz_ros2_control/issues/192) và
   [issue #1684](https://github.com/gazebosim/gz-sim/issues/1684).

Các phiên thử lỗi ban đầu được giữ trong `results/diagnostics/`; kết quả nghiệm
thu cuối nằm ở `results/p1_final` và `results/p1_warehouse`.

GUI mặc định đã được mở lại sau build cuối. Marker odometry được thu nhỏ và tắt
hiển thị covariance 3D để phù hợp robot nhỏ; RViz hiển thị robot/scan với Global
Status OK. Một bản tin scan cũ bị bộ lọc TF bỏ qua lúc startup; scan sau đó liên tục.

## Cấu trúc, dữ liệu tạm và giới hạn

- Mã nguồn hoạt động ở `src/forklift_description`, `src/forklift_control`,
  `src/forklift_gazebo`, `src/forklift_bringup`.
- `build/` và `install/` đã được dựng lại sau khi người dùng chuyển thư mục;
  symlink hiện trỏ vào `src/forklift_*`, không còn trỏ repo lồng đã bỏ.
- Các package Classic còn giữ nguyên và có `COLCON_IGNORE`.
- Chỉ có bán kính và khoảng cách tâm bánh được lấy từ kích thước người dùng.
  Khung, khối lượng, inertia theo khối cơ bản, càng, hành trình và vị trí LiDAR
  là mô hình tạm; phải thay bằng số đo trước REAL.
- Collision cầu và ma sát dị hướng xấp xỉ mecanum, chưa mô phỏng từng con lăn.
- LiDAR là mô hình GPU; chưa xác nhận đặc tính đo của X4 thật.
- Cơ cấu nâng P1 là actuator vị trí lý tưởng, chưa mô phỏng NEMA23 mất bước,
  homing, giữ tải hoặc giới hạn điện/cơ khí thật. Các phần đó thuộc P2/P5.
- Phần mềm dừng và khóa liên động SIM không thay thế E-stop phần cứng.
- Chưa ghi nhận benchmark hiệu năng Pi 4B; không dùng kết quả máy x86 để cam kết Pi.

## Thông tin cần bổ sung trước P2

Hoàn thành [robot_measurements.yaml](../config/robot_measurements.yaml): model
ESP32; CPR encoder tại trục bánh và tỉ số truyền; RPM/dòng motor; chiều rộng
bánh, khung và khối lượng; bước vít/vi bước/hành trình; tải dự kiến; công tắc
hành trình, E-stop và nguồn. Chưa dùng giá trị tạm SIM để chọn pin, driver hoặc tải nâng.

Cách build, teleop, nâng/hạ và tái hiện kiểm thử nằm trong [README](../README.md).
