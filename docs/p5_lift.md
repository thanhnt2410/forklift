# P5 — nâng hạ có giám sát trong mô phỏng

> Cập nhật CAD URDF20: mô hình mặc định đã đổi sang kích thước CAD.
> Xem [tích hợp URDF20](forklift_model.md). Kết quả kiểm thử cũ bên dưới thuộc mô hình trước khi đổi CAD.

Theo phạm vi người dùng yêu cầu: chỉ Gazebo/ROS 2, không nạp ESP32 hoặc mở cổng
phần cứng. Có homing, action có feedback/cancel/timeout, công tắc ảo, khóa xe và
kiểm tra vị trí vật lý. Chưa triển khai nhiệm vụ tự nhận dạng/lấy/giao pallet P6.

## Kiến trúc và giao diện

```text
/lift/home (HomeLift) ─┐
/lift/move (MoveLift) ─┴→ lift_supervisor → lệnh có sequence + heartbeat
                                               ↓
                                     lift_sim_io + watchdog
                                               ↓
                           JointTrajectoryController action → khớp Gazebo
                                               ↓
                              joint states → công tắc ảo + LiftSensors
                                               ↓
                                      lift_supervisor
/lift/moving + joint states → command_gate → khóa chuyển động xe
```

- Package mới: `forklift_interfaces`, `forklift_lift`.
- Action `/lift/home`: timeout giây, `0` chọn mặc định 60 s.
- Action `/lift/move`: `height` mét, `timeout` giây; timeout `0` được tính theo
  quãng đường và vận tốc. Không nhận goal nếu chưa homing, đang có goal khác,
  xe đang chạy, E-stop, fault hoặc phản hồi không hợp lệ.
- `/lift/state`: trạng thái có kiểu, homed, fault, height, target, offset và limit.
- `/lift/status`, `/diagnostics`, `/lift/marker`: quan sát trạng thái và lỗi.
- `/lift/moving`: **tín hiệu khóa xe**, true cả khi chưa homing/fault/mất dữ liệu;
  trường `moving` trong `/lift/state` mới thể hiện action đang thực thi.
- `/lift/stop`: dừng và giữ vị trí; action hiện tại kết thúc không thành công.
- `/lift/reset`: chỉ chấp nhận sau khi nguyên nhân lỗi đã hết và phản hồi hợp lệ;
  đưa về **UNHOMED**, không tự phát lại lệnh. Luôn homing lại sau reset.
- `/lift/sim_io/reset`: reset lỗi controller/watchdog ở lớp I/O khi đã dừng và
  nguyên nhân đã hết. Supervisor vẫn giữ fault cho đến reset riêng.

Các topic/action `/lift_controller/*` là giao diện nội bộ; thao tác trực tiếp có
thể bỏ qua supervisor. Đây là kiến trúc điều khiển, không phải cơ chế phân quyền ROS.
Backend PTY P3 vẫn dùng helper cũ để kiểm tra UART, chưa có công tắc độc lập.

## Homing và giới hạn

```text
UNHOMED → HOME_SEEK (nếu chưa chạm công tắc dưới)
         → HOME_BACKOFF (lùi khỏi công tắc và kiểm tra đã nhả)
         → HOME_APPROACH (dò lại chậm)
         → HOME_SETTLE (giữ, xác nhận đã đứng yên)
         → HOMED / IDLE
```

Mốc zero được ghi tại vị trí latch sau dò chậm. **Height là hành trình tính từ
mốc homing**, không phải độ cao càng so với sàn. Đích raw gửi controller là
`home_offset + height`.

Thông số SIM trong [lift.yaml](../src/forklift_lift/config/lift.yaml):

| Đại lượng | Giá trị |
|---|---:|
| Vận tốc nâng vận hành | 0,04 m/s |
| Tìm công tắc lần đầu | 0,02 m/s |
| Dò chậm lần cuối | 0,006 m/s |
| Lùi khỏi công tắc | 0,008 m |
| Ba mức thử | 0,05 / 0,12 / 0,20 m |
| Giới hạn phần mềm P5 | 0–0,24 m từ mốc homing |
| Công tắc trên ảo | joint raw 0,245 m |
| Hành trình URDF / stop vật lý | 0,25 m / khoảng đệm số học 5 mm |
| Mất phản hồi supervisor | 0,4 s |
| Watchdog I/O khi mất heartbeat | 0,3 s |

Trajectory có vận tốc cuối zero; supervisor chỉ báo thành công sau khi đủ gần
đích và vận tốc nhỏ ổn định. Core còn phát hiện không tiến triển, timeout,
limit bất thường, controller bị hủy/abort và xe bắt đầu chạy trong lúc nâng.

I/O watchdog độc lập giữ vị trí **thực đang đo**, không quay về vị trí phản hồi
cũ của supervisor. Đã thử treo supervisor bằng SIGSTOP; khi tiến trình trở lại,
goal cũ bị abort và mất homed. E-stop được cả supervisor lẫn I/O theo dõi.

## Chạy thử

```bash
cd /home/thanh/ros_ws/forklift
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_bringup p5.launch.py
```

RViz mở góc nhìn gần và hiển thị trạng thái càng. Ban đầu UNHOMED, xe bị khóa.
Trong terminal khác đã source workspace:

```bash
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}' --feedback
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.12}' --feedback
ros2 action send_goal /lift/move forklift_interfaces/action/MoveLift '{height: 0.0}' --feedback
# Dừng ngay trong mô phỏng; Ctrl+C ở action client không đảm bảo hủy goal:
ros2 service call /lift/stop std_srvs/srv/Trigger '{}'
```

`navigation.launch.py` và `mapping.launch.py` hiện dùng supervisor và **auto-home
một lần lúc khởi động trong SIM**. Có thể tắt bằng `auto_home:=false` rồi home
bằng action. Launch `sim`/`p5` mặc định không tự home. Khi lỗi xuất hiện, không có
auto-home hoặc tự bỏ fault. Helper cũ chỉ còn ở `lift_mode:=legacy` cho hồi quy P1/P3.

## Chèn lỗi SIM và phục hồi

```bash
ros2 param set /lift_sim_io lower_mode stuck_active
ros2 param set /lift_sim_io lower_mode normal
# Các chế độ khác: stuck_inactive, disconnected; cũng có upper_mode.
ros2 param set /lift_sim_io drop_feedback true
ros2 param set /lift_sim_io drop_feedback false
ros2 param set /lift_sim_io jammed true
ros2 param set /lift_sim_io jammed false
ros2 service call /command_gate/estop std_srvs/srv/SetBool '{data: true}'
ros2 service call /command_gate/estop std_srvs/srv/SetBool '{data: false}'
```

Sau khi nguyên nhân đã được xử lý và cơ cấu đã đứng yên:

```bash
ros2 service call /lift/sim_io/reset std_srvs/srv/Trigger '{}'
ros2 service call /lift/reset std_srvs/srv/Trigger '{}'
ros2 action send_goal /lift/home forklift_interfaces/action/HomeLift '{}' --feedback
```

Switch kẹt active/inactive được kiểm chứng qua trình tự nhả/dò công tắc.
`disconnected` dùng cờ validity của **kênh chẩn đoán giả lập**; không chứng minh
mạch NC một dây trên xe thật có thể phân biệt đứt dây với nhấn công tắc.
`jammed` mô phỏng một lỗi actuator đã được chẩn đoán; lỗi cơ cấu không có chẩn
đoán còn được phát hiện bằng điều kiện không tiến triển/timeout.

[Video homing/nâng/hạ cận cảnh](../results/p5/preview/lift_preview.mp4) ·
[Video đầy đủ dừng lỗi](../results/p5/demo/lift_demo.mp4) ·
[Ảnh trạng thái cuối](../results/p5/demo/rviz_finished.png) ·
[Tổng hợp kết quả](../results/p5/summary.json)

## Kiểm tra và bằng chứng

- Không tải: [38/38](../results/p5/unloaded/summary.json), bao gồm **10 lần tại
  mỗi mức 5, 12, 20 cm**. Sai số chấm từ chênh lệch pose vật lý link
  `lift_carriage` so với lúc homing, không lấy target làm phép đo.
- Khối lượng ảo 0,1 kg: [38/38](../results/p5/test_mass_0p1kg/summary.json), gồm **30/30** lượt chiều cao; [biểu đồ](../results/p5/lift_results.png).
- Ma trận lỗi/interlock: [39/39](../results/p5/faults/summary.json), gồm cancel,
  timeout, switch kẹt, kênh disconnected, actuator fault, hủy controller bên ngoài,
  mất feedback, E-stop, treo supervisor và không phát lại lệnh.
- Nav2 + EKF + supervisor mới: [38/38, 10/10 đích](../results/p5/navigation/acceptance_summary.json).
- Demo có tải ảo: [8/8](../results/p5/demo/summary.json), có đo pose vật lý khi E-stop và mất feedback; càng dừng và giữ độ cao.
- Core và hồi quy guard/gate/IMU: **19/19**: [log unit test](../results/p5/unit_tests.log).

```bash
python3 scripts/run_p5_checks.py --nominal-only --output results/p5/unloaded
python3 scripts/run_p5_checks.py --repetitions 1 --output results/p5/faults
python3 scripts/run_p5_checks.py --domain 123 --nominal-only --test-mass 0.1 \
  --output results/p5/test_mass_0p1kg
python3 scripts/run_p4_checks.py navigation --domain 125 --output results/p5/navigation
```

## Khối lượng thử và giới hạn nghiệm thu

`lift_test_mass:=0.1` tạo một khối **0,1 kg gắn cứng** vào carriage trong Gazebo,
để kiểm tra ảnh hưởng khối lượng trong mô phỏng. Không phải lấy pallet qua tiếp
xúc, không phải tải trọng được duyệt của xe thật. Trọng lực vẫn bật.

Khi có khối lượng thử, launch bắt đầu pause, kích hoạt controller nâng trước bằng
các bước physics ngắn rồi chạy liên tục; bánh được khởi tạo chạm sàn. Các lượt
khởi tạo cũ từ cao 5 mm đã gây va đập/chạm hard stop DART và bị loại; log vẫn giữ
trong các thư mục `results/p5/test_mass_*failure`. Không nới tiêu chí sai số hoặc
tắt trọng lực để đạt bài thử.

Gazebo dùng actuator vị trí lý tưởng; sai số nhỏ trong SIM không đại diện vít me,
NEMA23/TB6600 hoặc tải thật. Hệ số STEP/mét, vi bước, homing vật lý, công tắc,
chống tụt tải, E-stop khi mất nguồn và độ lặp lại bằng thước đo **chưa nghiệm thu**.
P5-SIM không thay thế gate G5 trên xe thật.
