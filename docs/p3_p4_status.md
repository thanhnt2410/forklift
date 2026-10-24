# Báo cáo P3/P4 — chỉ mô phỏng

Triển khai theo yêu cầu ngày 23/10/2026. Không nạp ESP32, mở serial thật hoặc
điều khiển xe thật. [Hướng dẫn chạy](p3_p4_simulation.md).

## P3: hardware interface với MCU giả

Đã thêm `forklift_hardware/PtySystem`, serial worker riêng, giao thức có CRC32,
sequence và boot ID. Odometry lấy từ phản hồi encoder lượng tử của MCU giả.
Worker phát hiện telemetry quá hạn, frame hỏng, reset và ngắt kết nối; lỗi được
giữ, không phát lại lệnh chuyển động cũ. Plugin chỉ chấp nhận PTY `/dev/pts/*`.

- Kiểm tra tích hợp: **24/24 đạt**, [tổng hợp](../results/p3/summary.json).
- Bao gồm tiến 1 m, ngang 1 m, quay 360°, đóng băng encoder trong khi có lệnh,
  mất telemetry, CRC hỏng, reboot và disconnect.
- Kiểm tra C++: **7 kết quả test, không lỗi**, gồm parser và từ chối cổng thật;
  [log](../results/p3/unit_tests.log). Số này gồm kết quả wrapper CTest/gtest.

CPR, đáp ứng motor và hệ số bước nâng của MCU giả là tham số mô phỏng.
Chưa có PID motor, encoder thực hoặc hiệu chuẩn sàn thật.

## P4: SLAM, AMCL và Nav2 trong Gazebo

Đã tạo map bằng SLAM Toolbox từ scan và odometry khi khảo sát kho. Map có
158 × 156 ô, độ phân giải 0,04 m; [dữ liệu khảo sát](../results/p4/mapping.json),
[map](../src/forklift_navigation/maps/warehouse.yaml).
Chế độ localization nạp lại map, dùng AMCL `OmniMotionModel` và Nav2 DWB có
`vx/vy/wz`. Footprint gồm càng; ObstacleFootprint kiểm tra va chạm theo hình xe.

DWB dùng cùng ngưỡng vị trí 0,06 m cho goal checker và chuyển sang căn góc.
Ngưỡng 0,025 m đã gây đứng gần đích trên costmap 0,04 m trong thử nghiệm;
không giữ cấu hình đó. Số mẫu góc lẻ có mẫu zero, lookahead đường cục bộ 0,6 m.
Cho phép đổi chiều căn chỉnh, progress checker tiếp tục phát hiện đứng lâu.
Đã bổ sung critic `ExactGoal` chấm khoảng cách liên tục tới pose đích để phá
trường hợp các quỹ đạo cùng điểm khoảng cách theo ô costmap. Path/collision
critics vẫn giữ nguyên vai trò ràng buộc đường và footprint. Hai bài C++ kiểm
tra phân biệt vị trí trong cùng ô và từ chối quỹ đạo rỗng. Điểm mở rộng theo
[API DWB Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_core/include/dwb_core/trajectory_critic.hpp).

Lượt nghiệm thu cuối đạt **34/34 kiểm tra**: **10/10 đích** và **24/24** kiểm tra
khởi động/vật cản/dừng lỗi. Sai số lớn nhất trong 10 đích là **0,1336 m**,
góc **6,63°**; đích sau vật cản lệch **0,1191 m**.
[Tổng hợp máy đọc](../results/p4_final/acceptance_summary.json),
[biểu đồ](../results/p4_final/navigation_summary.png).
[Video demo RViz](../results/p4_final/navigation_demo.mp4) được ghi riêng bằng
cấu hình cuối, sau lượt nghiệm thu; không phải video của toàn bộ 34 bài.
Kết quả từng đích nằm ở
[navigation_checks.json](../results/p4_final/navigation_checks.json).
Sai số đối chiếu với pose vật lý Gazebo, không lấy AMCL tự đánh giá chính nó.
Bộ đích gồm 10 pose để phủ tiến/ngang/lùi/quay; đây là biến thể SIM,
chưa thay thế phép lặp 5 đích × 2 và đo ngoài trên xe thật trong T5.

Các kiểm tra bổ sung: vật cản mới xuất hiện giữa đường, costmap có ô lethal,
đi vòng và không chồng footprint vào vật cản; cancel; mất scan; mất TF map–odom;
khôi phục dữ liệu chưa tự cho xe chạy lại, cần reset chủ động.
Kiểm tra guard/gate Python **6/6 đạt**. Hồi quy P1 **37/37 đạt** ở
`results/p1_regression_p3_p4/`.

## Giới hạn và phần tiếp theo

P3 PTY và P4 Gazebo là hai backend chạy riêng, dùng chung geometry/controller
contract. Ground truth chỉ dùng trong kiểm tra, không cấp vào odom/định vị.
AMCL khởi tạo tại pose spawn đã biết của mô phỏng.

Xe tự tới pose được chỉ định; chưa nhận dạng pallet, tự đưa càng vào kiện hoặc
nâng/giao hàng theo nhiệm vụ. Ở mốc P3/P4 chưa triển khai P5/P6; nay đã có [P5-SIM](p5_lift.md), P6 vẫn chưa triển khai. X4 thật,
Pi, PID ESP32, giới hạn cơ khí, tải và E-stop vật lý vẫn chưa nghiệm thu.
Các thư mục `p4_obstacle`, `p4_return` và log thử trong `p4` lưu các vòng chẩn đoán;
lượt GUI ban đầu ở `p4_gui_trial`, lượt siết tolerance chưa có critic ở `p4_tolerance_trial`; kết quả cuối cùng ở `p4_final`, không cộng chọn các lượt tốt thành một lần chạy.

## Sửa thao tác goal RViz

Cấu hình ban đầu thiếu panel `nav2_rviz_plugins/Navigation 2`: GoalTool chỉ cập
nhật GoalUpdater, cần panel để gửi action. Đã bổ sung panel vào cấu hình mặc định
và kiểm tra trực tiếp kéo thả chuột; kết quả ở
[rviz_click_check.json](../results/p4_final/rviz_click_check.json). Các bài kiểm tra
action trước đây không bao phủ lỗi ghép nối GUI này.
