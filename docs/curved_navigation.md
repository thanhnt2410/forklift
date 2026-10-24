# Điều hướng tiến/lùi và lấy–giao pallet theo path đã duyệt

Phạm vi: ROS 2 Humble / Gazebo Fortress, CAD trong `forklift_description`, chỉ SIM.
Người dùng đã duyệt [bản xem trước](../results/bidirectional_preview/optimized/route_preview.png).
Các điểm gốc được lưu nguyên vẹn trong
[`approved_route.json`](../src/forklift_task_manager/config/approved_route.json).

## Thực hiện đúng tuyến đã xem

`PLAN_MISSION` nạp chín đoạn đã duyệt và công bố toàn bộ trước chuyển động.
Ba chặng `NAV_PICK`, `NAV_DROP`, `RETURN_HOME` dùng Nav2 **FollowPath** với các
đỉnh gốc; không tự chọn nhánh đường khác qua NavigateToPose.
Trước mỗi chặng, `/is_path_valid` kiểm tra lại costmap; sai điểm xuất phát quá
0,25 m hoặc đường bị chặn thì dừng/báo lỗi. Controller vẫn kiểm tra va chạm dự phóng.
Điểm đầu chỉ được cắt phần đã đi qua, không nối thêm đỉnh từ vị trí lệch của xe.

- `/mission/route` và `/mission/route_markers`: toàn nhiệm vụ, gồm cả sau nâng và thả.
- `/mission/active_path`: tuyến đang chạy, hiển thị màu cam trong RViz.
- `/mission/route_info`: nguồn `approved_preview`, SHA256, các đoạn và điểm nâng/thả.
- `/plan`: dành cho chế độ lập đường trực tiếp; không phải nguồn path của mission đã duyệt.

LiDAR vẫn điều khiển ALIGN/INSERT; odometry/EKF giữ hướng khi rút. Các đoạn docking
trong bản xem trước là dự kiến hình học, không thay thế phản hồi cảm biến.

## Tiến và lùi trên toàn hành trình

| Chặng | Tuyến đã duyệt |
|---|---|
| Tới A | Lùi theo cung, đổi sang tiến; kết thúc bằng 1,30 m thẳng vào trục pallet |
| Mang A → B | Rút thẳng tối thiểu 0,35 m thoát trạm, tiếp tục lùi theo cung; kết thúc 0,90 m tiến thẳng |
| Về Home | Sau rút 0,35 m có đoạn tiến ngắn mở cung, rồi lùi phần lớn đường; hướng đỗ -75° |

Khoảng rút thực tế có thể dài hơn 0,35 m để tới điểm rút trong map khi có sai số định vị.
Nếu vùng quét phía sau bị chặn thì dừng. Home có tọa độ `(0,0)`; -75° là hướng
đỗ được chọn trong tập ứng viên và đã nằm trong bản duyệt.
Tổng chiều dài hình học chín đoạn khoảng 7,55 m, phần lùi khoảng 4,55 m.
Không áp dụng khoảng rút cố định 1,80 m của cấu hình cũ.

Nav2 nhận footprint đúng theo tải: `xmax=0,52 m` khi mang pallet,
`xmax=0,4513 m` khi không tải; `xmin=-0,1751 m`, nửa rộng `0,225145 m` theo CAD.
Sau thả, pallet là vật cản riêng. Giữ footprint có tải sau thả sẽ báo va chạm giả
ở đoạn tiến ngắn về Home. Kiểm tra va chạm luôn bật.

## Planner và ổn định khi đổi chiều

Chế độ lập đường trực tiếp cũ: `TerminalApproachPlanner` dùng Smac Hybrid-A* / REEDS_SHEPP cho phần tới entry,
rồi nối đoạn thẳng đúng hướng càng. Bán kính cua tối thiểu 0,50 m; hệ số chi phí
lùi và cong đều 1,35. Cho phép tiến/lùi khi hữu ích, ưu tiên thẳng qua chi phí;
không bảo đảm tối ưu toàn cục hoặc cấm mọi đường chữ S.

Ở chế độ trực tiếp, planner giữ tuyến đã chọn khi vẫn hợp lệ. Sai góc bám tuyến gần điểm đổi chiều
không tự tạo một vòng mới về đầu hành lang. Khi cắt phần đã đi qua, giữ nguyên
các đỉnh/tangent của path; không chèn vị trí lệch của robot thành điểm đầu mới
vì có thể tạo ra một đoạn ngược chiều giả.

Ở chế độ đối chiếu `controller:=rpp`, `StableReversingController` kế thừa Regulated Pure Pursuit, giữ điều chỉnh tốc độ
và kiểm tra va chạm. `allow_reversing=true`, `use_rotate_to_heading=false` theo
ràng buộc của RPP. Bộ bổ sung khôi phục hướng:

1. Khi lệch hướng path quá 0,40 rad, hoặc sát goal nhưng chưa đủ đúng góc, dừng.
2. Chờ tốc độ tịnh tiến dưới 0,008 m/s và quay dưới 0,02 rad/s.
3. Sửa góc tại chỗ tối đa 0,18 rad/s, kiểm tra footprint dự phóng mỗi lệnh.
4. Chờ ngừng quay rồi tiếp tục bám tuyến. Hướng lấy từ quaternion của path,
   nên lùi không làm xe tự quay 180°.

Cơ chế này xử lý sai góc tại điểm chuyển tiến/lùi và cuối tuyến; các cung cua
bình thường vẫn được bám bằng `vx` và `wz`. Có vật cản khi sửa góc thì dừng.

## Vận tốc và chu trình

| Chuyển động | Giới hạn hiện tại |
|---|---:|
| Nav2 tiến | 0,25 m/s |
| Nav2 lùi | 0,16 m/s |
| Rút thẳng | 0,08 m/s |
| ALIGN tiến | 0,035 m/s |
| INSERT | 0,015 m/s |
| ALIGN ngang | 0,02 m/s |
| Nav2 ngang | 0 |

Giảm tốc theo độ cong, vật cản và khoảng cách tới đích. Smoother giữ tỷ lệ các
thành phần khi chạm giới hạn. Dung sai Nav2: 0,035 m và 0,05 rad.
Motion mux chọn độc quyền Nav2 hoặc docking; command gate vẫn kiểm tra nâng,
watchdog, E-stop và giới hạn tốc độ bánh.

`NAV_PICK → ORIENT_PICK → ALIGN → INSERT → LIFT_PICK → VERIFY_LOAD →
RETREAT_PICK → NAV_DROP → ORIENT_DROP → ALIGN → INSERT → LOWER_RELEASE →
RETREAT_DROP → RETURN_HOME → DONE`.

ORIENT dùng LiDAR và Nav2 Spin có kiểm tra va chạm để sửa sai số còn lại.
INSERT chỉ ra lệnh dọc; lệch góc/ngang quá giới hạn thì dừng. Nâng/vận chuyển dùng
tiếp xúc pallet–càng thực trong Gazebo, không attach/teleport. Giữ AMCL OmniMotionModel
và wheel odometry + MPU6050/EKF vì xe vẫn có dịch ngang nhỏ khi docking.

## Kiểm chứng và chạy lại

Người dùng đã tạm chốt MPPI; mặc định là `controller:=mppi`:
`ros2 launch forklift_bringup p6.launch.py`. Chọn `controller:=rpp` để đối chiếu.
Overlay `nav2_mppi.yaml` chỉ thay controller;
AMCL, map, path đã duyệt, giới hạn tốc độ và docking giữ nguyên.
Adapter kiểm tra reference phía trước, hành lang bám tuyến và footprint dự đoán;
đường bị chặn thì dừng. Hai lượt thử MPPI và lượt demo GUI đều đạt 29/29.
RMS vật lý toàn tuyến ở hai lượt thử lớn hơn baseline RPP; việc chọn MPPI là quyết
định tiếp tục của người dùng, không phải kết luận MPPI bám tốt hơn trên mọi chặng.
Xem [báo cáo MPPI](../results/mppi_trial/README.md).
Runner hỗ trợ `--controller mppi`, `--scenario blocked_nav` (vật cản xuất hiện trên
path đang chạy) và `--scenario scan_loss_nav` (mất LiDAR trong lúc bám path).
Theo yêu cầu mới, không tự chạy thêm kiểm thử tránh vật cản ở các lượt tiếp theo.

Đánh giá bám quỹ đạo mới: [`results/trajectory_tracking`](../results/trajectory_tracking/README.md).
Ba cấu hình Pure Pursuit và một cấu hình AMCL đã được thử; cấu hình Pure Pursuit
có RMS tốt nhất ở lượt đầu không giữ được mức cải thiện khi chạy lại. Cấu hình
Nav2/AMCL mặc định được khôi phục nguyên bản. Chưa coi đây là nghiệm thu cải thiện
bám tuyến. Bộ kiểm tra bổ sung pose trong hệ `map` và bộ phân tích khoảng cách tới
polyline, sai số điểm dừng, thống kê riêng tiến/lùi. Chênh map/Gazebo còn bao gồm
khác biệt thời điểm lấy mẫu và sai lệch hệ bản đồ; không quy toàn bộ cho AMCL.

Bằng chứng triển khai tuyến đã duyệt: [`results/approved_route_sim`](../results/approved_route_sim).
Các kết quả `motion_stability`, `reverse_navigation`, `terminal_approach` thuộc
cấu hình trước, được giữ làm lịch sử. Không cộng gộp các lượt thành nghiệm thu nhiều chu trình.
Bộ kiểm tra ghi pose vật lý, tiếp xúc/tải, roll/pitch, lệnh vận tốc, path/SHA,
lùi theo cung có tải, lùi về Home, thứ tự giai đoạn và va chạm.
Ground truth chỉ đi vào bộ kiểm tra, không làm đầu vào điều khiển.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch forklift_bringup p6.launch.py
# Terminal khác đã source workspace:
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission '{mission: a_to_b}' --feedback
```

Kiểm chứng tách biệt, runner tự dọn theo domain/partition:

```bash
python3 scripts/run_p6_checks.py --output results/approved_route_sim/recheck --domain 207
python3 scripts/analyze_motion.py results/approved_route_sim/recheck
```

Bước vật lý giữ 1 ms. Đã sửa cleanup để tránh các phiên cũ chạy ngầm làm giảm RTF;
`cleanup.json` ghi các tiến trình còn lại. Tốc độ không được tăng để bù mô phỏng chậm.

Tuyến này dành cho vị trí trạm và bản đồ đã duyệt. Khi thay trạm/bố trí hoặc muốn
đổi tuyến, cần tạo và xem lại path. Tham số node `approved_route_file` rỗng cho
phép quay về cơ chế lập đường trực tiếp cũ; mặc định vẫn chạy file đã duyệt.
Chưa kiểm chứng lại ma trận nhiều chu trình/toàn bộ lỗi sau thay đổi này.

Kết quả cuối: lấy–giao–Home 29/29, hủy khi lùi 9/9, mất LiDAR 10/10; 60 unit Python và 5 trường hợp C++ đạt. RTF chu trình 0,989; cleanup cả ba lượt không còn tiến trình.
