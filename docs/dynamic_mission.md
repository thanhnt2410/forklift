# Lấy–giao pallet với A/B chọn trên RViz

## Luồng thao tác trong mô phỏng

Build lại cả interface và các node sau khi cập nhật. Hai máy ROS giao tiếp với
nhau phải dùng cùng bản `RunMission` mới; dừng các node cũ trước khi build/chạy lại.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
ros2 launch forklift_bringup p6.launch.py
```

Chờ `/navigation/status` là `READY` và càng đã homing. Trên RViz:

1. Chọn **Pick A** (phím A), click–kéo một pose **đứng trước pallet**, hướng càng
   vào pallet. Mặc định khoảng cách từ LiDAR tới mặt nhận dạng là 0,65 m.
2. Chọn **Drop B** (phím B), click–kéo **pose robot khi hạ hàng**, không phải tâm
   pallet. Chừa vùng trống cho hàng và đoạn lùi rút càng sau khi hạ.
3. Kiểm tra hai mũi tên A/B, rồi bấm **Run mission** trong panel **Pallet mission**.
4. **Stop mission** dừng nhiệm vụ, giữ vị trí cơ cấu nâng. **Clear A/B** xóa lựa
   chọn khi không có nhiệm vụ đang chạy.

Hai đầu vào phải thuộc hệ `map`, có quaternion hợp lệ và cách nhau ít nhất 0,30 m.

Không dùng nút Nav2 Goal để gửi chặng thứ hai khi nhiệm vụ đang chạy. Nhánh mới
sử dụng cách lập tuyến của code cũ: `ComputePathToPose` → công bố toàn tuyến →
`FollowPath`. Bốn profile `Pallet`, `Pallet11`, `Pallet13`, `Pallet15` có đoạn
cuối thẳng 0,9/1,1/1,3/1,5 m; chọn theo chiều dài + 0,6 × số đổi chiều +
0,12 × tổng góc quay, chỉ xét các đường dài không quá 1,5 lần đường ngắn nhất.
Cả A và B được kiểm tra với footprint có tải trước khi bánh xe được phép chạy.
Tuyến `approved_route.json` vẫn dành cho trạm cố định `a_to_b`; các pose A/B động
được lập tuyến mới bằng cùng phương pháp. Mission cũ vẫn giữ chu trình về Home.

Thử với world P6 mặc định: A xấp xỉ `(0.35, 0.70, yaw=0.06)` và B là pose robot
`(0.72, -0.60, yaw=0)`. Tâm pallet khi đặt xuống dự kiến gần `(0.95, -0.60)`.
Các tọa độ này chỉ dành cho world P6, không dùng làm tọa độ trên robot thật.

Không có GUI vẫn có thể publish `geometry_msgs/PoseStamped` vào
`/mission/pick_pose` và `/mission/drop_pose`, rồi gọi:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status
# Khi cần dừng:
ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
```

Hoặc gửi action trực tiếp, không cần node lưu lựa chọn:

```bash
ros2 action send_goal /mission/run forklift_interfaces/action/RunMission \
'{mission: pick_and_place, timeout: 600.0, pick_pose: {header: {frame_id: map}, pose: {position: {x: 0.35, y: 0.70}, orientation: {z: 0.0299955, w: 0.9995500}}}, drop_pose: {header: {frame_id: map}, pose: {position: {x: 0.72, y: -0.60}, orientation: {w: 1.0}}}}' --feedback
```

## Nhận dạng và hiệu chỉnh

```text
CHECK_READY → LOWER_FOR_PICK → DETECT_PICK_INITIAL → PLAN_MISSION → NAV_PICK → DETECT_PICK
  → [REFINE_PICK] → ORIENT_PICK → ALIGN → INSERT → LIFT_PICK
  → VERIFY_LOAD → RETREAT_PICK → NAV_DROP → LOWER_RELEASE
  → RETREAT_DROP → DONE
```

Ngay khi Run, robot giữ bánh đứng yên và thử nhận dạng quanh vùng A trong
2 giây. Nếu có năm scan mới ổn định và hiệu chỉnh nằm trong giới hạn, path
NAV_PICK đầu tiên dùng **A đã hiệu chỉnh**. Nếu chưa thấy mặt hoặc hiệu chỉnh
vượt giới hạn, robot giữ A sơ bộ và thử nhận dạng lại khi tới đó. Lỗi cảm biến,
TF gây mất readiness hoặc lệnh hủy vẫn dừng nhiệm vụ; không chuyển thành fallback.
`/mission/status.initial_detection` ghi `DETECTED`, `UNAVAILABLE` hoặc `OUT_OF_RANGE`,
pose đầu vào và pose đã sửa. Tham số node `initial_detection_timeout` mặc định
2 giây, chấp nhận 0–5 giây; giá trị 0 bỏ qua nhận dạng sớm. Launch riêng
`mission.launch.py` có argument cùng tên.

Khi tới gần, robot vẫn dừng và dùng năm scan khác timestamp để xác nhận một mặt có kích thước
đã cấu hình. Các mẫu phải nằm trong cửa sổ 1,5 s và ổn định trong 2,5 cm, 0,05 rad. Biến đổi TF dùng
frame và timestamp của scan; hỗ trợ cả offset và yaw của LiDAR. Từ tâm mặt và
pháp tuyến, robot tính lại pose đứng trước pallet. Sai khác goal trên 2,5 cm
hoặc 0,04 rad sẽ tính một path hiệu chỉnh ở nhánh dùng A sơ bộ, công bố và bám bằng `FollowPath`.
Nếu đã nhận dạng sớm, bước xác nhận gần so pose robot hiện tại với mặt pallet
vừa đo: sai lệch dọc tối đa 15 cm, ngang 6 cm và góc 0,10 rad có thể chuyển
thẳng sang docking, sau khi Nav2 xác nhận footprint trống trên đoạn quay,
căn ngang và tiến tới preinsert. Ngoài vùng này hoặc đoạn docking bị chặn,
robot lập thêm path REFINE_PICK. ALIGN và các ngưỡng cho phép INSERT được giữ
nguyên; dữ liệu xác nhận gần nằm trong `refinement`, gồm `arrival_error`,
`alignment_path_validated` và `replan_required` khi áp dụng.
AMCL còn thay đổi sau khi xuất phát hoặc pallet bị dịch chuyển vẫn có thể
làm bước kiểm tra gần yêu cầu REFINE_PICK dù đã nhận dạng được từ đầu.
NAV_PICK của nhiệm vụ động dùng `pickup_goal_checker` riêng: 6 cm, 0,10 rad
cho pose đứng trước pallet. B, REFINE_PICK và nhiệm vụ cũ vẫn dùng checker
chung 3,5 cm, 0,05 rad; các ngưỡng docking/INSERT không đổi. Stack Nav2 tự
ghép với `mission.launch.py` cần khai báo checker này như trong
`forklift_navigation/launch/navigation.launch.py` của repo.
Khi cần sửa ngang trong không gian gần, Nav2 có thể
lập một đoạn lùi ra trước rồi tiếp cận lại theo trục càng.
Đoạn nối ngắn được kiểm tra độ cong theo `minimum_turning_radius`; nếu không
đạt, planner phải tạo maneuver qua điểm vào hành lang thay vì nối tắt tới goal.
`/mission/route_markers` hiển thị toàn tuyến; `/mission/active_path` hiển thị
path thực đang bám. Mặc định ẩn `/plan` trong RViz mission vì topic đó chứa cả
các đường ứng viên chưa được chọn. Khi rút hàng xong, chặng B được tính lại từ
pose thực, với footprint có tải. Không chạy BT recovery của `NavigateToPose`.
Ở cuối đoạn/điểm đổi chiều, nếu còn lệch hướng, MPPI dừng tịnh tiến, sửa góc
theo hướng thân xe rồi mới tiếp tục; phép quay vẫn kiểm tra footprint có tải.
Sửa quá 0,25 m hoặc 0,35 rad bị từ chối. Vùng tìm và mặt phẳng quan sát được
cũng giới hạn khả năng bắt lại pallet; đây không phải phạm vi bảo đảm nhận dạng.

`/mission/pallet_pose` là mặt quan sát trong map;
`/mission/refined_pick_pose` là pose robot tiếp cận đã hiệu chỉnh.
`/mission/status.refinement` ghi pose ban đầu, pose sửa và số mẫu.

ALIGN dùng phản hồi LiDAR đã lọc để sửa ngang và góc. INSERT chỉ chạy dọc khi
đạt điều kiện căn chỉnh; mất mặt quan sát hoặc sai căn chỉnh sẽ dừng/báo lỗi.
Chỉ một nguồn Nav2 hoặc docking được motion mux chọn tại mỗi thời điểm.
Sau lỗi/hủy, không tự chạy lại; xử lý tải và cơ cấu nâng rồi mới gọi
`/mission/reset`. Kết thúc hoặc lỗi đều giữ nguồn docking với lệnh zero để
không phát lại goal Nav2 cũ.

B là vùng thả trống: điều hướng bằng AMCL/Nav2, hạ tại pose B, xác nhận hết tải
rồi lùi ít nhất 0,35 m, có kiểm tra LiDAR phía sau. Không có bước docking vào
một pallet hay marker tại B, không quay về Home.

## Giới hạn nhận dạng và độ chính xác

Detector hiện nhận dạng **mặt có bề rộng biết trước**, không phân loại mọi loại
pallet. Trong P6 đó là mặt kiện hàng có quan hệ hình học cố định với pallet.
Mặt thật phải nhìn thấy được ở cao độ quét và tâm/trục mặt phải trùng hoặc đã
hiệu chỉnh đúng với trục chèn càng. Chưa hỗ trợ tự suy ra lỗ càng từ pallet bất
kỳ. LiDAR CAD đang ở z=0,2946 m; cần kiểm tra đặc trưng quét trên pallet thật.

B không có mốc quan sát cục bộ nên sai số đặt hàng vật lý còn chịu sai số AMCL,
map và odometry. Kiểm thử tách **sai số robot so với goal B trong map** (mốc
kiểm tra 4 cm, 0,06 rad) khỏi **sai số pallet trong thế giới Gazebo** (vùng giao
hàng bán kính 15 cm). Đây không phải cam kết độ chính xác đặt hàng 4 cm ngoài
thực tế. Nếu yêu cầu vài milimét hoặc vài centimét tại B, cần thêm mốc quan sát
ở trạm thả và bước hiệu chỉnh tương đối tại B.

## Tích hợp robot thật

`navigation_real.launch.py` chỉ khởi chạy định vị, navigation, mission và RViz;
không mở cổng serial và không tự bật autonomous trên command gate. Các đầu vào:

- `/scan_raw` và TF `base_link → laser_frame` đã đo đúng.
- `/odom`, TF `odom → base_link` và `/joint_states` từ phần cứng.
- `/lift/home`, `/lift/move`, `/lift/stop`, `/lift/state` từ backend nâng đã hiệu chuẩn.
- `/pallet/load_detected` (`std_msgs/Bool`) từ cảm biến tải/tiếp xúc thật, phát
  liên tục, khoảng 10 Hz. `false` phải nghĩa là cảm biến còn hoạt động và không
  có tải. Không có heartbeat quá 0,6 s sẽ từ chối hoặc hủy nhiệm vụ.

**Cấu hình ESP32 hiện tại còn là nâng xung preset, chưa phù hợp chạy nhiệm vụ
nâng theo chiều cao.** Cần đo hành trình, hiệu chuẩn tọa độ nâng, đổi backend
sang chế độ position và kiểm tra homing trước. Không thay `Bool` bằng một giá
trị cố định để vượt kiểm tra tải. Phiên triển khai này không nạp firmware hoặc
chạy robot thật.

Khi phần cứng đã đáp ứng các hợp đồng trên, dừng SLAM mapping để AMCL sở hữu
`map → odom`, và chạy LiDAR với topic đầu vào navigation:

```bash
# Pi, thay cho phiên lidar đang publish /scan:
ros2 launch forklift_bringup lidar.launch.py scan_topic:=/scan_raw

# Laptop; backend phần cứng/driver nâng đã chạy riêng:
ros2 launch forklift_bringup navigation_real.launch.py map:=/duong/dan/map.yaml
```

Dùng **2D Pose Estimate** để khớp vị trí robot trên bản đồ trước khi chạy.
Launch dùng wheel odometry hiện có, không bật một EKF thứ hai. Mặc định mặt
nhận dạng rộng 0,24 m; cấu hình `pallet_face_width` và `staging_distance` theo
pallet thật. Thông số nâng/docking nằm trong `stations.yaml`; hình học cơ cấu
và footprint nằm trong `robot.yaml`, phải kiểm tra theo phần cứng.
Sau khi đã xác nhận phần cứng và định vị sẵn sàng, bật quyền nhận vận tốc tự động
ở command gate bằng `/command_gate/set_autonomous` (`SetBool`, `data: true`).

Có thể chạy riêng `mission.launch.py` để ghép với Nav2 có sẵn. Khi đó input
`velocity_smoother` phải là `/nav/cmd_vel_selected`; trong navigation launch
của repo, cấu hình này được bật bằng `p6:=true`.

## Kiểm thử lại

```bash
python3 scripts/run_p6_checks.py --dynamic --output results/dynamic_mission/nominal --domain 208
python3 scripts/run_p6_checks.py --dynamic --scenario rotated --output results/dynamic_mission/rotated --domain 209
```

Các scenario bổ sung: `early_direct` (định vị trước khi Run), `early_fallback`
(chưa thấy pallet ở đầu, sau đó pallet xuất hiện), `moved_pallet` (pallet dịch
chuyển sau nhận dạng sớm), `user_drop` (B đã chọn trong phiên debug),
`invalid_pick`, `invalid_drop`, `missing_pallet`, `cancel`, `scan_loss`, `cancel_loaded`,
`cancel_retreat`, `cancel_initial`, `scan_loss_initial`. Runner cô lập ROS domain và Gazebo partition, ghi config,
checksum, dữ liệu vật lý, trạng thái cuối, từng phép kiểm tra và cleanup.
Ground truth chỉ dùng ở checker, không đưa vào bộ điều khiển.

Kết quả triển khai ban đầu nằm trong
[báo cáo nghiệm thu mô phỏng](../results/final/dynamic_mission/README.md).
Kết quả sửa pipeline tạo path nằm trong
[báo cáo debug đường đi](../results/final/path_pipeline_fix/README.md).
Kết quả nhận dạng sớm và kiểm tra lại gần pallet nằm trong
[báo cáo nhận dạng lúc Run](../results/final/early_detection/README.md).

## Khi goal A quá sát pallet

A là pose **robot**, không phải tọa độ pallet. Ví dụ trong world P6, điểm
`(0.89, 0.57)` làm footprint/càng chồng vào pallet; hãy đặt A khoảng
`(0.35, 0.65)` với hướng càng về phía +X. Với goal không khả thi, bản hiện tại
báo `NO_VALID_PATH_NAV_PICK` hoặc `NO_VALID_PATH_NAV_DROP` trước khi chạy bánh.
Khi đã sửa A/B, robot không mang tải, càng đang thấp và navigation READY,
gọi `ros2 service call /mission/reset std_srvs/srv/Trigger '{}'` rồi bấm Run lại.
Sau khi cập nhật cần khởi động lại launch đang chạy để nạp node và các profile
planner mới; source lại workspace trong terminal trước khi launch.
