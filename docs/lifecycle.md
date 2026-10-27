# Quản lý lifecycle

Forklift dùng `nav2_lifecycle_manager`, theo cách tổ chức của bumperbot:

| Chế độ | Manager | Node được quản lý |
|---|---|---|
| Mapping thật/mô phỏng | `lifecycle_manager_mapping` | `map_saver_server` |
| Localization | `lifecycle_manager_localization` | `map_server`, `amcl` |
| Navigation | `lifecycle_manager_navigation` | `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother` |

SLAM Toolbox 2.6.10 trên Humble kế thừa `rclcpp::Node`, không hỗ trợ lifecycle.
Giống nhánh Humble của bumperbot, không đưa `slam_toolbox` vào `node_names`.
LiDAR, TF publisher, scan filter/relay, EKF và các node điều khiển thường cũng
không được đưa vào danh sách này. Lỗi thoát tiến trình SLAM, scan filter/relay,
saver hoặc manager trong launch mapping sẽ đóng phiên mapping.

`map_saver_server` là executable `paired_map_saver` của forklift, thay cho stock
map saver để giữ yêu cầu một lần Save Map lưu cả navigation và probability.
Nó dùng lifecycle Nav2 và bond heartbeat, mặc định timeout bond 4 giây.

- `unconfigured`: chưa tạo dịch vụ Save Map công khai.
- `inactive`: đã configure, dịch vụ có nhưng từ chối lưu (`result: 255`).
- `active`: nhận Save Map từ RViz/CLI, chuyển đến backend lấy snapshot SLAM.
- `cleanup`: bỏ timer/client/service, về unconfigured; có thể configure lại.
- `shutdown`: giải phóng tài nguyên, về finalized.

Public endpoint vẫn là `/slam_toolbox/save_map`. Backend snapshot nội bộ là
`/slam_toolbox/save_maps`; không dùng backend này để bỏ qua lifecycle.
Khi chưa có map, yêu cầu lưu active trả `1`; thành công trả `0`; lỗi trả `255`.
Mỗi lần chỉ nhận một yêu cầu lưu; yêu cầu đồng thời bị từ chối.

Frontend chờ bất đồng bộ nên lifecycle/heartbeat vẫn hoạt động khi dựng/lưu map
lâu. Timeout chờ mặc định 30 giây (`save_timeout` trên map_saver_server).
Deactivate chặn yêu cầu mới; yêu cầu đã nhận có thể hoàn thành sau đó.
Cleanup/shutdown hủy việc đợi và trả lỗi cho yêu cầu đang chờ; backend có thể
vẫn đang ghi file vì ROS service không có cơ chế cancel. Nếu timeout/lỗi, kiểm
tra log và file trước khi thử lại.

## Khởi chạy

Các launch mapping/navigation mặc định `autostart:=true`: manager tự configure
và activate. Không cần thêm lệnh sau khi khởi động.

Nếu muốn tự điều khiển quá trình khởi động:

```bash
ros2 launch forklift_bringup mapping.launch.py autostart:=false
# Hoặc xe thật:
ros2 launch forklift_bringup mapping_real.launch.py autostart:=false
```

Với `autostart:=false`, SLAM Humble vẫn chạy và dựng map; **chỉ saver** đang
chờ configure/activate. Pause saver không dừng xe, không pause SLAM.

Trong terminal đã source ROS/workspace và đặt cùng ROS_DOMAIN_ID, kiểm tra:

```bash
ros2 lifecycle get /map_saver_server
ros2 lifecycle nodes
```

Khởi động saver qua manager:

```bash
ros2 service call /lifecycle_manager_mapping/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
```

Các command của manager:

| Command | Tác dụng |
|---|---|
| `0` STARTUP | configure + activate từ unconfigured |
| `1` PAUSE | deactivate, ngừng nhận yêu cầu lưu mới |
| `2` RESUME | activate lại từ inactive |
| `3` RESET | deactivate + cleanup, về unconfigured |
| `4` SHUTDOWN | về finalized, cần chạy lại tiến trình để dùng tiếp |

Dùng cùng loại service với đường dẫn `/lifecycle_manager_localization/manage_nodes`
hoặc `/lifecycle_manager_navigation/manage_nodes` để quản lý các nhóm tương ứng.
Ưu tiên điều khiển qua manager thay vì gọi `ros2 lifecycle set` trực tiếp để
trạng thái manager và bond được đồng bộ.

Các lệnh ở đây chỉ quản lý node trong danh sách của từng manager. Sau khi
SHUTDOWN saver, muốn kết thúc cả Gazebo/SLAM/RViz thì Ctrl+C terminal launch.
