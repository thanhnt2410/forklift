# Lệnh kiểm thử

Chạy từ thư mục gốc workspace; source `/opt/ros/humble/setup.bash` và
`install/setup.bash` trước các lệnh kiểm thử. Không chạy các checker vào domain
của phiên robot đang hoạt động.

```bash
colcon build --base-paths src --symlink-install
ctest --test-dir build/forklift_navigation --output-on-failure

PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_cad_model.py tests/test_command_gate.py tests/test_docking.py \
  tests/test_dynamic_mission.py tests/test_imu_adapter.py tests/test_lift_core.py \
  tests/test_mission_route.py tests/test_motion_mux.py tests/test_navigation_guard.py \
  tests/test_p6_geometry.py tests/test_task_manager.py tests/test_tracking_metrics.py -q

PYTHONNOUSERSITE=1 python3 scripts/preview_bidirectional_route.py \
  --domain 190 --output results/path_pipeline_fix/recheck_preview

PYTHONNOUSERSITE=1 python3 scripts/run_p6_checks.py \
  --domain 180 --output results/path_pipeline_fix/recheck_legacy

for scenario in nominal user_drop rotated invalid_pick invalid_drop cancel scan_loss missing_pallet cancel_loaded cancel_retreat; do
  PYTHONNOUSERSITE=1 python3 scripts/run_p6_checks.py --dynamic \
    --scenario "$scenario" --domain 181 \
    --output "results/path_pipeline_fix/recheck_$scenario" || break
done
```

Test source/fixture trong `tests/` và `test/` được giữ local theo `.gitignore`
hiện có của repo. Các script acceptance trong `scripts/`, dữ liệu kết quả và
checksum nguồn được lưu cùng bản sửa. Runner tự cô lập Gazebo partition,
ghi config và cleanup riêng cho từng lượt; không mở cổng phần cứng.
