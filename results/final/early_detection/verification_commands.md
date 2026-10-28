# Chạy lại kiểm thử

Từ thư mục workspace, sau khi source ROS Humble và `install/setup.bash`:

```bash
colcon build --base-paths src --packages-select forklift_navigation forklift_task_manager forklift_bringup --symlink-install
ctest --test-dir build/forklift_navigation --output-on-failure

PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_cad_model.py tests/test_command_gate.py tests/test_docking.py \
  tests/test_dynamic_mission.py tests/test_imu_adapter.py tests/test_lift_core.py \
  tests/test_mission_route.py tests/test_motion_mux.py tests/test_navigation_guard.py \
  tests/test_p6_geometry.py tests/test_task_manager.py tests/test_tracking_metrics.py -q

for scenario in early_direct early_fallback moved_pallet nominal user_drop rotated invalid_pick invalid_drop cancel_initial scan_loss_initial cancel scan_loss missing_pallet cancel_loaded cancel_retreat; do
  PYTHONNOUSERSITE=1 python3 scripts/run_p6_checks.py --dynamic \
    --scenario "$scenario" --domain 201 \
    --output "results/early_detection/recheck_$scenario" || break
done

PYTHONNOUSERSITE=1 python3 scripts/run_p6_checks.py \
  --domain 201 --output results/early_detection/recheck_legacy
```

Chọn domain không trùng phiên robot đang chạy. Runner cleanup riêng từng
Gazebo partition/domain; không mở phần cứng. Fixture/unit test trong `tests/`
và `test/` giữ local theo `.gitignore` có sẵn của người dùng; các checker
acceptance trong `scripts/` và dữ liệu kết quả nằm trong bản sửa.

Thiết lập cảnh dùng trong checker:

- `early_direct`: 8 lần request scan update AMCL khi đứng yên, rồi Run.
- `early_fallback`: đưa pallet ra ngoài vùng tìm trước Run; đưa về khi bước đầu
  báo UNAVAILABLE, trước khi xe tiếp cận. Detector vẫn chỉ dùng scan thật.
- `moved_pallet`: dịch pallet từ y=0,65 sang y=0,55 sau nhận dạng sớm, tại NAV_PICK.
- `cancel_initial`/`scan_loss_initial`: hủy/drop scan ngay ở DETECT_PICK_INITIAL;
  phải dừng trước lập path hoặc chạy bánh, không chuyển thành fallback.
