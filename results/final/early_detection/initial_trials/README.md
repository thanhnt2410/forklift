# Lượt thử đầu

Cả hai lượt đã hoàn tất lấy–giao và rút càng. Phép kiểm tra thử nghiệm
`no_extra_pick_navigation` không đạt vì xác nhận gần phát hiện sai lệch sau khi
AMCL cập nhật trong lúc di chuyển. Lượt đầu dùng vùng docking hẹp hơn; lượt
thứ hai dùng giới hạn 12 cm/6 cm ở thời điểm đó và vẫn phải replan vì arrival_error khoảng
12,2 cm dọc, 6,7 cm ngang, vượt 12 cm/6 cm. Đây là lý do bộ cuối tách ca
định vị trước (`early_direct`) khỏi khởi động mới (`nominal`), vẫn kiểm tra
quyết định replan theo quan sát và giữ phép kiểm tra không REFINE_PICK cho ca direct.

- [Lượt đầu](first/checks.json)
- [Lượt thứ hai](second/checks.json)

Một ca hủy ở B sau đó chưa tới được điểm tiêm lệnh vì NAV_PICK bị kẹt cuối
chặng: sai lệch khoảng 4 cm ngang và 0,15 rad hướng.
[Lượt lỗi](navigation_stall/checks.json) được giữ lại. Bản bàn giao dùng goal
checker riêng cho NAV_PICK động (6 cm/0,10 rad), vẫn xác nhận LiDAR tại A và
kiểm tra footprint trước docking. B, REFINE_PICK và tuyến cũ giữ checker cũ.
