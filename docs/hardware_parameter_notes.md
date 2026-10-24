# Bổ sung thông số phần cứng — 24/10/2026

Người dùng xác nhận encoder 2 kênh A/B, tỉ số truyền ghi 21,3; cung cấp các số
280 / 220 RPM, torque 0,5 và gearbox length 19. Chưa có ảnh/link bảng gốc hoặc
đơn vị torque/điện áp. Không có nút E-stop và mạch ngắt driver.
Dòng NEMA17 và lead vít me chưa biết. Người dùng cho phép chọn vi bước.
Cập nhật: user cung cấp **11 PPR**, chọn mặc định hai kênh A/B; cấu hình giải mã x4.

## Có thể dùng phần nào?

[Bảng của nhà sản xuất XYT](https://www.xytmotors.com/encoder/153.html) có các dòng
JGA25-370 21,3:1 với 280 RPM không tải, 220 RPM tải định mức, torque định mức
0,50 kgf·cm và hộp số dài 19 mm. Nhiều bản điện áp 6/12/24 V có cùng các số này:
**không nhận dạng được điện áp hay đúng biến thể motor chỉ từ dãy số đã cho**.
Đây là đối chiếu để hiểu cột, chưa xác minh bảng đó thuộc động cơ của người dùng.

| Số đã cung cấp | Diễn giải có điều kiện |
|---|---|
| 21,3 | Tỉ số giảm tốc danh nghĩa; motor quay khoảng 21,3 vòng cho 1 vòng trục ra |
| 280 RPM | Có khả năng là tốc độ trục ra không tải ở điện áp ghi trong bảng |
| 220 RPM | Có khả năng là tốc độ trục ra tại tải định mức |
| Torque 0,5 | Nếu bảng ghi kgf·cm thì tương đương khoảng 0,049 N·m; chưa gán đơn vị cho dữ liệu user |
| Gearbox length 19 | Có khả năng chiều dài hộp số 19 mm; không phải số xung encoder |

Không dùng tốc độ này để tự tăng giới hạn firmware hoặc suy ra PID. Torque bánh
không phải tải trọng nâng cho phép của NEMA17/vít me.

## CPR danh nghĩa và kiểm chứng

Hai kênh A/B xác định kiểu phản hồi và cho phép đếm chiều. User đã cung cấp PPR = 11.
Với cách hiểu encoder ở trục motor và PPR là số chu kỳ trên một kênh/vòng motor:

```text
counts_x4_per_output_rev = 4 × PPR_motor × reduction_ratio
                        = 4 × 11 × 21.3 = 937.2
```

Nếu tài liệu đã ghi số đếm x4, không nhân 4 lần nữa; nếu encoder ở trục ra,
không nhân tỉ số truyền. Phải làm rõ quy ước của bảng trước khi sử dụng.
Giá trị 11 do user cung cấp, không suy từ tên GA25-370. Firmware dùng 937,2
cho bốn bánh với giả định bánh nối trực tiếp trục ra. Đây là số đếm danh nghĩa
trung bình mỗi vòng; phần lẻ xuất phát từ tỉ số truyền làm tròn, không phải một
xung có thể bị chia nhỏ. Giữ kiểu số thực và thay bằng hiệu chuẩn khi có số đo.

Cách hiệu chuẩn độc lập: sau khi xác minh nguồn/mức logic encoder, chưa cấp điện
cho motor, đếm quadrature x4 khi quay trục ra cùng chiều nhiều vòng có đánh dấu
(ví dụ 10 vòng), lấy tổng số đếm chia số vòng. Kiểm tra chiều âm và từng động cơ.
Tỉ số truyền ghi 21,3 có thể làm tròn, nên nên dùng số đếm đo được cho odometry.

## Nâng và dừng

- Chọn **vi bước dự kiến 1/8**; đây chưa phải DIP đã đặt. Firmware vẫn để
  step/m = 0 cho đến khi biết góc bước motor, lead vít me và vi bước thực.
- Đo lead bằng quãng nâng trên số vòng vít me, ví dụ quãng dịch chuyển sau 10 vòng
  chia 10. Không suy lead từ đường kính ngoài của vít.
- NEMA17 chỉ mô tả kích thước khung; cần mã/nhãn/tài liệu motor để đặt dòng TB6600.
  Không chọn dòng chỉ theo tên NEMA17 hoặc chọn đại từ khả năng driver.
- E-stop và mạch ngắt chưa có: vẫn giữ khóa đầu ra và chưa chuyển sang chạy xe
  thật. User chọn R_EN/L_EN kéo HIGH ngoài và ngắt nguồn thủ công cho bốn bánh:
  khi lỗi/mất Pi giữ PWM cuối, không tự dừng bánh. Lệnh dừng chủ động vẫn có hiệu lực.
  Nâng giữ homing/giới hạn và dừng STEP khi lỗi; ngắt dòng nâng không mặc nhiên giữ được tải.

Đã cập nhật CPR danh nghĩa trong board_config.h; vẫn khóa đầu ra do PID, wiring,
mức điện và cấu hình nâng chưa xác minh. Không nạp board hoặc chạy thử tránh vật cản.
