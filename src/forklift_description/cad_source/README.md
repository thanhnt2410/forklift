# Nguồn CAD tham khảo

Bản xuất SolidWorks URDF20 gốc được giữ nguyên nội dung trong `forklift.original.urdf`,
`forklift.csv`, `meshes/*.STL` và `export.log`. Các tên/URI URDF20 trong những file
nguồn là dữ liệu lịch sử; runtime không đọc các URI này trực tiếp.

Tái tạo mô hình dùng `python3 -s scripts/cad/import_forklift_cad.py` từ gốc workspace.
Script xuất mesh tên chuẩn sang `../meshes` và Xacro sang `../urdf`.
`legacy_sim` giữ các macro hình học đơn giản cũ, không được include khi chạy.
Bản sao package trước khi gộp: `results/model_layout/urdf20_before_merge.tar.gz`.
Metadata package nguồn khai báo giấy phép BSD; xem bản sao để đối chiếu nguồn gốc.
