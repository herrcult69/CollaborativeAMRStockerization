# Bài 9 — Footprint, clearance và vùng quét khi quay

## Mục tiêu

Phân biệt việc phát hiện vật cản với việc xác định toàn bộ robot có đi lọt qua khoảng trống hay không. Bài khái niệm và quan sát, chưa cấu hình planner/costmap.

Footprint là đa giác bao phần robot có thể va chạm khi chiếu xuống mặt phẳng di chuyển. Frame base_footprint là hệ tọa độ; nó không tự chứa đa giác footprint. Trong workflow hiện tại phải cấu hình hình học footprint cho costmap riêng.

## Dữ liệu đã đọc

Theo stacker_amr.urdf, thân chính dài 0.60 m, rộng 0.40 m; đây chưa phải kích thước toàn xe. Càng hướng ROS -X, đầu càng ở x=-0.30-0.025-0.025-0.50=-0.85 m so với base_link. Mép trước thân ở x=+0.30 m. Chiều dài bao thân và càng theo mô hình là 1.15 m, gốc frame không nằm giữa hình bao.

Chưa chốt chiều rộng toàn xe: phải tính bánh và collider thực tế. URDF hiện đặt rotation rpy=pi/2,0,0 cho visual bánh nhưng không đặt rotation tương ứng trong collision của bánh; không suy ra hình học collider Unity chỉ từ visual hoặc thông số trackWidth. Cần đối chiếu collider scene trước khi chốt footprint navigation. Khi có hàng nhô ngoài thân/càng, phải tính cả hàng.

## Bài toán minh họa

Giả sử (không phải số đo xe hiện tại) toàn robot rộng 0.60 m, đi thẳng chính giữa lối rộng 0.80 m thì clearance mỗi bên=(0.80-0.60)/2=0.10 m. Clearance đo từ biên robot tới vật cản, không đo từ gốc frame.

Đi lọt khi tiến thẳng không chứng minh quay được. Khi quay, càng dài quét qua vùng rộng quanh tâm quay; cần kiểm tra footprint ở nhiều pose trên quỹ đạo. Robot dài không thể được coi là một điểm hoặc chỉ dùng bề rộng thân để quyết định đường đi.

Inflation trong costmap tạo vùng chi phí quanh vật cản để ảnh hưởng lựa chọn đường đi. Nó không thay thế footprint và không đồng nghĩa toàn bộ inflation_radius là khoảng cấm hoặc clearance bắt buộc.

## Quan sát [Unity]

Ngoài Play, mở scene robot và Scene view nhìn từ trên. Chọn riêng các object có Collider và bật Gizmos/Edit Collider nếu cần. Quan sát thân, bánh, mast, càng và hàng đang mang. Không sửa collider trong bước này.

Hình dung đa giác bao quanh toàn bộ các phần có thể va chạm. Tìm bốn biên trước/sau/trái/phải so với base_link; chú ý đầu càng nhô về -X ROS (thường -Z local Unity). Giữ một cấu hình lift/hàng rõ ràng cho lần đo; chưa gọi kích thước URDF là kết quả đo scene.

Tự trả lời: nếu đầu xe vừa đi qua góc kệ, quay ngay có làm đầu càng phía sau quét trúng kệ không? Đây là lý do cần xét hình dạng và hướng robot trên đường đi.

## Trạng thái

Đã đọc URDF và ghi phép tính hình học. Chưa đo footprint scene, chưa tạo YAML kích thước để chạy navigation. Bài sau cần chốt hình học thực tế hoặc tiếp tục học map/localization; không coi script sensor hay TF đã tự cung cấp footprint cho costmap.

Nguồn inflation ROS1: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/costmap_2d/plugins/inflation_layer.cpp
