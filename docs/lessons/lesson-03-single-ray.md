# Bài 3 — Một tia đo khoảng cách

Ngày: 2026-09-24.

## Trạng thái

- Người học xác nhận A → B trong vùng không có vật cản đã chạy thành công. Đây là xác nhận của người học; chưa bổ sung log hoặc số đo cuối.
- Đã thêm `amr_ware_house/Assets/SingleRayLesson.cs` phục vụ bài học.
- Người học xác nhận phép đo một tia cho khoảng cách 2,5 m và 4,5 m như dự kiến; đã giải thích đúng khoảng cách tới mặt hộp.
- Người học xác nhận thử xoay cảm biến Y=90° thành công.
- Script hiện đã được mở rộng thành ba tia trong [bài tiếp theo](lesson-04-three-rays.md); các hướng dẫn và log một tia dưới đây mô tả phiên bản trước.

## Mục tiêu và khái niệm

Hiểu một phép đo trước khi tạo nhiều tia cho LiDAR. Raycast là phép kiểm tra một tia xuất phát từ một điểm, theo một hướng, giao với collider nào trong giới hạn khoảng cách. Collider là hình học mà physics dùng để kiểm tra va chạm; hình ảnh mesh đơn thuần chưa đủ.

LiDAR thật dùng ánh sáng để đo khoảng cách. Bài này dùng raycast để mô phỏng một phép đo hình học lý tưởng, chưa mô phỏng nhiễu hay đặc tính phản xạ. Quy ước bài: 1 Unity unit = 1 m.

## Thực hành [Unity]

1. Dừng controller ROS nếu đang chạy, rồi Stop Play. Lưu scene robot hiện tại.
2. Tạo scene mới bằng File → New Scene, chọn Basic (Built-in) hoặc scene trống. Đây là bàn thử cảm biến, không cần robot hoặc ROS. Có thể lưu scene tại `Assets/Scenes/SingleRayLesson.unity`.
3. Create Empty, đặt tên `RayOrigin`. Đặt Transform Position `(0, 1, 0)`, Rotation `(0, 0, 0)`, Scale `(1, 1, 1)`. Giữ object ở gốc Hierarchy, không có parent.
4. Add Component → Single Ray Lesson. Giữ Max Distance = 10, Obstacle Mask = mặc định.
5. Tạo 3D Object → Cube, đặt tên `TestObstacle`. Position `(0, 1, 3)`, Rotation `(0, 0, 0)`, Scale `(1, 1, 1)`. Giữ Box Collider bật, Is Trigger tắt và Layer = Default.
6. Play. Mở Scene view, bật Gizmos; chọn RayOrigin và nhấn F nếu cần tìm vị trí. Xem tia và Console.

Kết quả mong đợi: tia xanh lá và `[SingleRay] Hit TestObstacle: 2.500 m`. Tâm hộp nằm tại z=3 nhưng mặt gần cảm biến nằm tại z=2.5. Khoảng cách đo tới mặt collider, không phải tâm object.

Trong Play, sửa Position của TestObstacle để thử:

| Position | Kết quả mong đợi |
|---|---|
| (0, 1, 3) | 2.500 m |
| (0, 1, 5) | 4.500 m |
| (2, 1, 3) | No hit within 10.000 m nếu không có collider khác trên tia |

Các thay đổi Transform trong Play thường trở về trạng thái trước Play khi dừng. Ghi kết quả trước khi Stop.

## Đọc code

- `transform.position`: điểm bắt đầu trong world coordinates (tọa độ thế giới).
- `transform.forward`: hướng trục Z dương của object, biểu diễn trong world coordinates. Với Rotation bằng 0, tia đi theo world +Z.
- `Physics.Raycast(...)`: trả về true khi gặp collider phù hợp; `out RaycastHit hit` nhận chi tiết lần chạm.
- `maxDistance`: giới hạn phép đo; 10 m chỉ là thông số bàn thử.
- `obstacleMask`: chọn những layer được phép đo; khi gắn robot cần cấu hình để bỏ qua collider của chính robot.
- `QueryTriggerInteraction.Ignore`: bỏ qua trigger collider.
- `hit.distance`: khoảng cách từ điểm phát tia tới điểm chạm.
- `Debug.DrawRay`: vẽ tia phục vụ quan sát; phép đo do Physics.Raycast thực hiện.
- `nextLogTime`: hạn chế log một lần mỗi giây simulation, trong khi phép đo vẫn cập nhật mỗi frame.

Không có hit không có nghĩa khoảng cách bằng 0; nghĩa là không tìm thấy collider phù hợp trong tầm đo theo hướng này.

## Nếu kết quả khác dự kiến

- Không có log: kiểm tra Console có lỗi compile không, component có bật không, Unity có đang Play không và bộ lọc Log có bật không.
- Có log nhưng không thấy tia: mở Scene view và bật Gizmos.
- No hit: kiểm tra Rotation, Box Collider, Layer/Obstacle Mask và vị trí hộp.
- Hit object khác: đọc tên trong log để tìm collider đang chắn tia.
- Không đặt điểm phát bên trong collider cần đo: raycast không phát hiện collider chứa điểm phát.

## Liên hệ robot

URDF hiện có `laser_link`, nối với `base_link` bằng `laser_joint` tại ROS xyz=(0.25, 0, 0.225). Đây là mô tả mount, chưa chứng minh có cảm biến hoạt động trong scene. Sau khi bài đo độc lập đạt yêu cầu, kiểm tra vị trí và hướng thực tế của laser_link trong scene đang dùng, gắn probe, loại collider robot khỏi phép đo, rồi mới mở rộng thành nhiều tia và LaserScan.

## Kết quả thực đo

Theo xác nhận của người học: đo được 2,5 m và 4,5 m; thử xoay cảm biến Y=90° đúng dự kiến. Chưa có log lưu riêng.

Nguồn API: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/Physics.Raycast.html
