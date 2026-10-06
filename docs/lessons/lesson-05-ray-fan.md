# Bài 5 — Góc quét và độ phân giải góc

## Mục tiêu

Thấy vật cản có thể nằm giữa các tia, hiểu Field of View (FOV, góc quét bao phủ) và angular resolution (độ phân giải góc, bước góc giữa hai tia). Tạo mảng ranges để chuẩn bị học LaserScan, chưa gửi ROS.

## Code và trạng thái

Đã sửa Assets/SingleRayLesson.cs, giữ nguyên class để dùng lại component. Thêm rayCount (mặc định 19), fieldOfView (90 độ), logEachRay và ranges. Script giới hạn số tia 2–361 và góc quét 1–270 độ, tính cả hai tia biên. Kết quả runtime của bài này chưa được kiểm chứng trong Unity.

Với N tia và FOV=90 độ:

```text
angleMin = -FOV / 2
angleStep = FOV / (N - 1)
angle[i] = angleMin + i * angleStep
```

N tia có N-1 khoảng giữa chúng. N=3 cho bước 45 độ, N=19 cho bước 5 độ. Góc trong bài là Unity yaw tính bằng độ, trái âm/phải dương; chưa phải thứ tự/góc của ROS LaserScan.

Mỗi Update thực hiện một lần quét, lưu khoảng cách ở ranges[i]. Không hit được lưu float.PositiveInfinity; không được hiểu là toàn bộ hướng đó trống vô hạn. Chỉ biết không gặp collider hợp lệ trong maxDistance. Việc vẽ tia no-hit vẫn giới hạn ở maxDistance. Log tóm tắt mỗi giây simulation; đây không phải tần suất quét 1 Hz. Log Each Ray bật thêm thông tin từng tia.

## Thực hành [Unity]

1. Stop Play, chờ compile. Giữ RayOrigin Position=(0,1,0), Rotation=(0,0,0), Scale=(1,1,1), không có parent. Giữ một component Single Ray Lesson, Max Distance=10, mask mặc định.
2. Tắt ba hộp của bài cũ. Tạo Cube SmallObstacle ở Position=(1,1,3), Rotation=(0,0,0), Scale=(0.2,1,0.2). Box Collider bật, Is Trigger tắt, Layer Default. Không có collider khác chắn tia.
3. Đặt Ray Count=3, Field Of View=90. Play, xem Scene view với Gizmos bật. Dự kiến Hits=0: tia 0 độ đi x=0, tia +45 độ đi x=z; cả hai đều bỏ qua hộp.
4. Khi vẫn Play, đổi Ray Count=19. Dự kiến Hits=1. Tia +20 độ chạm hộp; bật Log Each Ray để xem Ray 13 khoảng 3.086 m. Khoảng cách tính tới mặt z=2.9: 2.9/cos(20 độ). Sai số số thực nhỏ là bình thường.
5. Mở Ranges trong Inspector: có 19 phần tử; index 13 chứa phép đo khoảng 3.086, các phần tử khác là Infinity nếu không có collider khác.

Góc nhìn tới tâm hộp khoảng atan(1/3)=18.4 độ. Với kích thước hộp này, tia 20 độ chạm collider, tia 15 độ không chạm. Góc 18.4 chỉ dùng giải thích, không phải góc được lấy mẫu.

## Đọc code

`angleMin + i * angleStep` thay cho mảng ba góc viết sẵn. `Quaternion.AngleAxis(angle, transform.up) * transform.forward` giữ nguyên ý nghĩa từ bài trước. Mảng ranges cấp phát lại khi số tia thay đổi, rồi được cập nhật mỗi frame. Đây là dữ liệu output; sửa phần tử thủ công trong Inspector sẽ bị ghi đè.

Nhiều tia giảm khoảng trống góc nhưng tăng số phép Raycast. Không khẳng định rằng tăng số tia sẽ phát hiện mọi vật cản. Gắn sensor lên robot, xử lý self-hit, timestamp, frame và quy ước góc ROS là các bước sau.

## Kết quả

Chưa chạy Unity. Cần người học xác nhận Hits khi N=3 và N=19, cùng log tia 20 độ. Thay đổi khi Play không lưu lại sau Stop; đặt lại giá trị mong muốn ngoài Play nếu muốn lưu scene.
