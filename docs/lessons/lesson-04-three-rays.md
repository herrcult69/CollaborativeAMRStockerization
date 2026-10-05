# Bài 4 — Ba tia đo khoảng cách

## Thay đổi và trạng thái

Cập nhật: người học phản hồi “oke” và yêu cầu bài tiếp, chưa gửi số đo/log ba tia cụ thể. Script hiện được mở rộng trong [bài 5](lesson-05-ray-fan.md); đặt Ray Count=3, Field Of View=90 và bật Log Each Ray để tái hiện hình học bài này (nhãn log đã đổi).

Đã sửa `amr_ware_house/Assets/SingleRayLesson.cs` từ một tia thành ba tia: trái -45°, giữa 0°, phải +45° theo quy ước quay quanh trục Y của Unity. Giữ nguyên tên class/file để component trong scene tiếp tục tham chiếu script cũ. Chưa chạy phiên bản ba tia trong Unity; kết quả dưới đây là dự kiến.

## Thực hành [Unity]

Stop Play, chờ Unity biên dịch. Giữ component Single Ray Lesson đã gắn, Max Distance=10 và Obstacle Mask mặc định. Không thêm component thứ hai.

Đặt RayOrigin ở Position (0,1,0), Rotation (0,0,0), Scale (1,1,1), không có parent. Dùng ba Cube ở gốc Hierarchy với Rotation (0,0,0), Scale (1,1,1), Box Collider bật, Is Trigger tắt, Layer Default:

| Tên | Position | Tia | Khoảng cách dự kiến |
|---|---|---|---|
| LeftObstacle | (-3,1,3) | Left -45° | khoảng 3,536 m |
| CenterObstacle | (0,1,3) | Center 0° | 2,500 m |
| RightObstacle | (3,1,3) | Right +45° | khoảng 3,536 m |

Có thể đổi tên và dùng lại hộp cũ làm CenterObstacle, rồi nhân đôi hai lần. Không để thêm collider chắn tia. Play, mở Scene view và bật Gizmos. Console in ba kết quả mỗi giây simulation.

Hai tia chéo chạm góc gần của hộp ở x=±2,5 và z=2,5. Chiều dài tia là sqrt(2,5² + 2,5²) ≈ 3,536 m, không chỉ là khoảng cách theo Z. Chấp nhận sai số nhỏ do số thực tại góc collider.

Tắt riêng GameObject LeftObstacle trong Play: chỉ Left chuyển sang No hit nếu không có vật khác trên tia. Center và Right vẫn đo bình thường.

## Đọc code

`angles` là mảng ba góc; `rayNames` là nhãn tương ứng. Vòng `for` chạy lần lượt ba phép đo.

```csharp
Vector3 direction = Quaternion.AngleAxis(angles[i], transform.up)
    * transform.forward;
```

`Quaternion.AngleAxis` tạo phép quay với góc tính bằng độ quanh một trục. `transform.up` là trục Y của cảm biến biểu diễn trong world coordinates, còn `transform.forward` là hướng nhìn của cảm biến. Nhân phép quay với vector cho hướng tia mới; không xoay GameObject. Vì dùng các trục của cảm biến, cả cụm tia đi theo hướng cảm biến khi cảm biến quay.

Phép Raycast và cách lấy hit.distance giữ nguyên bài trước. `shouldLog` được tính trước vòng lặp để cả ba tia cùng được log mỗi giây; phép đo và vẽ tia vẫn chạy mỗi frame.

Đây là góc Unity tính bằng độ, chưa phải quy ước góc ROS LaserScan. Ba tia còn có khoảng trống lớn: vật thể giữa hai tia có thể bị bỏ sót. Bài sau tăng số tia và kiểm tra góc quét trước khi đóng gói LaserScan.

## Kết quả thực đo

Chờ người học chạy và gửi ba dòng log. Chưa xác nhận runtime của bản ba tia.

Nguồn: https://docs.unity3d.com/2022.3/Documentation/ScriptReference/Quaternion.AngleAxis.html
