# Bài 6 — Gắn cảm biến lên robot và loại self-hit

## Trạng thái

Người học phản hồi đã xong bài dải quét và yêu cầu tiếp tục; chưa gửi log cụ thể. Đã đọc prefab reverse_stacker_amr: laser_link là con của base_link (Transform fileID 6366581916795576229), local position x=0, y≈0.225, z=0.25; local rotation identity. SampleScene và 1pallet đều tham chiếu prefab này. Chưa xác nhận scene đang mở hoặc các override runtime.

Bài này dùng lại SingleRayLesson.cs, không sửa code hoặc scene tự động. Chưa có kết quả thực hành gắn sensor.

## Khái niệm

Sensor mount là vị trí và hướng lắp cảm biến tương đối với thân robot. Parent-child trong Unity làm cảm biến đi theo robot. Nó chưa tự tạo TF bên ROS. Self-hit là tia đo trúng collider của chính robot.

## Thực hành [Unity]

1. Stop Play, lưu scene bàn thử rồi mở scene đã dùng chạy A → B. Dừng go_to_point/move_base nếu đang chạy. Bài này chỉ dùng keyboard để di chuyển; tắt các component tự di chuyển như StackerPointToPoint/RosTestBridge nếu có.
2. Mở Hierarchy robot, tìm base_link → laser_link. Kiểm tra local Position gần (0,0.225,0.25), Rotation (0,0,0). Không reset Transform của link robot nếu khác; kiểm tra override trước.
3. Trên laser_link, Add Component → Single Ray Lesson, chỉ một bản. Đặt Ray Count=19, Field Of View=90, Max Distance=10. Không tạo thêm RayOrigin trong scene này.
4. Layer → Add Layer, đặt một User Layer còn trống tên RobotBody. Chọn object gốc của riêng robot, gán RobotBody và chọn Yes, change children. Kiểm tra các object có collider của thân/bánh/càng/sensor cũng thuộc RobotBody. Không gán môi trường hoặc hàng độc lập vào layer này.
5. Trên Single Ray Lesson, đặt Obstacle Mask=Everything rồi bỏ RobotBody và Ignore Raycast. Giữ các layer vật cản được chọn. Không tắt collider hoặc thay Physics collision matrix: mask này chỉ lọc phép đo raycast.
6. Đặt Cube TestObstacle có Box Collider bật (không trigger), Layer Default trước mặt cảm biến khoảng 2–3 m. Hộp phải cắt mặt phẳng quét ngang ở độ cao cảm biến. Để hộp ở gốc Hierarchy, không làm con của robot.
7. Play, xem Scene view/Gizmos. Xác nhận cụm tia phát tại laser_link và tia giữa theo hướng +Z của base_link. Bật Log Each Ray trong thời gian ngắn để kiểm tra tên collider trúng tia.
8. Dùng keyboard như bài điều khiển đã chạy thành công, tiến chậm một đoạn rồi quay nhẹ trong vùng trống. Chỉ một nguồn điều khiển hoạt động. Tia phải di chuyển/quay theo xe; hộp đứng yên. Khi tiến thẳng về cùng mặt hộp, khoảng cách tia giữa giảm gần bằng quãng đường tiến. Không yêu cầu khoảng cách chỉ giảm khi đang quay vì tia có thể đổi điểm chạm hoặc mất hit.

Nếu không thấy laser_link trong scene hoặc vị trí/hướng khác prefab đã đọc, cần xem hierarchy/override thực tế trước khi sửa. Nếu tia trúng chính robot, xem Layer của object chứa collider được log (có thể là object con tên collision). Không tắt collider để giải quyết self-hit.

## Liên hệ code

transform.position, transform.forward và transform.up thuộc object gắn component, nay là laser_link. obstacleMask truyền vào Physics.Raycast quyết định những layer có thể được đo. Các collider bị bỏ qua vẫn tồn tại để xử lý va chạm vật lý.

## Điều kiện hoàn thành

- Tia đi theo robot và phát từ đúng sensor.
- Tia đo vật cản ngoài robot, không trúng collider RobotBody.
- Khoảng cách thay đổi hợp lý khi robot di chuyển.

Chưa publish /scan hay tạo TF laser_link bên ROS trong bài này. Bài sau xử lý góc Unity/ROS, radian, timestamp và LaserScan sau khi xác nhận mount.

Nguồn: https://docs.unity3d.com/2022.3/Documentation/Manual/use-layers.html
