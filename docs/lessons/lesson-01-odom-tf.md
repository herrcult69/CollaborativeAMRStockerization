# Bài 1 — Quan sát /odom và TF khi xe di chuyển

## Mục tiêu

Keyboard control đã được người dùng xác nhận hoạt động. Bài này thêm ideal simulated odometry: lấy pose thực tế của robot trong Unity, gửi `/odom` và TF `odom → base_footprint` từ cùng pose và timestamp. Script không điều khiển bánh xe.

Goal cuối milestone 1 vẫn là xe tự đi từ A đến B rồi dừng; bài này tạo feedback về vị trí để dùng cho bước điều khiển đó.

## Gắn component trong Unity

1. Stop Play trước khi sửa để cấu hình không mất khi thoát Play.
2. Chọn GameObject chứa `StackerController`.
3. Add Component → `PlanarOdometryPublisher`.
4. Mở hierarchy của robot, kéo object **`base_link`** vào trường **Robot Base**. Trong prefab hiện tại, `base_link` có ArticulationBody và di chuyển; object Unity tên `base_footprint` chỉ là wrapper không có ArticulationBody.
5. Giữ Publish Hz = 20. Chỉ gắn một publisher này cho robot.
6. Save scene. Giữ endpoint đang chạy và nhấn Play.

Script giả định trục +Z local của object được theo dõi là hướng tiến của xe trong Unity. Nếu lái thẳng mà `/odom` tăng chủ yếu theo Y hoặc X âm, cần kiểm tra hướng frame/chiều chạy trước khi tiếp tục; không sửa dấu ngẫu nhiên.

ROS frame `base_footprint` được tính bằng pose phẳng của `base_link` (bỏ độ cao, roll và pitch), không đọc từ wrapper cùng tên trong Unity. Trong model này hai gốc chỉ lệch theo chiều cao 0,05 m, nên dùng X/Y và yaw của thân xe phù hợp với bài mô phỏng phẳng. Hướng dẫn ban đầu chọn wrapper `base_footprint` đã được sửa sau khi kiểm tra prefab.

## Clock của bài này

Script phát simulation time của Unity trên `/clock`, đồng thời dùng cùng timestamp cho `/odom` và TF. Chỉ một component được publish `/clock`. Đây là cấu hình thay thế wall time sau khi đo thấy timestamp Windows/Unity lệch so với ROS trong Docker.

**[Inside ROS Docker container]**

```bash
rosparam set /use_sim_time true
```

Đặt parameter trước khi khởi động controller và RViz. Restart RViz nếu nó đã chạy với clock cũ. Giữ Unity Play để clock tiến; tắt controller trước khi Stop/Play lại vì simulation time sẽ reset. Endpoint vẫn có thể giữ chạy. `rostopic echo -n 1 /clock` phải trả về thời gian tính từ đầu lượt Play.

## Mở terminal kiểm tra

Giữ terminal chạy Unity endpoint. Mở terminal khác.

**[Ubuntu WSL]**

```bash
docker exec -it ros1_amr_core bash
```

Lệnh mở một shell khác trong cùng container, không tạo container mới.

**[Inside ROS Docker container]**

```bash
source /opt/ros/noetic/setup.bash
rostopic hz /odom
```

Lệnh đo tần suất nhận message; mong đợi khoảng 20 Hz (có thể thấp hơn do FPS). Ctrl+C chỉ dừng việc xem tần suất.

```bash
rostopic echo /odom/pose/pose/position
```

Hiển thị position. Trong Unity, nhấn W chạy thẳng từ vị trí ban đầu: X nên tăng; Y gần 0. Ctrl+C trước khi chuyển phép kiểm tra.

```bash
rosrun tf tf_echo odom base_footprint
```

Hiển thị transform giữa hai frame. Translation phải khớp position trong `/odom`; khi quay, yaw thay đổi. Quay trái theo quy ước ROS làm yaw tăng, có thể wrap tại ±180°. Ctrl+C để dừng xem.

```bash
rostopic echo /odom/twist/twist
```

Hiển thị vận tốc trong frame của xe: tiến có `linear.x` dương, quay trái có `angular.z` dương. Khi xe dừng, các giá trị gần 0.

## Xem trong RViz

**[Inside ROS Docker container]**

```bash
rviz
```

Lệnh mở cửa sổ RViz qua cấu hình GUI hiện có.

**[RViz]**

1. Global Options → Fixed Frame: `odom`.
2. Add → TF: xem frame của robot di chuyển so với `odom`.
3. Add → Odometry → Topic: `/odom`: xem pose bằng mũi tên. Nếu có nhiều mũi tên, đó là lịch sử pose; có thể đặt Keep = 1 để chỉ giữ pose mới nhất.
4. Add → Grid nếu chưa có.

Bài này chưa cần RobotModel: chỉ frame và mũi tên là đủ kiểm tra dữ liệu. Chưa cần chạy `move_base` hoặc một node khác publish `odom → base_footprint`.

## Thí nghiệm để hiểu dữ liệu

| Thao tác trong Unity | /odom | TF |
|---|---|---|
| Bắt đầu Play | Position gần (0,0,0), orientation ban đầu | base_footprint gần gốc odom |
| Tiến thẳng từ A | X tăng | Frame xe rời gốc theo X |
| Quay tại chỗ | Orientation đổi; position gần giữ nguyên | Trục của frame xe xoay |
| Tiến tiếp sau khi quay | X/Y đổi theo hướng mới | Frame xe tiến theo hướng mới |
| Thả phím, xe dừng | Twist gần 0; pose giữ nguyên | Transform giữ pose, vẫn được publish |

`odom` được đặt tại vị trí/hướng bắt đầu Play. Dừng rồi Play lại sẽ tạo gốc mới; không dùng dữ liệu của hai lượt Play như một hành trình liên tục.

## Đọc code theo thứ tự

- `Start`: đăng ký hai publisher và chuẩn bị clock.
- Lần `LateUpdate` đầu: ghi nhớ vị trí và hướng ban đầu làm mốc `odom`.
- `relative`: tính vị trí so với mốc ban đầu.
- `heading`: tính yaw tương đối, đổi dấu theo quy ước Unity/ROS.
- `bodyVelocity`: tính vận tốc từ hai mẫu position và biểu diễn trong frame xe.
- `odom`: đóng gói pose và twist.
- `transformMessage`: đóng gói cùng pose thành TF.
- Hai lệnh `Publish`: gửi hai loại message độc lập.

Đây là mô hình phẳng (bỏ qua Z, roll, pitch), không phải wheel odometry. Covariance bằng 0 chỉ dành cho ground-truth lý tưởng của bài này. Chưa dùng nó như mô hình sai số sensor thật.

## Khi có lỗi

- Không thấy `/odom`: kiểm tra component đã bật, Robot Base đã gán, Unity đang Play, endpoint kết nối và Console không có lỗi compile.
- Có topic nhưng không có dữ liệu: dùng `rostopic hz /odom` và xem Unity Console/endpoint logs.
- Xe chạy nhưng position luôn 0: kiểm tra có chọn nhầm prefab wrapper đứng yên không.
- RViz báo không có `map`: đổi Fixed Frame thành `odom`.
- TF có pose giật: kiểm tra có publisher khác cùng gửi transform không.
- TF báo thời gian quá cũ/tương lai: kiểm tra clock Windows/container và `/use_sim_time`.

Hoàn thành bài khi position, hướng quay và vận tốc khớp chuyển động thực tế. Sau đó mới triển khai controller dùng `/odom` để tiến đến B.
