# Bài 7 — Unity publish LaserScan trên /scan

## Trạng thái và phạm vi

Người học yêu cầu tiếp tục sau bài mount; chưa cung cấp log/scene để xác minh độc lập. Đã thêm Assets/PlanarLaserScanPublisher.cs. Đã đối chiếu LaserScanMsg và HeaderMsg trong ROS-TCP Connector hiện có với định nghĩa ROS1. Chưa compile/chạy publisher trong Unity hoặc xác nhận /scan runtime. Không chỉnh scene tự động.

## Khái niệm

/scan là tên topic, sensor_msgs/LaserScan là kiểu message. Một message chứa cả dải khoảng cách kèm góc, giới hạn đo, frame và thời gian.

ROS laser frame: +X phía trước, +Y bên trái, +Z lên trên. Góc dương quay sang trái. Unity sensor: +Z phía trước, +X bên phải, +Y lên trên. Vì thế unityAngleDegrees = -rosAngleRadians * Rad2Deg. Mảng mới chạy phải → giữa → trái, khác mảng bài SingleRayLesson chạy trái → phải.

Với 19 tia, FOV 90 độ: angle_min≈-0.785398, angle_max≈0.785398, angle_increment≈0.0872665. ranges[0] bên phải, ranges[9] chính giữa, ranges[18] bên trái. Góc i = angle_min + i*angle_increment.

stamp lấy Time.timeAsDouble như PlanarOdometryPublisher. Publisher mới không phát thêm /clock. scan_time là khoảng thời gian thực giữa hai lần quét, có thể lớn hơn 0.1 giây vì phụ thuộc frame rate. time_increment=0 vì các tia cùng lấy một trạng thái physics, không mô phỏng scanner quay lần lượt. intensities rỗng vì chưa mô phỏng cường độ phản xạ.

Không hit → +Infinity. Hit dưới range_min → -Infinity (phép đo quá gần, không phải free space). Không dịch điểm phát tia ra range_min vì có thể bỏ qua vật chắn gần. Origin phải nằm ngoài collider môi trường cần đo; robot self-colliders được lọc bằng mask.

## Thực hành [Unity]

Stop Play và dừng controller trước khi đổi scene/runtime. Trên laser_link, tắt Single Ray Lesson, thêm một Planar Laser Scan Publisher. Đặt Ray Count=19, Field Of View=90, Range Min=0.05, Range Max=10, Publish Hz=10. Đây là thông số bài học, chưa phải cấu hình navigation cuối cùng.

Obstacle Mask chọn các layer môi trường, bỏ RobotBody và Ignore Raycast như bài mount. Cấu hình mask không tự sao chép từ component cũ.

Giữ PlanarOdometryPublisher bật, Robot Base=base_link; khởi động bridge theo daily.md. Không chạy node điều khiển tự động trong bài kiểm tra sensor. Đặt /use_sim_time=true trước Play rồi nhấn Play.

## Kiểm tra [Ubuntu WSL]

```bash
docker exec -it ros1_amr_core bash
```

## Kiểm tra [Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosparam set /use_sim_time true
```

Sau khi Unity Play:

```bash
rostopic echo -n 1 /clock
rostopic type /scan
rostopic echo -n 1 /scan
rostopic hz /scan
```

Type cần là sensor_msgs/LaserScan, frame_id=laser_link, có 19 ranges, góc đúng bảng trên và timestamp tăng cùng simulation. Tần suất khoảng 10 Hz khi Unity chạy realtime đủ nhanh; có thể thấp hơn do frame rate. Ctrl+C để dừng hz. Có tên topic chưa chứng minh message đang tới.

Đặt vật bên phải rồi bên trái sensor, kiểm tra vị trí giá trị hữu hạn trong mảng thay đổi đúng phía. Với vật cản trước mặt, không kỳ vọng mọi range bằng nhau vì tia chéo đo khoảng cách khác.

## Đọc code

RegisterPublisher khai báo kiểu dữ liệu topic. LateUpdate giới hạn tốc độ quét. angleMin/angleStep dùng radian; vòng lặp đổi dấu sang góc Unity, raycast và ghi thẳng vào ranges theo thứ tự ROS. Header đóng dấu simulation time và frame laser_link. ros.Publish gửi cả message qua bridge. Mảng mới cho mỗi scan tránh sửa dữ liệu đã đưa vào hàng đợi gửi.

## Giới hạn và bước sau

frame_id chỉ là tên hệ tọa độ, không tạo TF. Bài này nghiệm thu dữ liệu /scan; chưa khẳng định RViz hiển thị scan trong odom. Bài sau kiểm tra TF base_footprint → base_link → laser_link và hiển thị RViz. Không thêm static map → odom để che lỗi TF.

Nếu thiếu /scan: xem Console compile errors, component bật/đúng object, Play và endpoint. Nếu toàn inf: kiểm tra mask, collider, hướng và độ cao vật cản. Nếu có số rất nhỏ hoặc -inf: kiểm tra self-hit và khoảng cách dưới min. Nếu chỉ đảo trái/phải: kiểm tra hướng lắp link, không đảo mảng tùy ý.

Nguồn định nghĩa: https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/LaserScan.msg
