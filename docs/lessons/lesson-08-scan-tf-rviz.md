# Bài 8 — TF cảm biến và LaserScan trong RViz

## Trạng thái

Người học xác nhận đã thực hiện /scan, không yêu cầu đối chiếu log. Đã tạo amr_navigation/launch/laser_tf.launch dựa trên stacker_amr.urdf; chưa chạy launch/RViz trong phiên này. Không chỉnh scene hoặc runtime.

## Khái niệm

Scan chứa điểm trong frame laser_link. Để vẽ trong odom, RViz cần chuỗi odom → base_footprint → base_link → laser_link. Unity đang phát cạnh đầu. Hai cạnh còn lại mô tả mount cố định, có thể phát trên /tf_static. Static nghĩa là quan hệ với frame cha không đổi, không có nghĩa sensor đứng yên trong thế giới.

Theo URDF: base_footprint → base_link có z=0.05 m; base_link → laser_link có xyz=(0.25,0,0.225) m, orientation identity. Tổng base_footprint → laser_link là (0.25,0,0.275). Unity mount (0,0.225,0.25) tương ứng ROS (0.25,0,0.225). Mô hình này giả định xe chạy phẳng, các mount scene khớp URDF. Odom hiện chiếu pose base_link xuống mặt phẳng và bỏ roll/pitch; không dùng static height này cho địa hình có nghiêng/nhún đáng kể.

## Tránh trùng TF

Repo có amr_description/display.launch chạy robot_state_publisher từ URDF; nó có thể đã phát hai cạnh mount. Kiểm tra trước bằng tf_echo, không khởi chạy cả display.launch và laser_tf.launch cho cùng các cạnh.

[Ubuntu WSL] mở terminal ROS mới:

```bash
docker exec -it ros1_amr_core bash
```

[Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
rosrun tf tf_echo base_footprint base_link
```

Quan sát vài giây rồi Ctrl+C, tiếp tục:

```bash
rosrun tf tf_echo base_link laser_link
```

Nếu cả hai có transform đúng thì bỏ qua launch mới, dùng nguồn hiện có. Nếu cả hai thiếu và không có robot_state_publisher dự kiến cung cấp chúng:

```bash
roslaunch amr_navigation laser_tf.launch
```

Giữ terminal chạy. Nếu chỉ thiếu base mount, thêm publish_laser_mount:=false; nếu chỉ thiếu laser mount, thêm publish_base_mount:=false. Nếu robot_state_publisher đang chạy nhưng thiếu TF, xem lỗi/robot_description của nó trước, không chồng nguồn để che lỗi.

Launch chỉ thêm hai mount, không khởi động bridge, clock, RViz hoặc map transform. Không cần catkin_make chỉ vì thêm launch khi package đã được build/source như các bài trước.

## Kiểm tra và hiển thị

Giữ bridge, /use_sim_time=true và Unity Play với odom/scan publisher. Mở terminal container khác, source như trên:

```bash
rosrun tf tf_echo odom laser_link
```

Pose sensor phải thay đổi khi robot đi/quay. Ctrl+C rồi chạy:

```bash
rviz
```

[RViz]

- Global Options → Fixed Frame = odom.
- Add → TF.
- Add → LaserScan, Topic=/scan, Style=Points, Size (m)=0.03, Decay Time=0.
- Chọn view TopDownOrtho nếu cần nhìn từ trên. LaserScan Status cần OK.
- Lưu config bằng File → Save Config As nếu muốn dùng lại.

## Thực nghiệm

Đặt hộp hoặc tường đứng yên trước robot. Dùng keyboard làm nguồn điều khiển duy nhất. Khi đứng yên, scan hiện mặt vật cản. Khi tiến chậm, khoảng cách sensor tới vật giảm nhưng mặt tường trong odom gần như giữ nguyên vị trí. Khi quay nhẹ, tập điểm có thể đổi/mất vì FOV 90 độ, nhưng phần mặt tường còn được quan sát phải nằm đúng chỗ. Không kỳ vọng nhìn thấy toàn bộ các mặt hộp hoặc vật ngoài mặt phẳng quét.

## Đọc launch

static_transform_publisher nhận x y z qx qy qz qw parent child. Quaternion (0,0,0,1) biểu diễn không xoay tương đối. z=0.05 là từ footprint lên base_link; laser offset tính từ base_link. TF tự ghép các cạnh với odometry để đổi tọa độ điểm scan sang odom.

## Khi hiển thị lỗi

- Unknown frame laser_link / no transform: kiểm tra các cạnh TF và frame_id chính xác; topic /scan tồn tại chưa đủ.
- Fixed Frame map không tồn tại: dùng odom ở bài này; chưa có map/localization.
- Scan bám theo robot thay vì nằm trên vật tĩnh: kiểm tra Fixed Frame, mount và chiều góc, không tuning planner.
- Extrapolation/time: xác nhận /clock chạy, /use_sim_time=true trước khi chạy RViz; thử restart RViz sau khi đổi clock. Odom và scan hiện có lịch publish độc lập, scan mới có thể phải đợi mẫu TF kế tiếp; lỗi kéo dài cần kiểm tra timestamp/tần suất, không thêm offset thời gian tùy ý.
- RobotModel không cần cho bài này; không chạy display.launch chỉ để thêm model khi static mount launch đang chạy.

## Điều kiện hoàn thành

LaserScan Status OK, thấy TF sensor, mặt vật cản đứng yên trong odom khi xe tiến/quay (trong phạm vi được quét). Đây là kiểm chứng cả vị trí/hướng lắp, quy ước góc, odometry và thời gian. Chưa xác nhận runtime trong tài liệu này.

Nguồn CLI ROS1: https://raw.githubusercontent.com/ros/geometry2/noetic-devel/tf2_ros/src/static_transform_broadcaster_program.cpp
