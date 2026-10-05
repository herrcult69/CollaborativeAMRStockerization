# Bài 11 — Dùng bản đồ đã lưu và định vị AMCL

## Trạng thái và khái niệm

Đã tìm thấy warehouse_training_01.pgm và warehouse_training_01.yaml trong amr_navigation/maps. YAML có resolution=0.05, origin=(-20,-20,0) và image=/catkin_ws/src/amr_navigation/maps/warehouse_training_01.pgm. Xác nhận có file không đồng nghĩa đã kiểm định chất lượng bản đồ.

map_server đọc cặp file và cung cấp /map. AMCL (Adaptive Monte Carlo Localization) dùng tập giả thuyết vị trí/hướng, gọi là particles, đối chiếu /scan với map cùng chuyển động odometry để cập nhật ước lượng. AMCL phát /amcl_pose, /particlecloud và TF map → odom. Bản đồ giữ nguyên trong bài localization.

Đã thêm warehouse_localization.launch và dependency amcl. Kiểm tra XML; chưa chạy launch trong container hoặc xác nhận hội tụ AMCL.

## Chuẩn bị

Dừng gmapping bằng Ctrl+C trong terminal mapping. Không dừng bridge hoặc nguồn mount TF. Không chạy AMCL cùng gmapping hoặc static publisher cho map → odom. rosnode list phải không còn /slam_gmapping trước khi localization. Không chạy node điều khiển tự động trong bài này.

Có thể giữ lượt Unity Play hiện tại. Nếu muốn reset về điểm xuất phát, dừng các node phụ thuộc thời gian như gmapping/AMCL trước, Stop/Play Unity rồi khởi động localization; mở lại RViz nếu còn cache thời gian cũ. Giữ cùng scene/vị trí kệ với lúc tạo bản đồ.

## Chạy [Ubuntu WSL — terminal mới]

```bash
docker exec -it ros1_amr_core bash
```

## Chạy [Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
rospack find amcl
roslaunch amr_navigation warehouse_localization.launch
```

Launch mặc định đọc map vừa tìm thấy. Không tự khởi động bridge, sensor TF, Unity hoặc move_base. map_server/AMCL phải là nguồn duy nhất của các chức năng tương ứng. Nếu rospack không tìm thấy amcl, xử lý package runtime trước; Dockerfile đã có ros-noetic-amcl.

## RViz

Fixed Frame=map; Map topic=/map; LaserScan topic=/scan; TF như bài trước. Thêm PoseArray topic=/particlecloud để thấy giả thuyết, tùy chọn PoseWithCovariance topic=/amcl_pose để xem ước lượng pose.

Nhấn 2D Pose Estimate, click tại vị trí gần đúng của robot trên bản đồ rồi kéo chuột theo hướng đầu robot và thả. Đây là ước lượng vị trí ban đầu, không phải goal hay lệnh di chuyển. Định vị gốc robot gần base_footprint, không chọn vị trí sensor hoặc đầu càng.

Không dùng trực tiếp Unity world (-6,0,-6) làm map pose. YAML origin (-20,-20,0) mô tả góc gốc ảnh lưới trong map, không phải nơi robot xuất phát. Hướng màn hình camera Unity khác hướng bản đồ ROS; so sánh theo tường/kệ. Nếu khởi tạo cùng chỗ với lượt mapping, pose đầu có thể gần gốc map nhưng vẫn kiểm tra scan thay vì mặc định.

Lái tay tiến/quay chậm trong khoảng trống để AMCL cập nhật. Quan sát scan khớp các cạnh kệ/tường và particles tập trung quanh robot. Kệ lặp lại dễ gây nhầm dãy; chọn initial pose gần đúng và quan sát thêm vùng có hình dạng riêng. Particlecloud tập trung không đủ nếu scan vẫn khớp sai dãy.

## Điều kiện qua bài

Map hiển thị từ file sau khi dừng SLAM; scan chồng đúng phần vật cản đang thấy, vẫn ổn khi di chuyển; TF map → odom do AMCL cung cấp. AMCL không tự lái xe. Bước sau mới cấu hình footprint/costmaps/move_base.

Nếu chờ initial pose hoặc map → odom chưa hiện, xác nhận /scan,/clock và chuỗi odom → laser_link đang hoạt động rồi gửi 2D Pose Estimate. Nếu scan lệch, kiểm tra pose/hướng ban đầu, TF, thời gian và scene có đổi so với map không. Không chỉnh YAML origin để ép scan khớp và không thêm static map → odom.

Nguồn cấu hình AMCL ROS1: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/amcl/examples/amcl_diff.launch
