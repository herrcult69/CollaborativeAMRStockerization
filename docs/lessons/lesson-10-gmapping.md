# Bài 10 — Lái xe xây dựng bản đồ với gmapping

## Khái niệm

SLAM vừa ước lượng vị trí vừa xây dựng bản đồ từ quan sát. gmapping nhận /scan và thông tin chuyển động qua TF odom, xuất /map kiểu nav_msgs/OccupancyGrid và TF map → odom. Nó không điều khiển xe; người học lái bàn phím.

Occupancy grid chia mặt phẳng thành ô: occupied (vật cản), free (trống), unknown (chưa biết). Resolution=0.05 nghĩa là mỗi ô 5×5 cm, không phải độ chính xác định vị được bảo đảm. Trong RViz Map color scheme mặc định: vật cản đen, trống trắng, chưa biết xám. Map giữ kết quả nhiều scan, khác /scan chỉ là một lần đo.

## Chuẩn bị

Mở WarehouseTraining, scan 181 tia/180 độ, range_max=10, publish 10 Hz. Giữ bridge, clock, odometry và một nguồn TF mount đang chạy theo daily.md. Scan đã hiển thị đúng trong odom. Không đổi thông số scan giữa một lượt mapping. Không chạy go_to_point/move_base hoặc AMCL trong bài này; không thêm static map → odom.

Dockerfile đã liệt kê ros-noetic-gmapping và ros-noetic-map-server nhưng chưa kiểm tra package runtime trong phiên này.

[Ubuntu WSL — terminal mới]

```bash
docker exec -it ros1_amr_core bash
```

[Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rospack find gmapping
rospack find map_server
roslaunch amr_navigation warehouse_mapping.launch
```

Hai lệnh rospack cần trả về đường dẫn. Nếu thiếu package, ghi lỗi để xử lý trước, không mặc định phải build lại project. Launch mới không cần catkin_make khi package amr_navigation đã được build/source. Nếu thiếu setup, xem lỗi mount tại daily.md.

Launch khởi động slam_gmapping; /use_sim_time=true, base_frame=base_footprint, odom_frame=odom, map_frame=map. Map resolution 0.05 m, map update interval 1 s. Xử lý scan sau dịch chuyển khoảng 0.2 m/quay 0.1 rad hoặc temporal update 1 s. Đây là cấu hình khởi đầu, chưa tuning hoặc xác nhận chất lượng map.

## RViz và lái xe

[RViz] đổi Fixed Frame từ odom sang map khi gmapping bắt đầu nhận scan. Add → Map → Topic=/map. Giữ LaserScan=/scan và TF. Nếu map chưa tồn tại ngay, chờ Unity chạy/scan tới; không tạo TF giả để hết lỗi.

[Unity] click Game view, WASD lái chậm. Từ ô xanh đi dọc lối trái, vòng qua đầu kệ, lần lượt khám phá các lối giữa/phải và đường ngang. Quan sát các mặt kệ từ nhiều phía rồi trở lại gần điểm xuất phát. Quay chậm để các scan liên tiếp còn chồng lấp. Tia không xuyên kệ: ô phía sau chưa quan sát có thể còn xám; không cần mọi ô trong map chuyển trắng.

Không Stop/Play giữa lượt vì Unity reset clock và gốc odom. Nếu cần làm lại: dừng gmapping bằng Ctrl+C, dừng controller nếu có, reset Play, rồi khởi động gmapping mới; restart RViz nếu giữ dữ liệu thời gian cũ.

## Lưu bản đồ

Giữ Unity Play và gmapping đang chạy, thả phím cho xe đứng yên. Mở một terminal container khác và source hai môi trường như trên.

[Inside ROS Docker container]

```bash
rosrun map_server map_saver -f /catkin_ws/src/amr_navigation/maps/warehouse_training_01
```

Đợi báo Done. Nếu chờ mãi, kiểm tra /map có message (ví dụ rostopic echo -n 1 /map/info ở terminal khác); tên topic tồn tại chưa đủ. Lưu lần khác dùng tên mới để không ghi đè kết quả muốn giữ.

Kết quả dự kiến là warehouse_training_01.pgm và warehouse_training_01.yaml trong thư mục maps. PGM lưu lưới bản đồ; YAML lưu resolution, origin, đường dẫn ảnh và thresholds. Chúng nằm trong workspace bind mount nên tồn tại trên Windows. Chỉ sau khi lưu thành công mới dừng gmapping/Unity. Khi chuyển sang localization, dừng gmapping trước khi chạy AMCL.

## Kết quả và giới hạn

Đã thêm warehouse_mapping.launch, khai báo runtime dependencies gmapping/map_server, tạo thư mục maps; chưa chạy SLAM hoặc tạo map thực đo trong phiên này. Điều kiện qua bài: map phản ánh các lối/kệ đã quan sát, không lệch/chồng tường nghiêm trọng, lưu được cặp PGM/YAML. Bài sau đọc map đã lưu bằng map_server và định vị AMCL.

Nếu map chồng tường hoặc xoay sai: xem scan/TF/clock và tốc độ lái trước, chưa tuning planner. Odometry hiện lấy ground truth Unity nên bài này chưa đại diện cho độ trôi encoder thực tế.

Nguồn implementation/parameters: https://raw.githubusercontent.com/ros-perception/slam_gmapping/melodic-devel/gmapping/src/slam_gmapping.cpp
