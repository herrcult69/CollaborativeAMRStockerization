# Bài 12 — Footprint và costmap preview

## Thay đổi và trạng thái

Thêm costmap_common/global_costmap/local_costmap.yaml, costmap_preview.launch và Unity FootprintPreview.cs. Chưa chạy ROS/Unity trong phiên này. Footprint hình chữ nhật là hình bao tạm cho xe không hàng: ROS x từ -0.90 tới +0.35, y từ -0.35 tới +0.35 m, padding 0.02 m. Lấy biên dài dựa trên URDF (đầu càng -0.85, thân trước +0.30), chiều rộng chọn bảo thủ; chưa xác nhận bao phủ toàn collider scene hoặc hàng.

## Kiểm tra footprint [Unity]

Stop Play sau khi dừng AMCL nếu cần gắn component; khi Play lại cần khởi động lại AMCL/RViz để bỏ clock cũ, rồi 2D Pose Estimate. Gắn FootprintPreview lên base_link chuyển động, giữ giá trị mặc định. Bật Gizmos, nhìn Scene từ trên: viền cyan phải bao collider thân, bánh, càng và hàng nếu có. Nó chỉ vẽ, không đổi collider/physics. Không gắn lên wrapper base_footprint đứng yên. Nếu có phần nhô ra, phải đo và sửa cả hình bao ROS lẫn preview trước khi tự lái.

Tên frame base_footprint không tự định nghĩa polygon. YAML dùng polygon trong hệ ROS (x trước, y trái). Hình preview ở độ cao base_link để nhìn dễ; chỉ kiểm tra hình chiếu mặt bằng. Tính cả padding, khung rộng 0.74 m, dài 1.29 m. Polygon này chưa phải cấu hình navigation đã nghiệm thu.

## Chạy costmap

Giữ bridge, scan/odom/clock, một nguồn mount TF, map_server+AMCL đang chạy; SLAM dừng. Đặt initial pose và xác nhận scan khớp map. Không chạy move_base khác hoặc go_to_point.

[Ubuntu WSL]

```bash
docker exec -it ros1_amr_core bash
```

[Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch amr_navigation costmap_preview.launch
```

Launch dùng move_base để quản lý hai costmap nhưng đổi cmd_vel sang /costmap_preview/cmd_vel_unused; robot Unity vẫn chỉ nghe /cmd_vel. Không gửi goal bài này. Recovery bị tắt. Khi học navigation thật phải có cấu hình planner/giới hạn vận tốc và launch riêng đã kiểm chứng.

## Hiển thị [RViz]

Fixed Frame=map. Add Map với topic /costmap_preview/global_costmap/costmap, Color Scheme=costmap, Alpha=0.6. Thêm Map thứ hai topic /costmap_preview/local_costmap/costmap, cùng color scheme. Bật từng bản đồ để dễ quan sát, tránh hai lớp màu che nhau. Giữ map gốc /map Alpha thấp nếu cần.

Add Polygon → /costmap_preview/local_costmap/footprint. Local costmap là cửa sổ 8×8 m đi theo robot trong odom, global dùng map. Cả hai lấy static map + scan + inflation. Lớp static trong local giúp giữ hình học đã biết ngay cả ngoài FOV 180 độ; vật mới chưa được scan thấy vẫn không được biết.

## Thực nghiệm

Trong Unity Play, đặt Cube mới ở vùng trống đã map, cách trước robot khoảng 2 m. Collider thuộc Default và cắt mặt phẳng quét. Costmap phải ghi nhận mặt thùng và vùng inflation trong khi map gốc vẫn giữ nguyên. Tắt Cube khi robot vẫn nhìn vào chỗ ấy: các tia quan sát vùng trống sẽ xóa dấu obstacle do scan trước để lại. Nếu tắt vật ngoài góc quét, không yêu cầu xóa ngay. Không dùng thí nghiệm này để xóa tường/kệ nằm sẵn trong static map: lớp static tiếp tục giữ chúng.

inf_is_valid=true cho phép dùng các tia +inf để clearing tới tầm 10 m; obstacle_range=9 m ngăn đánh dấu endpoint chuyển đổi gần 10 m thành vật giả. expected_update_rate=0.5 là khoảng thời gian kỳ vọng tối đa giữa quan sát theo giây, không phải Hz. Obstacle marking/clearing hoạt động trong vùng và độ cao được cấu hình.

inflation_radius=0.65 m, cost_scaling_factor=3 là giá trị khởi đầu để quan sát, không phải clearance được bảo đảm. Tăng bán kính không sửa được footprint sai. Tăng transform_tolerance tùy ý không phải cách sửa clock/TF thiếu.

## Điều kiện qua bài

Footprint bao robot theo cấu hình thử; cả costmap hiển thị, scan obstacle marking/clearing và inflation hoạt động hợp lý. Node preview không phát điều khiển về Unity. Chưa khẳng định runtime thành công; cần thực hành trước khi cấu hình autonomous goal.

Nguồn ROS1 obstacle layer: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/costmap_2d/plugins/obstacle_layer.cpp
