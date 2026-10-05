# Bài 13 — Goal đầu tiên bằng move_base

## Thay đổi và trạng thái

Thêm warehouse_navigation.launch và dwa_warehouse.yaml, khai báo navfn runtime dependency. Dùng lại costmap của bài preview, không sửa footprint hoặc Unity controller. Chưa chạy autonomous simulation, chưa đo sai số goal hoặc xác nhận tránh vật cản.

NavFn tìm global path trên costmap, DWA thử các quỹ đạo ngắn và chọn vận tốc (linear.x, angular.z). move_base điều phối và phát /cmd_vel. Path không phải lệnh vận tốc. Robot differential drive nên vận tốc ngang y=0. Cấu hình thử đầu chỉ tiến/quay, không lùi vì sensor quét phía trước 180 độ; quay và phần càng sau vẫn cần footprint/map đúng.

Giới hạn khởi đầu: tiến 0.25 m/s, quay 0.45 rad/s; tolerance vị trí 0.20 m, hướng 0.20 rad (~11.5 độ). Gia tốc planner 0.4 m/s², 0.6 rad/s² là giả định cần kiểm chứng với physics, không phải giới hạn áp đặt lên actuator. Recovery tắt để lần đầu quan sát rõ nguyên nhân khi không tìm được chuyển động hợp lệ. NavFn không lập đường qua vùng unknown.

## Trước khi chạy

Giữ bridge, clock/odom/scan, TF mount và map_server+AMCL. Dừng gmapping, go_to_point và costmap_preview. Không chạy thêm launch move_base.launch cũ. Xác nhận scan khớp map và hình footprint bao collider xe/càng ở cấu hình không hàng đang thử; bài trước chỉ mới có hình bao tạm. Để xe đứng yên ở vùng trống, không nhấn WASD khi ROS đang điều khiển.

## Chạy

[Ubuntu WSL — terminal mới]

```bash
docker exec -it ros1_amr_core bash
```

[Inside ROS Docker container]

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch amr_navigation warehouse_navigation.launch
```

Giữ terminal chạy. Launch không tự khởi động localization/bridge/TF; nó dùng các node đã chạy. Không cần rebuild chỉ vì thêm YAML/launch trong workspace đã build/source.

## RViz

Fixed Frame=map. Sửa các topic /costmap_preview/... từ bài trước thành /move_base/...:

| Display | Topic |
|---|---|
| Map | /move_base/global_costmap/costmap |
| Map | /move_base/local_costmap/costmap |
| Polygon | /move_base/local_costmap/footprint |
| Path | /move_base/NavfnROS/plan |
| Path | /move_base/DWAPlannerROS/local_plan |

Hai Path có thể chưa có dữ liệu trước khi gửi goal. Giữ /map, /scan và TF khi cần quan sát. Dùng 2D Pose Estimate chỉ để đặt vị trí thật nếu localization chưa đúng, không để đặt đích.

## Thử goal đầu

[RViz] Nhấn 2D Nav Goal, chọn điểm trong vùng trống cách robot 1–2 m phía trước; kéo mũi tên cùng hướng xe. Chọn điểm đủ xa kệ và tường để toàn footprint tại goal không va chạm. Lệnh này bắt đầu điều khiển robot, không chỉ vẽ mũi tên.

[Unity] Quan sát xe chạy theo đường và dừng. Không dùng WASD cùng lúc; bàn phím trong controller hiện tại có thể ghi đè lệnh ROS.

[Inside ROS Docker container — terminal khác đã source] Có thể mở trước khi gửi goal:

```bash
rostopic echo /move_base/result
```

status.status=3 là SUCCEEDED; 4 là ABORTED; 2 là PREEMPTED. Không suy ra thành công chỉ từ robot dừng. Ghi pose đầu, goal, pose cuối và quan sát xe thực tế dừng. Position tolerance là cấu hình ước lượng, không phải số đo sai số đã chứng minh.

## Hủy goal/dừng

[Inside ROS Docker container — terminal khác đã source]

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{stamp: {secs: 0, nsecs: 0}, id: ''}"
```

Lệnh hủy các goal đang hoạt động. Nếu cần dừng simulation ngay, Pause trong Unity. Controller hiện tại giữ targetVelocity và chưa có watchdog /cmd_vel, nên không coi kill node hoặc mất bridge là đảm bảo bánh dừng. Trước khi unpause sau lỗi kết nối phải khôi phục điều khiển và gửi dừng; kết thúc lượt có thể Stop Play sau khi dừng các node phụ thuộc clock. Không chỉ Ctrl+C move_base rồi giả định xe đã nhận lệnh zero.

## Bước thử tiếp trong cùng bài

Sau goal thẳng thành công, thử goal gần cần rẽ nhẹ trong vùng rộng. Chỉ sau đó đặt một Cube cố định chắn đường thẳng tới goal nhưng có lối vòng rộng, xác nhận nó hiện trong costmap rồi gửi goal mới. Không kéo Cube vào xe đang chạy. Nếu ABORTED, giữ log planner/controller và costmap để tìm nguyên nhân, chưa tăng tốc hay giảm footprint cho xe lọt.

Mục tiêu: kết nối đầy đủ map/localization/costmap/planner/controller; chưa đánh dấu milestone 2 hoàn tất trước khi có bằng chứng các kịch bản tránh vật cản và bị chặn.

Nguồn DWA: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/dwa_local_planner/cfg/DWAPlanner.cfg
Nguồn limits: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/base_local_planner/src/local_planner_limits/__init__.py

## Debug oscillation — 2026-09-26

Người học báo goal abort tại sim time 1207.2304, robot quay tại chỗ hoặc lắc trái/phải. Kiểm tra runtime: /cmd_vel chỉ có publisher /move_base và subscriber /unity_endpoint; odom tồn tại, đọc được pose, xe đứng yên sau abort. Timeout thực tế 15 s, distance 0.10 m; recovery_behavior_enabled=false. DWA runtime khớp YAML (min/max angular speed 0.08/0.45). Không lấy được current_goal trong cửa sổ đọc 3 s; chưa có chuỗi cmd_vel/odom trước abort để xác nhận nguyên nhân dao động.

move_base reset watchdog oscillation dựa trên độ dịch chuyển vị trí, không tính riêng tiến triển về yaw. Ở tốc độ quay 0.08 rad/s, quay pi rad có thể cần khoảng 39 s nên ngưỡng 15 s có thể ngắt lượt quay hợp lệ. Đã đổi default timeout thành 60 s qua launch argument, giữ nguyên distance, planner và recovery. Đây là nới thời gian quan sát có cơ sở, chưa chứng minh chữa được lắc trái/phải.

Áp dụng bằng Ctrl+C terminal navigation rồi chạy lại warehouse_navigation.launch; không Stop/Play Unity hoặc restart AMCL trong lần đổi này. Thử goal gần phía trước, hướng cuối giống hướng đầu robot. Nếu vẫn lắc, đọc /cmd_vel/angular/z và /odom/twist/twist/angular/z trong lúc đang chạy để phân biệt planner đổi dấu lệnh và physics phản ứng không khớp; không tắt watchdog hoặc giảm footprint để che lỗi.

Câu log “Even after executing all recovery behaviors” là thông báo chung ở nhánh abort; không chứng minh recovery đã chạy khi recovery bị tắt. Chưa xác nhận test sau thay đổi timeout.

### Sửa mâu thuẫn giới hạn DWA khi khởi động

Sau hướng dẫn thử tăng min_vel_trans từ 0.03 lên 0.10, người học gặp DWA failed to produce path / valid control could not be found. Đối chiếu source SimpleTrajectoryGenerator: với use_dwa=true và controller_frequency=10 Hz, từ đứng yên cửa sổ tiến tối đa 0.4×0.1=0.04 m/s, cửa sổ quay tối đa 0.6×0.1=0.06 rad/s. Cả hai đều dưới ngưỡng min_vel_trans=0.10 và min_vel_theta=0.08; mọi mẫu khởi động bị loại ở bước kiểm tra vận tốc tối thiểu, trước khi xét costmap.

Đã hoàn nguyên min_vel_trans=0.03 và thêm chú thích tránh tăng riêng tham số này. Kiểm tra toán cửa sổ mẫu xác nhận có mẫu vượt ngưỡng tiến; chưa xác nhận runtime và chưa giải quyết vấn đề physics không đáp ứng lệnh nhỏ đã quan sát trước đó. Hướng dẫn tăng riêng min_vel_trans trước đó thiếu kiểm tra tính tương thích với gia tốc/tần suất.

Áp dụng bằng dừng rồi chạy lại warehouse_navigation.launch, không reset Unity/AMCL. Nếu xe vẫn không di chuyển ở lệnh nhỏ sau khi DWA có lệnh, tiếp tục đo đáp ứng vận tốc ROS trực tiếp khi planner đã dừng; không suy ra ma sát/drive là nguyên nhân chỉ từ target velocity. Không tăng acc_lim chỉ để vượt bộ lọc khi chưa đo đáp ứng thực.

Nguồn: https://raw.githubusercontent.com/ros-planning/navigation/noetic-devel/base_local_planner/src/simple_trajectory_generator.cpp
