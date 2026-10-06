# Bài 2 — Xe tự đến B trong vùng trống

Người dùng đã xác nhận `/odom` và TF khớp với chuyển động trong RViz, đã tắt RosTestBridge và dọn đường trống.

## Cách hoạt động

Node `go_to_point` đọc pose từ `/odom`, tính khoảng cách đến goal trong frame `odom`, rồi gửi `/cmd_vel` ở khoảng 20 Hz. Nếu lệch hướng trên 0,35 rad thì quay trước; khi hướng phù hợp thì tiến tối đa 0,2 m/s. Tốc độ quay giới hạn 0,5 rad/s. Đến trong bán kính 0,2 m thì gửi lệnh dừng và kết thúc.

Đây là closed-loop control: lệnh được tính lại dựa trên pose mới. Chưa có tránh vật cản hoặc yêu cầu hướng cuối tại đích. Việc node báo đến đích cần được đối chiếu với xe thực tế dừng trong Unity.

Node kiểm tra frame, giá trị pose và độ mới của odometry. Khi dữ liệu cũ quá 0,5 giây hoặc clock lệch quá 0,5 giây, node gửi dừng rồi kết thúc. Đây là dừng qua ROS; nếu mất cả kết nối đến Unity thì không bảo đảm lệnh đến nơi. Có thể Stop Play để dừng simulation.

## Chuẩn bị

**Clock:** trước khi chạy controller, trong ROS Docker container chạy `rosparam set /use_sim_time true`. Publisher Unity hiện phát `/clock` và đóng dấu `/odom`, TF bằng cùng simulation time. Restart RViz nếu vừa chuyển từ wall time. Không tăng ngưỡng tuổi message để che lỗi lệch clock.

**[Unity]**

- Giữ RosTestBridge tắt, StackerController và PlanarOdometryPublisher bật.
- Robot Base của publisher trỏ tới `base_link`.
- Stop rồi Play lại để đặt gốc odom tại A. Giữ xe đứng yên, không nhấn WASD trong lúc node điều khiển.
- Đảm bảo khoảng 2 m phía trước theo hướng bắt đầu không có vật cản.
- Giữ endpoint hoạt động. Không chạy move_base hoặc publisher `/cmd_vel` khác cùng lúc.

## Build nếu chưa build sau khi thêm node

**[Ubuntu WSL]**

```bash
docker exec -it ros1_amr_core bash
```

Mở shell trong container hiện có.

**[Inside ROS Docker container]**

```bash
source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin_make
source /catkin_ws/devel/setup.bash
```

Build đăng ký script Python với catkin để roslaunch tìm được executable. Mong đợi build kết thúc không có lỗi. Nếu đã build thành công, chỉ cần source môi trường trong shell mới.

## Chạy đến B

**[Inside ROS Docker container]**

```bash
roslaunch amr_navigation go_to_point.launch goal_x:=2.0 goal_y:=0.0
```

Lệnh này bắt đầu điều khiển xe. B là (2,0) trong `odom`, tức 2 m phía trước mốc bắt đầu Play, không phải 2 m phía trước pose hiện tại.

Log `Distance` phải giảm; khi dưới 0,2 m sẽ thấy `ARRIVED` và node gửi vận tốc 0. Xe có thể cần thời gian ngắn để dừng do physics. Kiểm tra pose cuối vẫn nằm trong ngưỡng mong muốn.

Ctrl+C kết thúc node và gửi lệnh dừng. Dừng Play trước khi reset vị trí xe hoặc bắt đầu một lượt mới; không reset odom trong khi controller vẫn chạy.

## Kiểm chứng

**[Inside ROS Docker container, terminal khác]**

```bash
rostopic echo /cmd_vel
```

Xem lệnh: tiến có linear.x dương; lệnh cuối có linear.x và angular.z bằng 0. Ctrl+C dừng xem.

```bash
rostopic echo /odom/pose/pose/position
```

Xem pose đến gần (2,0); sai số Euclidean sqrt((x-2)^2+y^2) nên không quá 0,2 m sau khi dừng. RViz cho thấy mũi tên tới đích, Unity xác nhận xe đứng yên. Chưa đánh dấu milestone hoàn tất chỉ dựa vào code hoặc log.

Nếu xe quay sai hướng, khoảng cách tăng, hoặc node báo clock lệch: Ctrl+C, Stop Play và gửi log để chẩn đoán trước khi thử lại.

Sau khi đích thẳng hoạt động, có thể thử một đích lệch như (2,1) trong vùng trống để kiểm tra cả quay và tiến.

## Lỗi đã gặp: Unity và ROS dùng clock không đồng bộ

### Triệu chứng

Khi chạy controller, node kết thúc ngay với log:

```text
Goal in odom: (2.00, 0.00), tolerance 0.20 m
Waiting for /odom
Odometry stale or clocks disagree. Stopped; restart this node after fixing it.
```

`Waiting for /odom` thoáng xuất hiện lúc khởi động chưa phải lỗi. Lỗi xảy ra sau khi node nhận được message nhưng kiểm tra thời gian không đạt. `process has finished cleanly` nghĩa là node chủ động kết thúc, không có nghĩa xe đã đến đích.

### Kết quả đo và nguyên nhân

- `/odom` vẫn tới đều khoảng **21 Hz**: không phải mất luồng odometry.
- `/use_sim_time` chưa được đặt: ROS mặc định dùng wall time của hệ điều hành trong môi trường Docker/WSL.
- Độ trễ đo được khoảng **−0,58 đến −0,94 giây**: timestamp của message đi trước thời gian ROS.

**Clock** là nguồn thời gian hiện tại. **Timestamp** là thời điểm được ghi vào message. **Wall time** là giờ hệ điều hành; **simulation time** là thời gian của simulation.

Cách cũ tạo thời gian như sau:

```text
Unity: giờ UTC của Windows lúc bắt đầu + thời gian trôi qua trong Unity
ROS:   giờ hệ điều hành trong môi trường Docker/WSL
```

Controller tính:

```text
Tuổi message = thời gian ROS hiện tại − timestamp của /odom
```

Ví dụ ROS đang ở giây 100,0 nhưng message ghi giây 100,7 thì tuổi message là −0,7 giây. Theo clock của ROS, message nằm "trong tương lai". Ngưỡng chấp nhận là ±0,5 giây nên controller gửi lệnh dừng rồi thoát.

**Nguyên nhân đã xác nhận là hai nguồn thời gian không đồng bộ.** Đây không phải sự không tương thích giữa Unity và Docker. Phép đo chưa tách riêng được mức đóng góp của clock hệ điều hành và cách publisher cũ cộng thời gian đã trôi qua; không kết luận chính xác clock nào bị sai chỉ từ log này.

### Cách sửa đã triển khai

`PlanarOdometryPublisher` hiện dùng simulation time của Unity (`Time.timeAsDouble`) làm nguồn chung:

```text
Unity simulation time
    ├── /clock → ROS dùng làm thời gian hiện tại khi /use_sim_time = true
    ├── timestamp của /odom
    └── timestamp của TF
```

Như vậy, timestamp và clock ROS cùng một nguồn, không phụ thuộc vào việc giờ Windows và Docker có khớp nhau hay không. Vẫn giữ kiểm tra dữ liệu cũ; không tăng ngưỡng để bỏ qua lỗi clock. Trễ truyền thực tế hoặc mất dữ liệu vẫn có thể khiến controller dừng.

### Thứ tự áp dụng và kiểm tra

1. **[Terminal chạy controller]** Nếu controller còn chạy, nhấn Ctrl+C để kết thúc trước khi reset simulation.
2. **[Unity]** Stop Play và chờ compile script mới. Đóng RViz nếu đang dùng clock cũ.
3. **[Inside ROS Docker container]** Chạy:

   ```bash
   rosparam set /use_sim_time true
   rosparam get /use_sim_time
   ```

   Lệnh đầu chọn simulation time; lệnh sau phải trả về `true`. Nếu ROS master khởi động lại, kiểm tra lại parameter này vì thiết lập thủ công không tự được lưu qua lần khởi động master mới.

4. **[Unity]** Nhấn Play, giữ endpoint hoạt động và xe đứng yên.
5. **[Inside ROS Docker container]** Chạy:

   ```bash
   rostopic echo -n 1 /clock
   ```

   Mong đợi message có `clock.secs` là số giây từ đầu lượt Play, ví dụ 8, thay vì Unix timestamp dạng `178983...`. Nếu lệnh cứ chờ, kiểm tra Unity Console, publisher và endpoint trước khi chạy controller.

6. **[Inside ROS Docker container]** Chạy lại:

   ```bash
   roslaunch amr_navigation go_to_point.launch goal_x:=2.0 goal_y:=0.0
   ```

   Lệnh bắt đầu điều khiển xe. Có thể mở lại RViz từ terminal khác với Fixed Frame = `odom`.

Chỉ để một publisher phát `/clock`. Không Stop/Play lại khi controller còn chạy vì thao tác đó reset cả simulation time và gốc `odom`.

### Nếu cần chẩn đoán lại

**[Inside ROS Docker container]**

```bash
rostopic hz /odom
```

Kiểm tra dữ liệu có tới đều không; mong đợi gần tần suất publisher. Ctrl+C để dừng xem, rồi chạy:

```bash
rostopic delay /odom -w 5
```

Lệnh đo chênh lệch giữa clock ROS và timestamp message trên cửa sổ 5 mẫu. Giá trị âm đáng kể cho thấy timestamp đi trước clock ROS; giá trị dương lớn có thể là dữ liệu trễ hoặc lệch clock. Với `/clock` được gửi riêng qua mạng, có thể có chênh lệch nhỏ do thứ tự nhận và xử lý. Ctrl+C để dừng xem.

Log controller đã được bổ sung `receipt gap` (thời gian thực từ lúc nhận mẫu cuối) và `ROS time minus stamp` (tuổi timestamp theo ROS), giúp phân biệt ngừng nhận dữ liệu với vấn đề timestamp.

**Trạng thái:** đã sửa code và hướng dẫn; các test toán điều khiển pass. Chưa có xác nhận từ người dùng rằng xe đã đến B sau khi đổi clock.
