# Milestone 1 — Xe đi từ A đến B

## Goal

Xe trong Unity đi từ điểm A đến điểm B. Bài đầu xác minh chuyển động thủ công; mục tiêu cuối là gửi một điểm B và xe tự đến gần điểm đó rồi dừng, trong vùng thử nghiệm phẳng, trống. Tránh vật cản bằng navigation stack là bước mở rộng sau khi nền tảng hoạt động.

Tiêu chí dự kiến cho thử nghiệm cuối: xe đến cách B không quá 0,2 m và dừng. Đây là ngưỡng thử nghiệm ban đầu, có thể điều chỉnh theo kích thước và độ chính xác robot.

## Trình tự học qua thực hành

Tiến độ: người dùng đã xác nhận kết nối Unity–ROS, keyboard control và mũi tên odometry/TF trong RViz khớp chuyển động của xe. Đã triển khai controller đến điểm B, build và test toán điều khiển thành công. Lần chạy controller gặp lỗi clock; đã sửa sang simulation time, nhưng chưa có xác nhận xe đến B và dừng sau sửa lỗi. **Milestone chưa được đánh dấu hoàn tất.**

1. Xác minh ROS master, Unity endpoint và kết nối Unity.
2. Dùng keyboard đưa xe từ A đến B gần đó để kiểm tra physics và controller.
3. Publish ideal simulated odometry trên `/odom` và TF `odom → base_footprint` từ cùng một pose.
4. Hiển thị trong RViz: di chuyển thẳng làm position đổi; quay tại chỗ làm orientation đổi.
5. Dùng feedback từ `/odom` để tạo `/cmd_vel`, điều khiển xe đến B trong vùng trống và dừng.
6. Thêm `/scan`, map/localization và `move_base` cho navigation có vật cản.

## Kết quả kiểm tra ngày 2026-09-19

Phát hiện khi thực hành: Unity object `base_footprint` không có ArticulationBody; `base_link` mới là thân vật lý cần theo dõi. Robot Base của publisher phải gán `base_link`. ROS frame đầu ra vẫn là `base_footprint`, được tính bằng pose phẳng của thân xe. Đã sửa hướng dẫn và thêm kiểm tra cấu hình trong script; sau đó người dùng đã xác nhận hiển thị RViz đúng.

Các dòng dưới đây là trạng thái **ở lần kiểm tra ban đầu**, trước khi hoàn thành kết nối và đổi clock:

- Container `ros1_amr_core` đang chạy.
- ROS master truy cập được; `rosnode list` chỉ thấy `/rosout` tại thời điểm kiểm tra.
- Chưa thấy Unity endpoint node hoặc các topic sensor tại thời điểm kiểm tra.
- `/use_sim_time` chưa được đặt, ROS mặc định dùng wall time.
- `StackerController.cs` đã có keyboard control và nhận `/cmd_vel`.
- `RosTestBridge.cs` cũng nhận `/cmd_vel` và có `transform.Translate` trong callback. Nếu component này gắn lên robot, cần tắt nó trước khi thử điều khiển `/cmd_vel`, tránh hai component cùng tác động lên chuyển động.

Đây là kết quả kiểm tra theo thời điểm, không phải trạng thái hiện tại hoặc trạng thái được bảo đảm cho những lần khởi động sau. Tiến độ mới nhất được tổng hợp bên dưới.

## Danh sách file đã tạo và sửa trong milestone

Danh sách này ghi nhận công việc trong chuỗi bài thực hành của cuộc trao đổi; không phải bản thống kê mọi thay đổi trong repository. Các đường dẫn bên dưới tính từ file này.

### Source code và cấu hình build

| File | Thay đổi | Mục đích / nội dung thực hiện |
|---|---|---|
| [PlanarOdometryPublisher.cs](../amr_ware_house/Assets/PlanarOdometryPublisher.cs) | Tạo mới, sau đó sửa khi debug | Đọc pose của `base_link`; lấy pose bắt đầu Play làm gốc `odom`; chuyển tọa độ Unity sang ROS cho mô hình phẳng; tính vận tốc từ hai mẫu; publish `/odom` và TF `odom → base_footprint`. Thêm kiểm tra tránh gán nhầm wrapper; thay clock ban đầu bằng simulation time và publish `/clock`. |
| [go_to_point.py](../CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/scripts/go_to_point.py) | Tạo mới, bổ sung log chẩn đoán | ROS node đọc `/odom`, tính khoảng cách và sai lệch hướng đến B, publish `/cmd_vel`; quay trước nếu lệch hướng lớn, giảm tốc gần đích, gửi dừng khi vào tolerance. Kiểm tra frame, pose và thời gian; log riêng receipt gap và tuổi timestamp khi lỗi. |
| [go_to_point.launch](../CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/launch/go_to_point.launch) | Tạo mới | Cho phép truyền `goal_x`, `goal_y`; cấu hình tolerance 0,2 m, vận tốc tiến tối đa 0,2 m/s và vận tốc quay tối đa 0,5 rad/s. |
| [CMakeLists.txt](../CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/CMakeLists.txt) | Sửa | Thêm `catkin_install_python` để catkin đăng ký executable `go_to_point.py`, giúp roslaunch tìm được node. |
| [test_go_to_point.py](../CollaborativeAMRStockerization/catkin_ws/src/amr_navigation/scripts/test_go_to_point.py) | Tạo mới | 4 test offline: dừng tại đích, quay trước khi tiến, xử lý góc qua biên ±π, hội tụ trong mô phỏng động học lý tưởng với nhiều goal. Không publish lệnh điều khiển robot. |

### Tài liệu

| File | Thay đổi | Nội dung |
|---|---|---|
| [definitions.md](definitions.md) | Tạo mới | Định nghĩa tiếng Việt và ví dụ cho coordinate frame, transform, TF, `/tf_static`, odometry, LiDAR và navigation. |
| [lesson-01-odom-tf.md](lesson-01-odom-tf.md) | Tạo mới và cập nhật | Hướng dẫn gắn publisher, kiểm tra `/odom`, TF và RViz; sửa nguồn pose thành `base_link`; cập nhật cách dùng `/clock`. |
| [lesson-02-go-to-point.md](lesson-02-go-to-point.md) | Tạo mới và cập nhật | Build/run controller, kiểm chứng đến B; ghi lại lỗi clock, số đo, nguyên nhân và quy trình khắc phục. |
| [milestone-1.md](milestone-1.md) | Tạo mới và cập nhật | Goal, tiến độ, danh sách file, quá trình thực hiện và bằng chứng phục vụ report. |

### Các thành phần được sử dụng nhưng không ghi nhận sửa source trong bài này

- [StackerController.cs](../amr_ware_house/Assets/StackerController.cs): tận dụng keyboard control và subscriber `/cmd_vel` có sẵn để điều khiển bánh xe.
- [RosTestBridge.cs](../amr_ware_house/Assets/RosTestBridge.cs): phát hiện script cũng di chuyển GameObject khi nhận `/cmd_vel`; người dùng đã tắt component trong Unity, không phải xóa hoặc sửa source file.
- Prefab `reverse_stacker_amr.prefab` và URDF `stacker_amr.urdf`: đọc để xác định hierarchy, ArticulationBody và offset giữa hai frame; không ghi nhận sửa các file model này trong bài.
- Docker và Unity endpoint: sử dụng môi trường có sẵn để kết nối. Không quy các thay đổi khác trong `server.py`, `exceptions.py`, `entrypoint.sh` hoặc tài liệu setup cho milestone này nếu chưa có lịch sử xác nhận.

## Những thao tác cấu hình đã thực hiện hoặc đã hướng dẫn

| Thao tác | Trạng thái / ý nghĩa |
|---|---|
| Khởi động endpoint, kết nối Unity qua port 10000 | Người dùng xác nhận hoạt động. |
| Gắn `PlanarOdometryPublisher` và gán Robot Base | Đã thực hành; sửa từ wrapper `base_footprint` sang thân vật lý `base_link`. |
| Tắt component `RosTestBridge` | Người dùng xác nhận đã tắt để tránh xung đột điều khiển `/cmd_vel`. |
| RViz: Fixed Frame = `odom`, thêm TF và Odometry | Người dùng xác nhận mũi tên khớp chuyển động. |
| Build workspace bằng `catkin_make` | Đã chạy thành công trong container. |
| Đặt `/use_sim_time = true`, chạy lại Unity và controller | Đã hướng dẫn sau sửa clock; chưa có xác nhận hoàn tất thử nghiệm mới từ người dùng. |

Cấu hình Inspector là thao tác trong Unity; cần Save scene để giữ lại. Không coi hướng dẫn Save scene là bằng chứng đã lưu thay đổi vào file `.unity`. ROS parameter đặt thủ công cũng không phải sửa source file và cần kiểm tra lại sau khi ROS master khởi động lại.

## Quá trình thực hiện và kết quả

### 1. Xác minh nền tảng chuyển động

Kiểm tra container, ROS master và endpoint; người dùng thử W/A/D trong Unity và xác nhận xe tiến/quay bình thường. Bước này xác nhận controller và physics đủ để bắt đầu thu dữ liệu chuyển động, chưa chứng minh tự đi đến goal.

### 2. Tạo feedback vị trí và hướng

Thêm publisher để lấy ground-truth pose từ Unity làm **ideal simulated odometry**. Output gồm `/odom` chứa pose/twist và `/tf` chứa quan hệ `odom → base_footprint`. Gốc `odom` được chọn theo vị trí và hướng lúc bắt đầu Play; không phải vị trí cố định mặc định của toàn nhà kho.

### 3. Sửa lỗi odometry luôn bằng 0

Triệu chứng: xe di chuyển nhưng position luôn `[0, 0, 0]`. Kiểm tra prefab thấy `base_footprint` là wrapper không có ArticulationBody, trong khi `base_link` mới là thân vật lý. Sửa reference nguồn pose sang `base_link`, giữ tên ROS frame đầu ra là `base_footprint` vì script dùng pose phẳng của thân xe.

Sau sửa, người dùng cung cấp TF có translation `[2.470, -0.852, 0.000]` và yaw khoảng `−113.743°`; sau đó xác nhận mũi tên RViz di chuyển/quay đúng theo xe. Các số này là ví dụ pose quan sát được, không phải kết quả tới đích B.

### 4. Triển khai controller đi đến điểm

Vòng điều khiển: đọc `/odom` → tính khoảng cách và góc đến goal → gửi `/cmd_vel` → nhận pose mới. Goal đầu tiên `(2, 0)` trong `odom`; tolerance 0,2 m. Thuật toán chỉ chạy trên vùng trống, không có obstacle avoidance, không yêu cầu orientation cuối tại goal.

Đã build thành công và chạy 4 test offline thành công. Test hội tụ dùng mô hình động học lý tưởng nên không thay thế kiểm chứng physics, độ trễ và độ trượt trong Unity.

### 5. Chẩn đoán và sửa lỗi clock

Lần chạy controller thực tế đã nhận `/odom` nhưng dừng với `Odometry stale or clocks disagree`. Đo được luồng odometry khoảng 21 Hz và tuổi timestamp âm khoảng 0,58–0,94 giây. Nguyên nhân xác nhận là nguồn thời gian của publisher và ROS không đồng bộ; chưa tách riêng mức đóng góp của clock hệ điều hành và cách publisher cũ cộng thời gian.

Đổi publisher sang Unity simulation time, phát `/clock` và dùng cùng timestamp cho odometry/TF; ROS cần `/use_sim_time = true`. Giữ kiểm tra timeout thay vì tăng ngưỡng để bỏ qua lỗi. Đã bổ sung log chi tiết và cập nhật hướng dẫn. Chưa có kết quả chạy A → B sau thay đổi này.

## Bảng bằng chứng cho report

| Hạng mục | Bằng chứng hiện có | Kết luận |
|---|---|---|
| Unity–ROS và manual driving | Xác nhận trực tiếp của người dùng | Đạt bước kiểm tra nền tảng |
| Pose và TF | Log translation/yaw, xác nhận RViz đúng | Đạt bước quan sát chuyển động trước khi đổi clock |
| Build ROS package | `catkin_make` kết thúc thành công | Node đã được đăng ký trong workspace |
| Toán điều khiển | `Ran 4 tests ... OK` | Pass các test offline đã triển khai |
| Chạy controller trong Unity | Log dừng vì clock lệch | Chưa đạt thử nghiệm tới B |
| Sửa clock | Code và tài liệu đã cập nhật | Chờ kiểm chứng runtime với `/clock` |
| Xe đến B và dừng | Chưa có pose cuối/video/xác nhận | Chưa hoàn thành milestone |

## Nội dung có thể dùng trong report

Trong milestone 1, hệ thống đã được bổ sung cơ chế xuất ideal simulated odometry và TF từ Unity sang ROS1, cho phép quan sát vị trí và hướng của robot trong RViz. Controller đi đến điểm được xây dựng theo nguyên tắc closed-loop control, sử dụng odometry làm feedback và gửi yêu cầu vận tốc qua `/cmd_vel`. Quá trình thực hành đã phát hiện và xử lý hai vấn đề tích hợp: chọn nhầm object nguồn pose trong Unity hierarchy và thiếu đồng bộ thời gian giữa publisher với ROS. Package đã build thành công, các test toán điều khiển đã pass; thử nghiệm xác nhận robot đến đích sau sửa clock vẫn cần thực hiện.

## Việc còn lại để kết thúc milestone

1. Xác nhận `/clock` hoạt động và ROS sử dụng simulation time.
2. Chạy lại goal `(2, 0)`; ghi nhận log khoảng cách giảm và pose cuối.
3. Xác nhận xe thực tế dừng và sai số vị trí sau dừng không quá 0,2 m.
4. Lưu screenshot/video Unity và RViz, log `ARRIVED`, pose cuối để làm bằng chứng report.
5. Nên thử thêm goal lệch như `(2, 1)` để đánh giá cả quay và tiến; ghi kết quả thực đo, không suy ra từ test offline.

`/scan`, mapping/localization, costmaps và tránh vật cản bằng `move_base` chưa được triển khai trong phần thực hành này; đó là bước mở rộng tiếp theo.
