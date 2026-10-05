# ROS Navigation — Definitions & Examples

Tài liệu ôn tập cho dự án robot vận chuyển hàng từ warehouse đến store. Giải thích bằng tiếng Việt, giữ nguyên thuật ngữ tiếng Anh.

Stack của dự án: **Unity trên Windows + ROS1 Noetic trong Docker + ROS Navigation Stack (`move_base`)**.

## 1. Nhìn tổng thể

```text
Unity: robot, physics, môi trường và cảm biến mô phỏng
    │
    ├── /odom: ước lượng chuyển động
    ├── /tf: transform thay đổi theo thời gian
    └── /scan: khoảng cách đo bởi LiDAR
    │
    ▼ qua ROS-TCP Connector và ros_tcp_endpoint
ROS: xử lý dữ liệu, localization và navigation
    │
    └── /cmd_vel: yêu cầu vận tốc
              │
              ▼
         Unity điều khiển robot
```

Fixed transform có thể do ROS publish trên `/tf_static`. Không bắt buộc Unity phải publish mọi transform.

**Unity** mô phỏng những gì xảy ra trong thế giới. **RViz** hiển thị dữ liệu mà ROS nhận được: frame, scan, odometry, map và đường đi.

## 2. Node, topic, message, publisher và subscriber

| Term | Definition | Example trong dự án |
|---|---|---|
| Node | Một chương trình đang chạy trong ROS | `move_base` xử lý navigation |
| Topic | Kênh truyền dữ liệu có tên | `/scan` là kênh truyền laser scan |
| Message | Cấu trúc của dữ liệu được truyền | `sensor_msgs/LaserScan` quy định các trường của laser scan |
| Publisher | Thành phần gửi message lên topic | Script Unity gửi khoảng cách đo được |
| Subscriber | Thành phần đăng ký nhận message từ topic | Thành phần navigation nhận scan |
| Broadcaster | Thành phần gửi transform vào hệ thống TF | Script gửi pose của thân xe so với `odom` |

Tên `/scan` hay `/odom` là tên topic, không phải đường dẫn file. Topic tồn tại không đảm bảo đang có dữ liệu truyền liên tục.

## 3. Coordinate frame — một anchor có hệ trục

**Coordinate frame** là hệ tọa độ gồm một điểm gốc và các trục X, Y, Z. Có thể hình dung nó như một **anchor có cả vị trí lẫn hướng**.

Frame có thể gắn với thân xe, camera, LiDAR hoặc một mốc cố định trên sàn. Frame là quy ước toán học; không cần một vật thể thật tại điểm gốc.

Ví dụ: câu “vật cản nằm ở phía trước 2 m” cần nói rõ phía trước của frame nào: thân xe hay camera?

| Frame | Ý nghĩa |
|---|---|
| `odom` | Mốc cố định cục bộ để theo dõi chuyển động liên tục; pose ước lượng trong frame này có thể bị drift |
| `base_footprint` | Mốc của robot trên mặt sàn, thường dùng cho navigation 2D |
| `base_link` | Mốc gắn với thân robot |
| `laser_link` | Mốc gắn với LiDAR |
| `camera_link` | Mốc gắn với camera |
| `map` | Mốc toàn cục của bản đồ, dùng khi có mapping/localization |

Các frame name thường viết không có dấu `/` đầu tên. Phân biệt **`odom` là frame** và **`/odom` là topic**.

## 4. Transform — frame này nằm và hướng như thế nào so với frame kia?

Một **transform** gồm:

- **Translation**: độ dịch chuyển theo các trục.
- **Rotation**: độ xoay giữa hai frame.

Trong sơ đồ `parent → child`, ta mô tả pose của child so với parent. Đây không phải mũi tên chỉ chiều truyền message.

Ví dụ `base_link → laser_link`: LiDAR nằm trước gốc thân xe 0,2 m, cao hơn 0,5 m và hướng cùng thân xe. Theo quy ước trục ROS cho thân robot (X tiến, Y trái, Z lên):

```text
translation: x = 0.2 m, y = 0 m, z = 0.5 m
rotation: không xoay tương đối
```

Các số này chỉ là ví dụ; triển khai phải dùng đúng vị trí lắp cảm biến trong model.

## 5. TF, /tf và /tf_static

**TF** là hệ thống quản lý và tra cứu quan hệ giữa các frame theo thời gian. **TF không trực tiếp đo vị trí**, không tự phát hiện Unity GameObject di chuyển.

Một broadcaster phải lấy hoặc tính pose rồi gửi transform. Nguồn dữ liệu có thể là odometry, kết quả localization hoặc pose trong Unity.

### /tf — quan hệ có thể thay đổi theo thời gian

Ví dụ `odom → base_link`, tạm bỏ qua `base_footprint` để dễ nhìn:

| Thời điểm | Pose của xe so với `odom` |
|---|---|
| Bắt đầu | x = 0 m, y = 0 m, hướng ban đầu |
| Chạy thẳng | x = 2 m, y = 0 m, hướng ban đầu |
| Quay tại chỗ | x = 2 m, y = 0 m, quay trái 90° |

Transform thay đổi khi **vị trí hoặc hướng** thay đổi. Quay tại chỗ cũng làm transform thay đổi.

Xe tạm dừng vẫn dùng `/tf`; không chuyển sang `/tf_static` vì hiện tại xe đứng yên.

### /tf_static — quan hệ được thiết kế cố định

Ví dụ camera được bắt vít trên thân xe: `base_link → camera_link` không đổi khi xe chạy hay quay.

**Static nghĩa là cố định so với parent frame, không có nghĩa là đứng yên trong nhà kho.**

Nếu camera nằm trên một khớp quay/nghiêng, quan hệ qua khớp đó thay đổi và cần dynamic transform trên `/tf`.

### Một frame có thể có quan hệ cố định với xe nhưng thay đổi so với sàn

```text
odom ──────────→ base_link ──────────→ laser_link
      thay đổi                cố định
        /tf                 /tf_static
```

ROS ghép hai transform để tính pose của LiDAR so với `odom`. Không cần publish thêm một cạnh trực tiếp `odom → laser_link`.

Trong TF tree, mỗi child có một parent. Mỗi quan hệ nên có một nguồn publish chịu trách nhiệm, tránh hai nguồn gửi kết quả mâu thuẫn.

## 6. Ví dụ ghép transform và scan

Giả sử mọi frame đang hướng cùng chiều, xe nằm tại x = 2 m trong `odom`:

```text
Xe so với odom:          x = 2.0 m
LiDAR so với xe:         x = 0.2 m
Vật cản trước LiDAR:     khoảng cách = 1.5 m

Vật cản trong odom:      x = 2.0 + 0.2 + 1.5 = 3.7 m
```

TF cung cấp vị trí và hướng của cảm biến; `/scan` cung cấp khoảng cách đến vật cản. Nếu xe quay, phải tính cả rotation, không thể chỉ cộng các giá trị X.

## 7. Odometry và /odom

**Odometry** là quá trình ước lượng chuyển động. Trên robot thật, một nguồn phổ biến là **wheel encoder** — cảm biến đo mức quay bánh xe.

Message `nav_msgs/Odometry` trên `/odom` chứa:

| Term | Definition |
|---|---|
| Position | Vị trí |
| Orientation | Hướng |
| Pose | Position và orientation |
| Linear velocity | Vận tốc tịnh tiến, thường dùng m/s |
| Angular velocity | Vận tốc quay, thường dùng rad/s |
| Twist | Linear velocity và angular velocity |
| Covariance | Thông tin mô tả độ không chắc chắn của ước lượng |

Ví dụ dùng frame tree của dự án:

```text
header.frame_id = "odom"
child_frame_id  = "base_footprint"
```

Pose được biểu diễn trong `odom`; twist được biểu diễn trong `base_footprint`.

**Publish `/odom` không tự động publish TF.** Ta cần cung cấp cả message odometry và transform tương ứng, với pose và timestamp nhất quán.

Trong bài đầu, có thể lấy pose mô phỏng thực tế từ Unity để tạo **ideal simulated odometry**. Đây là dữ liệu lý tưởng phục vụ kiểm tra kết nối và tọa độ, không phản ánh sai số encoder hay trượt bánh.

**Drift** là sai số tích lũy theo thời gian. Ví dụ bánh quay nhưng trượt trên sàn, odometry từ encoder có thể báo đã đi xa hơn thực tế.

## 8. LiDAR, raycast và /scan

**LiDAR** đo khoảng cách bằng laser. LiDAR và radar là hai loại cảm biến khác nhau; trong các ví dụ của dự án này ta dùng LiDAR.

**Raycast** là phép kiểm tra một tia có giao với collider nào trong Unity hay không. Ta dùng nhiều tia theo các hướng để mô phỏng planar LiDAR.

Topic `/scan` dùng message `sensor_msgs/LaserScan`:

| Trường | Ý nghĩa |
|---|---|
| `header.frame_id` | Frame của cảm biến, ví dụ `laser_link` |
| `header.stamp` | Thời điểm thu nhận tia đầu tiên |
| `angle_min`, `angle_max` | Góc bắt đầu và kết thúc, đơn vị radian |
| `angle_increment` | Khoảng góc giữa hai tia liên tiếp |
| `ranges` | Mảng khoảng cách đo được, đơn vị mét |
| `range_min`, `range_max` | Giới hạn khoảng cách đo |

Với tia thứ i: `angle[i] = angle_min + i × angle_increment`.

Theo quy ước LaserScan, góc 0 hướng theo +X của sensor frame; góc tăng quanh +Z, ngược chiều kim đồng hồ khi nhìn từ phía trên nếu Z hướng lên.

Ví dụ tia hướng thẳng phía trước có `ranges[i] = 1.5`: vật cản được đo cách cảm biến 1,5 m, không phải cách gốc nhà kho 1,5 m.

Khi triển khai, cần lọc collider của chính robot để tránh cảm biến liên tục đo trúng thân xe.

## 9. Các thuật ngữ navigation sẽ dùng tiếp

| Term | Definition | Example |
|---|---|---|
| Mapping | Xây dựng bản đồ môi trường | Tạo bản đồ các lối đi và kệ hàng |
| Localization | Ước lượng robot đang ở đâu trên bản đồ | Đối chiếu scan với tường để xác định pose |
| AMCL | Một node localization trên bản đồ có sẵn | Cung cấp transform `map → odom` trong cấu hình thông thường |
| Footprint | Hình chiếu vùng robot chiếm chỗ trên mặt sàn | Dùng kích thước xe và phần hàng nhô ra để kiểm tra va chạm |
| Costmap | Lưới biểu diễn chi phí di chuyển, vật cản và vùng cần tránh | Vùng gần kệ hàng có chi phí cao hơn vùng trống |
| Global planner | Tính đường đi tổng thể đến goal | Chọn lối đi từ kho đến cửa hàng |
| Local planner | Tính chuyển động ngắn hạn phù hợp với vật cản và giới hạn robot | Giảm tốc, quay để bám đường |
| `move_base` | Node điều phối navigation, sử dụng planners và costmaps | Nhận goal và tạo lệnh vận tốc |
| `/cmd_vel` | Topic chứa yêu cầu vận tốc, thường dùng `geometry_msgs/Twist` | `linear.x = 0.2` yêu cầu tiến 0,2 m/s |

Localization là thành phần riêng, không phải chức năng tự có bên trong `move_base`. Lệnh vận tốc là yêu cầu; vận tốc thực tế có thể khác do physics hoặc giới hạn điều khiển.

## 10. Những nhầm lẫn cần tránh

| Nhầm lẫn | Cách hiểu đúng |
|---|---|
| TF đo vị trí xe | TF truyền/tra cứu transform do thành phần khác cung cấp |
| Frame chỉ là một điểm | Frame có cả gốc và hướng các trục |
| Camera di chuyển trong nhà kho nên phải dùng dynamic transform với thân xe | Xét quan hệ camera so với thân xe; gắn cứng thì dùng static transform |
| Xe dừng thì đổi sang `/tf_static` | Xe có thể di chuyển nên quan hệ odometry vẫn là dynamic |
| `/tf_static` đo khoảng cách đến vật cản | Khoảng cách LiDAR nằm trong `/scan` |
| Gửi `/odom` là đủ để có TF | Cần publish transform tương ứng |
| Unity hierarchy tự xuất hiện trong ROS | Cần publisher/broadcaster hoặc robot description và node phù hợp |
| RViz mô phỏng physics | Unity mô phỏng physics; RViz hiển thị dữ liệu ROS |

## 11. Bước thực hành tiếp theo

Mục tiêu đầu: điều khiển xe bằng bàn phím trong Unity và quan sát chuyển động đúng trong RViz.

1. Kiểm tra ROS master và Unity endpoint đang chạy.
2. Xác định đúng GameObject tương ứng với `base_footprint` và `base_link`.
3. Chọn quy ước tọa độ và clock chung cho các message.
4. Publish `/odom` và dynamic TF `odom → base_footprint` từ cùng một pose.
5. Cung cấp các fixed transform của thân xe và sensor.
6. Kiểm tra trong RViz với Fixed Frame là `odom`.
7. Sau khi chuyển động hiển thị đúng, thêm `/scan` rồi mới đến navigation.

Unity và ROS có quy ước trục khác nhau: không chép trực tiếp tọa độ Unity vào message ROS. Timestamp của odometry, scan và TF phải cùng hệ thời gian. Các giá trị và frame trong tài liệu là hướng dẫn thiết kế, không có nghĩa các publisher đã được triển khai.

## 12. Tài liệu tham khảo

- [ROS REP 105 — Coordinate Frames](https://reps.openrobotics.org/rep-0105/)
- [ROS1 Odometry message](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/nav_msgs/msg/Odometry.msg)
- [ROS1 LaserScan message](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/LaserScan.msg)
- [Unity ROSGeometry — chuyển đổi tọa độ](https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/main/ROSGeometry.md)
- [Ghi chú navigation của dự án](navigation.md)
- [Ghi chú startup hằng ngày](daily.md)

Lưu ý: một số ghi chú cũ dùng đường dẫn `D:\Project_Project`. Workspace trong phiên làm việc hiện tại nằm ở `D:\Project\Robot`; cần đối chiếu đường dẫn trước khi chạy lệnh từ ghi chú cũ.

==================================================
TF explain: rosrun tf tf_echo odom base_footprint
At time 1789834540.614
- Translation: [2.470, -0.852, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.837, 0.547]
            in RPY (radian) [0.000, 0.000, -1.985]
            in RPY (degree) [0.000, 0.000, -113.743]
            
Dòng dễ hiểu hơn là RPY, viết tắt của Roll, Pitch, Yaw:
+ Roll: Nghiêng thân sang trái/phải
+ Pitch: Chúi hoặc ngẩng thân
+ Yaw: Quay hướng trên mặt sàn

Kết quả này nghĩa là: so với mốc lúc bắt đầu Play, xe đang ở X = 2,470 m, Y = −0,852 m và hướng quay khoảng −113,743°.
Translation — vị trí xe so với odom
Translation: [2.470, -0.852, 0.000]
                X       Y      Z

