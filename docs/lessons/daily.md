# Daily startup — Unity + ROS1 Noetic trong Docker

Quy trình cập nhật ngày 2026-09-25 cho bài **Unity publish /odom → ROS tính điều khiển → Unity nhận /cmd_vel**. Bổ sung xử lý workspace rỗng sau khi bật lại Docker tại [mục 11](#11-workspace-rỗng-sau-khi-bật-lại-docker--sự-cố-ngày-2026-09-25).

## 1. Có bắt buộc dùng WSL để kích hoạt Docker không?

**Không.** Docker CLI cũng chạy được từ PowerShell khi Docker Desktop đã hoạt động. `wsl -d Ubuntu-22.04` chỉ mở Ubuntu; nó không tự khởi động Docker Desktop, ROS master hoặc endpoint.

Ta ưu tiên Ubuntu WSL cho project này vì dùng được đường dẫn và lệnh Linux thống nhất. Compose hiện còn mount `/tmp/.X11-unix` và `/mnt/wslg` phục vụ hiển thị RViz qua WSLg, nên chạy từ môi trường khác cần kiểm tra lại các mount GUI này.

| Thành phần | Vai trò |
|---|---|
| Windows | Chạy Unity và Docker Desktop |
| Docker Desktop | Quản lý Docker engine để chạy container |
| Ubuntu WSL | Terminal Linux dùng quản lý container; môi trường WSLg phục vụ GUI |
| Container `ros1_amr_core` | Chạy ROS1 Noetic, master, endpoint, controller và RViz |

Mở Ubuntu chưa có nghĩa là đã vào container ROS. Không dùng shell `docker-desktop` làm terminal phát triển thông thường.

Workspace hiện tại:

```text
Windows: D:\Project\Robot\CollaborativeAMRStockerization\catkin_ws
WSL:     /mnt/d/Project/Robot/CollaborativeAMRStockerization/catkin_ws
Docker:  /catkin_ws
```

Đây là các cách truy cập cùng source qua mount. Đường dẫn cũ `D:\Project_Project` không còn đúng với workspace này.

## 2. Phân biệt roscore và Unity endpoint

**roscore** khởi động ROS master và các thành phần nền tảng của ROS1. Master giúp các node đăng ký và tìm nhau; Unity ROS-TCP Connector không kết nối trực tiếp tới master.

**ros_tcp_endpoint** là cầu nối giữa Unity TCP và ROS topics/services. Trong project, node này mang tên `/unity_endpoint`.

```text
Unity ROS-TCP Connector
    ↓ 127.0.0.1:10000 trên Windows
Docker port forwarding 10000:10000
    ↓
ros_tcp_endpoint lắng nghe 0.0.0.0:10000 trong container
    ↕
ROS topics/services và các node
```

ROS master thường dùng port **11311**. Unity dùng port **10000**, không đổi thành 11311.

**Container chạy ≠ master chạy ≠ endpoint chạy ≠ Unity đang publish dữ liệu.**

## 3. Terminal 1 — khởi động container và endpoint

**[Windows]** Mở Docker Desktop, chờ engine sẵn sàng.

**[PowerShell]**

```powershell
wsl -d Ubuntu-22.04
```

Mở Ubuntu; prompt chuyển sang dạng `user@machine:...$`.

**[Ubuntu WSL — terminal 1]**

```bash
cd /mnt/d/Project/Robot/CollaborativeAMRStockerization/docker
docker compose up -d
docker compose ps
```

`up -d` khởi động container ở background; `ps` phải cho thấy `ros1_amr_core` ở trạng thái Up.

**Kiểm tra mount trước khi source hoặc build — [Ubuntu WSL]:**

```bash
docker exec ros1_amr_core ls -la /catkin_ws
docker exec ros1_amr_core ls /catkin_ws/src
```

Phải thấy source của project, gồm các package như `amr_navigation`, `amr_description`. Workspace đã build còn có `build` và `devel`. Nếu `/catkin_ws` rỗng hoặc thiếu `src`, xử lý mount theo **mục 11** trước; container Up chưa chứng minh mount hoạt động.

**Cấu hình hiện tại chưa tự chạy master/endpoint:** Compose dùng `command: bash`; entrypoint chỉ source môi trường rồi chạy command. Vì vậy cần bước tiếp theo.

**[Ubuntu WSL — terminal 1]**

```bash
docker exec -it ros1_amr_core bash
```

Mở shell trong container đang chạy, thường có prompt `root@...:/catkin_ws#`.

**[Inside ROS Docker container — terminal 1]**

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
```

Hai lệnh source giúp shell tìm ROS và các package đã build. Đọc kết quả:

- Có `/unity_endpoint`: endpoint đã chạy, không chạy thêm bản thứ hai.
- Chỉ có `/rosout`: master đang chạy, endpoint chưa chạy.
- `Unable to communicate with master`: master chưa truy cập được.

Nếu chưa có endpoint, chạy:

```bash
roslaunch amr_navigation unity_bridge.launch
```

Lệnh khởi động endpoint. Với cấu hình master local hiện tại, roslaunch tự khởi động master nếu chưa có; nếu master đã chạy, nó sử dụng master đó. Không cần chạy thêm roscore.

Mong đợi log server lắng nghe **0.0.0.0:10000**. Giữ terminal này chạy. Ctrl+C tại đây sẽ dừng launch/endpoint.

Nếu `/catkin_ws/src` đã có đúng source nhưng workspace chưa build và thiếu `devel/setup.bash`, chạy trong container (nếu cả `src` cũng thiếu, làm mục 11 trước):

```bash
cd /catkin_ws
catkin_make
source /catkin_ws/devel/setup.bash
```

Mong đợi build không lỗi, rồi chạy launch bên trên. Không cần build lại mỗi ngày nếu không có thay đổi cần build.

## 4. Terminal 2 — kiểm tra ROS và chọn clock

Mở thêm Ubuntu WSL, giữ terminal endpoint.

**[Ubuntu WSL — terminal 2]**

```bash
docker exec -it ros1_amr_core bash
```

**[Inside ROS Docker container — terminal 2]**

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
rosparam set /use_sim_time true
rosparam get /use_sim_time
```

Cần thấy `/unity_endpoint` và parameter trả về `true`. ROS sẽ dùng simulation time do Unity phát trên `/clock`. Đặt parameter trước khi khởi động controller/RViz; kiểm tra lại sau khi ROS master khởi động lại vì thiết lập thủ công không tự lưu qua lần khởi động master mới.

## 5. Chuẩn bị Unity

**[Unity — chưa Play]** Mở scene đã cấu hình cho bài ROS:

| Component | Cấu hình |
|---|---|
| StackerController | Bật để nhận /cmd_vel và điều khiển bánh xe |
| PlanarOdometryPublisher | Bật; Robot Base = **base_link**, Publish Hz = 20 |
| StackerPointToPoint | Tắt khi chạy controller ROS |
| RosTestBridge | Tắt để tránh logic di chuyển thử nghiệm can thiệp |

Trong **Robotics → ROS Settings**:

```text
Protocol:         ROS1
ROS IP Address:   127.0.0.1
ROS Port:         10000
Connect On Start: bật
```

Save scene, nhấn Play. Kiểm tra Console và log endpoint. Nếu đã Play trước khi endpoint khởi động, có thể Stop rồi Play lại sau khi dừng controller.

Mỗi lượt Play reset gốc odom và simulation time. Không Stop/Play khi controller đang điều khiển xe.

## 6. Kiểm chứng kết nối bằng dữ liệu

**[Inside ROS Docker container — terminal 2]**

```bash
rostopic echo -n 1 /clock
```

Phải nhận một message có số giây tính từ đầu lượt Play. Lệnh tự kết thúc sau một message. Nếu chờ mãi, chưa có dữ liệu để dùng.

```bash
rostopic hz /odom
```

Phải thấy tần suất gần 20 Hz, tùy FPS. Ctrl+C chỉ dừng xem tần suất.

```bash
rosrun tf tf_echo odom base_footprint
```

Xem translation/rotation. Dùng tên ROS frame `base_footprint`, dù object Unity được đọc là `base_link`. Ctrl+C để dừng xem.

Chỉ chạy controller khi /clock và /odom đã có dữ liệu.

## 7. Chạy A → B bằng ROS

**[Unity]** Đường trống, xe đứng yên tại A, không nhấn WASD khi node điều khiển. B=(2,0) trong odom là 2 m theo hướng ban đầu của lượt Play, không phải 2 m trước pose hiện tại bất kỳ.

**[Inside ROS Docker container — terminal 2]**

```bash
roslaunch amr_navigation go_to_point.launch goal_x:=2.0 goal_y:=0.0
```

Lệnh bắt đầu điều khiển xe. Mong đợi Distance giảm và ARRIVED khi sai số trong 0,2 m. Kiểm tra xe thực tế dừng trong Unity. Ctrl+C kết thúc controller và gửi dừng; Stop Play dừng simulation nếu cần.

**[Inside ROS Docker container — terminal 3, tùy chọn]** Sau khi vào container và source ROS:

```bash
rviz
```

Chọn Fixed Frame = odom; thêm TF và Odometry topic /odom. Restart RViz nếu vừa chuyển từ wall time sang simulation time.

Chi tiết ở [bài 1](lesson-01-odom-tf.md) và [bài 2](lesson-02-go-to-point.md).

## 8. Connection đỏ hoặc không có dữ liệu

**[Inside ROS Docker container]**

```bash
rosnode list
rostopic info /clock
rostopic info /odom
netstat -ltn
```

| Kết quả | Cách hiểu và xử lý |
|---|---|
| Chỉ có /rosout | Master chạy nhưng thiếu endpoint; chạy unity_bridge.launch |
| Không liên lạc được master | Kiểm tra môi trường ROS, khởi động bridge như bước 3 |
| Port 10000 chưa LISTEN | Xem lỗi tại terminal chạy endpoint |
| /clock có Publishers: None | Chưa có publisher; topic xuất hiện do subscriber không chứng minh dữ liệu đang chảy |
| /odom là Unknown topic | Publisher chưa đăng ký; kiểm tra Unity connection và component trong scene |
| Có topic nhưng không có message | Kiểm tra Unity Play, Console, component có bị disable hoặc compile lỗi không |
| Position luôn 0 dù xe chạy | Robot Base cần gán base_link, không phải wrapper base_footprint |

Xem log endpoint tại terminal đang chạy roslaunch. **docker compose logs không phải nơi đáng tin để xem output của endpoint khởi động riêng qua docker exec**; nó chủ yếu hiển thị output tiến trình chính container. Ctrl+C khi đang xem logs chỉ dừng xem logs.

## 9. Lần khôi phục kết nối ngày 2026-09-20: thực tế đã làm gì?

Triệu chứng: Unity connection đỏ, hai lệnh /clock và /odom không nhận dữ liệu.

Kiểm tra cho thấy container và **roscore đã chạy sẵn**, nhưng chỉ có node /rosout, không có /unity_endpoint. Hai topic ban đầu đều Unknown; /use_sim_time đã là true. Unity log báo kết nối tới 127.0.0.1:10000 thất bại.

**Thao tác khắc phục là khởi động Unity endpoint, không phải khởi động lại roscore.** Từ shell host, lệnh tương đương thao tác đã dùng là:

**[PowerShell]**

```powershell
docker exec ros1_amr_core bash -lc 'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; roslaunch amr_navigation unity_bridge.launch'
```

Các lệnh bên trong dấu nháy chạy trong container. Đây là cách viết gộp của bước vào container rồi source và roslaunch; không chạy thêm lệnh này nếu endpoint đã hoạt động.

Sau khởi động, đã xác nhận:

- Node /unity_endpoint xuất hiện.
- Port 0.0.0.0:10000 ở trạng thái LISTEN.
- Ngay sau đó /clock chưa có publisher và /odom chưa có, nên đã hướng dẫn Play lại Unity và kiểm tra publisher.

Endpoint sẵn sàng chưa đủ để kết luận Unity đang publish. Cần kiểm chứng bằng message thực tế ở bước 6. Việc dùng PowerShell để khởi động endpoint lần này cũng cho thấy WSL không bắt buộc để gửi lệnh Docker; WSL là workflow ưu tiên của project.

## 10. Kết thúc buổi làm việc

1. Ctrl+C tại terminal controller nếu còn chạy, rồi Stop Play trong Unity.
2. Ctrl+C tại terminal endpoint và đóng RViz.
3. **[Inside ROS Docker container]** Gõ `exit` để trở về WSL.
4. **[Ubuntu WSL]**

```bash
cd /mnt/d/Project/Robot/CollaborativeAMRStockerization/docker
docker compose down
```

Lệnh dừng và gỡ container của Compose. Source trong bind mount vẫn ở Windows; không coi file chỉ lưu trong filesystem nội bộ container là dữ liệu bền vững.

Trình tự cần nhớ: **Docker Desktop → container → master/endpoint → chọn simulation clock → Unity Play → kiểm tra /clock và /odom → controller A → B**.

## 11. Workspace rỗng sau khi bật lại Docker — sự cố ngày 2026-09-25

### Triệu chứng và bằng chứng

Trong container, lệnh `source /catkin_ws/devel/setup.bash` báo:

```text
bash: /catkin_ws/devel/setup.bash: No such file or directory
```

Lần này đã kiểm tra và xác nhận:

- Container `ros1_amr_core` đang Up; `/opt/ros/noetic/setup.bash` vẫn tồn tại.
- `/catkin_ws` trong container rỗng, không có cả `src` và `devel`.
- Project trên Windows vẫn có `src`, `build`, `devel`; Ubuntu WSL đọc được `devel/setup.bash`.
- Docker inspect vẫn ghi bind mount từ `/mnt/d/Project/Robot/CollaborativeAMRStockerization/catkin_ws` tới `/catkin_ws`, nhưng dữ liệu không hiện trong container.
- Sau khi restart service từ Ubuntu WSL, container thấy lại các thư mục, source thành công và `rospack find amr_navigation` trả về `/catkin_ws/src/amr_navigation`.

**Đây là lỗi truy cập bind mount trong lần chạy đó, không phải source bị mất hoặc cần build lại.** Chưa xác định được nguyên nhân sâu hơn khiến mount rỗng; thời điểm khởi động Docker/WSL chỉ là khả năng, chưa có bằng chứng kết luận. Restart là cách khôi phục đã thành công, chưa phải bản sửa bảo đảm lỗi không tái diễn.

### Bước 1 — phân biệt thiếu build và thiếu mount

**[Inside ROS Docker container]**

```bash
ls -la /catkin_ws
ls /catkin_ws/src
ls -l /catkin_ws/devel/setup.bash
```

| Kết quả | Hướng xử lý |
|---|---|
| Có đúng source trong `src`, chỉ thiếu `devel/setup.bash` | Source `/opt/ros/noetic/setup.bash`, vào `/catkin_ws` và build theo mục 3 |
| `/catkin_ws` rỗng hoặc `src` không tồn tại | Kiểm tra mount theo các bước dưới, chưa chạy `catkin_make` |
| File setup có thật nhưng source báo lỗi khác | Đọc lỗi cụ thể, không mặc định đó là lỗi mount |

### Bước 2 — kiểm tra source từ Ubuntu WSL

Mở Ubuntu WSL ngoài container. Nếu đang ở container, gõ `exit`; nếu quay về PowerShell, dùng `wsl -d Ubuntu-22.04` để vào Ubuntu.

**[Ubuntu WSL]**

```bash
ls -ld /mnt/d/Project/Robot/CollaborativeAMRStockerization/catkin_ws/src
ls -l /mnt/d/Project/Robot/CollaborativeAMRStockerization/catkin_ws/devel/setup.bash
docker inspect ros1_amr_core --format '{{json .Mounts}}'
```

Trong trường hợp đã build như phiên này, hai lệnh `ls` phải tìm thấy đường dẫn. Inspect cần thể hiện đúng source project và destination `/catkin_ws`. Inspect chỉ xác nhận cấu hình mount, không thay thế việc đọc file thực tế.

Nếu WSL cũng không đọc được source, kiểm tra ổ D và đường dẫn project trước; không tạo một thư mục `src` trống để che lỗi. Không xóa `build`/`devel` hoặc copy source vào filesystem riêng của container để xử lý triệu chứng này.

### Bước 3 — nối lại mount bằng restart

Dừng controller nếu còn chạy rồi Stop Play trong Unity. Restart container sẽ ngắt shell `docker exec` và dừng các node ROS đang chạy bên trong.

**[Ubuntu WSL]**

```bash
cd /mnt/d/Project/Robot/CollaborativeAMRStockerization/docker
docker compose restart ros1_stack
docker exec ros1_amr_core ls -la /catkin_ws
docker exec -it ros1_amr_core bash
```

`ros1_stack` là tên service Compose; `ros1_amr_core` là tên container. Sau restart cần thấy lại `src`, `build`, `devel` của workspace đã build.

**[Inside ROS Docker container]**

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rospack find amr_navigation
```

Kết quả đã xác nhận ngày 2026-09-25:

```text
/catkin_ws/src/amr_navigation
```

Nếu vẫn thiếu source sau restart, dừng tại đây và ghi lại kết quả `ls` ở cả WSL/container cùng `.Mounts` để chẩn đoán tiếp; không lặp restart hoặc build trên thư mục rỗng.

### Bước 4 — khởi động lại buổi ROS

Khôi phục mount không tự khởi động lại endpoint hoặc các node đã chạy qua `docker exec`.

1. **[Inside ROS Docker container — terminal 1]** Chạy `roslaunch amr_navigation unity_bridge.launch` như mục 3 và giữ terminal mở.
2. **[Ubuntu WSL — terminal mới]** Vào lại container bằng `docker exec -it ros1_amr_core bash`.
3. **[Inside ROS Docker container — terminal mới]** Source hai file môi trường, chạy `rosparam set /use_sim_time true` như mục 4.
4. Nếu đang học scan, khởi động lại nguồn TF mount đã chọn: `roslaunch amr_navigation laser_tf.launch` hoặc nguồn `robot_state_publisher` đã dùng, không chạy cả hai cho cùng transform. Dùng terminal riêng nếu cần.
5. **[Unity]** Play; kiểm tra `/clock`, `/odom`, và `/scan` nếu đang dùng sensor. Sau đó mở lại RViz.
6. Chỉ chạy controller khi dữ liệu đã trở lại. Không cần build lại nếu source và file setup vẫn còn nguyên như lần khôi phục này.

## 12. RViz lỗi xcb sau restart — ghi chú ngày 2026-09-26

### Nguyên nhân và cách hiểu socket X11

RViz chạy trong container nhưng hiển thị cửa sổ trên Windows qua WSLg:

```text
RViz trong Docker -> /tmp/.X11-unix/X0 -> X server của WSLg -> cửa sổ Windows
```

`X0` là Unix socket do X server tạo, không phải file dữ liệu cố định. Socket có thể được tạo lại khi WSL/Windows khởi động lại. Với `DISPLAY=:0`, RViz cần kết nối tới display số 0.

Compose hiện có bind mount:

```yaml
- /tmp/.X11-unix:/tmp/.X11-unix:rw
```

Compose mount **cả thư mục**, trong đó có socket `X0`; nó không tự tạo X server hoặc socket. Có mount trong cấu hình chưa đủ: container phải thấy socket đang hoạt động.

Bằng chứng đã kiểm tra trong lần lỗi này:

- Ubuntu WSL có `X0` trong `/tmp/.X11-unix` và `/mnt/wslg/.X11-unix`.
- Trong container, `/tmp/.X11-unix` rỗng và `/mnt/wslg/.X11-unix` không tồn tại.
- Các thư viện phụ thuộc của Qt xcb đều được tìm thấy. Log debug xác nhận plugin được load, sau đó báo `qt.qpa.xcb: could not connect to display :0` rồi mới báo lỗi không khởi tạo được plugin.

**Nguyên nhân trực tiếp đã xác nhận: container không truy cập được socket display của WSLg.** Cài lại RViz/Qt không xử lý được mount rỗng này.

Vì sao hôm qua chạy được nhưng hôm nay lỗi? Hai khả năng là Docker tự khởi động container trước khi WSLg sẵn sàng (`restart: unless-stopped`), hoặc Compose được chạy từ môi trường khác khiến đường dẫn Linux được resolve tới thư mục khác. Chạy Compose từ PowerShell có thể khiến nguồn mount thuộc môi trường Docker Desktop thay vì Ubuntu WSL có `X0`; điều này phụ thuộc cách Docker Desktop xử lý đường dẫn, không phải mọi lần chạy từ PowerShell đều lỗi. Nếu thư mục nguồn bị thay thế trong quá trình khởi động, bind mount cũ cũng có thể vẫn trỏ tới thư mục cũ.

**Chưa xác nhận khả năng nào gây ra lần restart này.** PowerShell vẫn dùng được cho `docker exec`; điểm cần chú ý ở đây là nguồn bind mount khi tạo container.

### Cách khôi phục — thực hiện từ Ubuntu WSL

Đây là hướng khôi phục đề xuất; phiên chẩn đoán này chưa kiểm chứng RViz chạy thành công sau recreate.

1. **[Unity]** Dừng controller và Stop Play trước. Recreate sẽ thay container, ngắt các shell và node ROS đang chạy. File chỉ nằm trong filesystem riêng của container cần được lưu ra ngoài trước; workspace bind mount vẫn nằm trên Windows.

2. **[PowerShell — nếu chưa mở Ubuntu WSL]**

   ```powershell
   wsl -d Ubuntu-22.04
   ```

3. **[Ubuntu WSL]** Kiểm tra display đã sẵn sàng:

   ```bash
   echo "$DISPLAY"
   ls -la /tmp/.X11-unix
   ```

   Mong đợi `:0` và socket `X0`. Nếu chưa có `X0`, cần xử lý WSLg trước; recreate container lúc này chưa giải quyết được nguồn socket bị thiếu.

4. **[Ubuntu WSL]** Tạo lại container để thiết lập mount mới:

   ```bash
   cd /mnt/d/Project/Robot/CollaborativeAMRStockerization/docker
   docker compose up -d --force-recreate ros1_stack
   docker exec ros1_amr_core ls -la /tmp/.X11-unix
   docker exec -it ros1_amr_core bash
   ```

   Kết quả kiểm tra trong container phải có `X0`. `docker compose up -d` thông thường có thể dùng lại container cũ; `--force-recreate` buộc tạo lại. Nếu vẫn rỗng dù WSL có `X0`, ghi lại `.Mounts` bằng lệnh inspect ở mục 11 để kiểm tra nguồn mount, không lặp lại cài Qt.

5. **[Inside ROS Docker container]** Source môi trường:

   ```bash
   source /opt/ros/noetic/setup.bash
   source /catkin_ws/devel/setup.bash
   echo "$DISPLAY"
   ```

   Mong đợi `:0`. Khởi động lại master/endpoint và các node theo mục 3–6 hoặc mục 11, bước 4. Sau đó mở terminal container riêng, source môi trường và chạy:

   ```bash
   rviz
   ```

   Thành công khi cửa sổ RViz xuất hiện và không còn lỗi kết nối display. Nếu socket đã có nhưng vẫn lỗi, lấy log bằng `QT_DEBUG_PLUGINS=1 rviz` trong container để chẩn đoán tiếp.

**Thói quen startup:** mở Docker Desktop, vào Ubuntu WSL, kiểm tra `X0`, rồi chạy Compose từ thư mục project trong WSL. Đây là workflow giúp tránh nhầm nguồn mount; vẫn cần kiểm tra socket bên trong container nếu lỗi tái diễn.

## 13. Robot không quay tại chỗ ở tốc độ thấp — ghi chú ngày 2026-09-26

### Triệu chứng và kết luận

Sau khi sửa lỗi DWA không tạo được lệnh, robot vẫn gần đứng yên hoặc lắc nhẹ trái/phải. Khi thử trực tiếp với `/move_base` đã dừng, lệnh quay `angular.z = 0.1` tạo đúng target hai bánh `-0.293/+0.293 rad/s`, nhưng vận tốc bánh thực rất nhỏ. Cả tiến và lùi `0.022 m/s` cũng không bám target; tăng lệnh hoặc vừa tiến vừa quay thì robot chuyển động rõ hơn.

Khi cố định thân và tắt collider bánh để thử quay tự do, hai bánh bám target chính xác. Bài thử tự động trên bản sao scene warehouse tái hiện được lỗi mà không cần ROS/DWA. Tăng độ chính xác solver và giảm timestep giúp bánh đáp ứng lệnh nhỏ dưới tải.

**Kết luận có bằng chứng:** cấu hình giải physics/tiếp xúc mặc định chưa phù hợp với mô hình robot ở tốc độ thấp. Chưa tách riêng được mọi cơ chế bên trong PhysX; không kết luận riêng ma sát caster hoặc số mặt mesh bánh là nguyên nhân duy nhất. Thử tắt sleep và tăng damping riêng lẻ không cho thấy cải thiện đủ để giữ lại.

### Các file đã sửa

| File | Thay đổi | Phạm vi |
|---|---|---|
| [StackerController.cs](../amr_ware_house/Assets/StackerController.cs) | Thêm `solverIterations = 32`, `solverVelocityIterations = 8`; `ConfigureJoints()` áp dụng cho mọi `ArticulationBody` thuộc robot. | Chỉ robot này; cấu hình trước dùng mặc định `6/1`. |
| [TimeManager.asset](../amr_ware_house/ProjectSettings/TimeManager.asset) | `Fixed Timestep: 0.02` → `0.01` giây. | Toàn project; tăng số bước physics từ 50 lên 100 mỗi giây simulation. |
| [RobotPhysicsDiagnostics.cs](../amr_ware_house/Assets/Editor/RobotPhysicsDiagnostics.cs) | Thêm công cụ chụp cấu hình physics bằng menu `Tools > Robot > Save Physics Snapshot` và kiểm tra cấu hình trong batch mode. | Công cụ chẩn đoán Editor, không phải phần sửa điều khiển bánh. |

Không đổi DWA, hình dạng collider bánh, damping hoặc force limit để áp dụng bản sửa này. Các thử nghiệm nằm riêng trong `.diagnostics/UnityPhysicsLab`; xem [báo cáo chẩn đoán](physics-findings.md) và [kết quả xác nhận CSV](../.diagnostics/UnityPhysicsLab/Results/summary.csv).

### Kết quả kiểm tra

Đã chạy 32 kịch bản xác nhận trên bản sao physics của scene. Bảng dưới là vận tốc thực trung bình trong bài thử tự động, không phải số đo mới từ phiên Play của người dùng:

| Lệnh yêu cầu | Trước: solver `6/1`, timestep `0.02 s` | Sau: solver `32/8`, timestep `0.01 s` |
|---|---:|---:|
| Tiến `0.022 m/s` | `0.00046 m/s` | `0.0213 m/s` |
| Quay `+0.1 rad/s` | Gần 0 | `+0.0849 rad/s` |
| Quay `-0.1 rad/s` | Gần 0 | `-0.0850 rad/s` |
| Quay `+0.5 rad/s` | `0.310 rad/s` | `0.474 rad/s` |

Unity biên dịch thành công; kiểm tra batch xác nhận mọi articulation của robot nhận solver `32/8` và timestep là `0.01 s`. **Người dùng đã thử trong Unity và xác nhận robot quay tại chỗ được, rất mượt.** Xác nhận này là quan sát chuyển động; chưa có số đo odom mới để khẳng định vận tốc thực đúng tuyệt đối.

### Áp dụng và bước tiếp theo

- **[Unity]** Stop rồi Play lại để `Start()` gọi `ConfigureJoints()`. Kiểm tra mục `Articulation Solver` trên Stacker Controller là `32/8`, và `Project Settings > Time > Fixed Timestep` là `0.01`.
- Khi thử `/cmd_vel` trực tiếp, dừng launch navigation và kiểm tra `rostopic info /cmd_vel` không còn `/move_base` phát lệnh. `warehouse_navigation.launch` tự khởi động node này; nút `2D Nav Goal` chỉ gửi goal, không khởi động node.
- Dừng publisher chưa bảo đảm bánh dừng vì controller giữ target cuối. **[Inside ROS Docker container]** Gửi zero sau mỗi lượt thử: `rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"`.
- Còn khoảng 15% sai số quay ở lệnh `0.1` trong bài thử tự động; cấu hình mới tốn thêm CPU. Cần kiểm tra tốc độ simulation thực và thử goal navigation gần, vùng trống. Chưa đánh dấu hoàn tất navigation/né vật cản chỉ từ việc robot quay mượt.
