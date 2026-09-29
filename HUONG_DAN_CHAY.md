# HƯỚNG DẪN CÀI ĐẶT VÀ CHẠY — ur3_llm_control

Áp dụng cho **Ubuntu 22.04** hoặc **Windows + WSL2 + Ubuntu 22.04**.
Làm lần lượt từ Bước 0 đến Bước 9. Mỗi khối lệnh có thể copy dán nguyên vào terminal.

Quy ước:
- Workspace: `~/ur3_ws`
- "Terminal mới" = mở một cửa sổ/tab terminal Ubuntu mới.
- Dòng bắt đầu bằng `#` là chú thích, không cần gõ.

---

## Bước 0 — (chỉ WSL2) chuẩn bị Windows

Nên dùng **Windows 11** (có sẵn WSLg để hiện cửa sổ Gazebo/RViz). Trong PowerShell (Admin):

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
```

Gazebo + MoveIt + RViz cần nhiều RAM. Tạo file `C:\Users\<tên bạn>\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
```

Rồi chạy `wsl --shutdown` trong PowerShell và mở lại Ubuntu 22.04.

Kiểm tra đúng phiên bản Ubuntu (phải là 22.04 / jammy):

```bash
lsb_release -a
```

---

## Bước 1 — Cài ROS 2 Humble

```bash
# 1.1 Locale UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 1.2 Thêm kho apt của ROS 2
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 1.3 Cài ROS 2 Humble + công cụ build (colcon, rosdep)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools
```

> Nếu máy đã có ROS 2 Humble từ trước: vẫn chạy `sudo apt update && sudo apt upgrade -y`.
> Các gói `ros-humble-*` phải cùng đợt cập nhật, lệch phiên bản giữa MoveIt và UR hay gây lỗi khó hiểu.

---

## Bước 2 — Cài UR, MoveIt 2, Gazebo, ros2_control

```bash
sudo apt install -y \
  ros-humble-ur \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-yaml python3-numpy python3-pytest git

# rosdep (nếu báo "already exists" thì bỏ qua lệnh init)
sudo rosdep init
rosdep update
```

> **Lưu ý quan trọng:** KHÔNG có gói `ros-humble-ur-simulation-gazebo` trên apt cho Humble
> (README cũ ghi sai chỗ này, nên lệnh cài bị lỗi `Unable to locate package`).
> Gói này sẽ được build từ source ở Bước 3.

---

## Bước 3 — Tạo workspace và build

```bash
mkdir -p ~/ur3_ws/src
cd ~/ur3_ws/src

# 3.1 Gói mô phỏng UR cho Gazebo Classic (nhánh humble, build từ source)
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git

# 3.2 Đưa package ur3_llm_control vào src
#   - Ubuntu: file tải về thường ở ~/Downloads
#   - WSL2  : file ở Windows, ví dụ /mnt/c/Users/<tên bạn>/Downloads
tar -xJf ~/Downloads/ur3_llm_control_fixed.tar.xz -C ~/ur3_ws/src
# (WSL2) tar -xJf /mnt/c/Users/<tên bạn>/Downloads/ur3_llm_control_fixed.tar.xz -C ~/ur3_ws/src

ls ~/ur3_ws/src
# phải thấy: Universal_Robots_ROS2_Gazebo_Simulation  ur3_llm_control

# 3.3 Cài phụ thuộc còn thiếu và build
cd ~/ur3_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Kết quả mong đợi: `Summary: 2 packages finished`.

```bash
# 3.4 Tự động source mỗi khi mở terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ur3_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 3.5 Kiểm tra ROS thấy đủ package
ros2 pkg prefix ur3_llm_control
ros2 pkg prefix ur_simulation_gazebo
ros2 pkg prefix ur_moveit_config
ros2 pkg executables ur3_llm_control
# phải có: check_scene, llm_robot_node, offline_cli, send_command
```

---

## Bước 4 — Cài và cấu hình 9Router với Gemini

### 4.1 Cài Node.js ≥ 18 (Ubuntu 22.04 mặc định chỉ có Node 12, không chạy được 9Router)

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash
source ~/.bashrc
nvm install 22
node -v          # v22.x
npm install -g 9router
```

### 4.2 Lấy API key Gemini (Google AI Studio)

Mở https://aistudio.google.com/app/apikey (đăng nhập tài khoản Google) → **Create API key** → copy
chuỗi bắt đầu bằng `AIza...`.

> Về gói "Gemini Plus" của bạn: gói trả phí cho **ứng dụng Gemini** (Google AI Plus/Pro) **không bao gồm
> quyền gọi API**. API key AI Studio là thứ riêng, có free tier, dư dùng cho bài lab này.
> 9Router có đường khác là provider **Gemini CLI** (đăng nhập OAuth bằng tài khoản Google, model
> `gc/...`), nhưng chính 9Router cảnh báo: phiên OAuth không được cấp phép để dùng qua router,
> tài khoản có thể bị hạn chế/khoá. Chỉ dùng nếu bạn chấp nhận rủi ro đó.

### 4.3 Chạy 9Router — Terminal 0 (để nguyên terminal này suốt buổi)

```bash
9router --no-browser
```

Mở trình duyệt (trên WSL2 thì mở bằng trình duyệt Windows): **http://localhost:20128**

1. Đăng nhập: mật khẩu mặc định `123456` → đổi mật khẩu khi được hỏi.
2. **Providers** → chọn **Gemini** → dán API key `AIza...` → lưu → bấm test (phải báo OK).
3. **Endpoint** → tạo / copy **API key của 9Router** (dạng `sk-...`). Đây là key để code gọi
   9Router, **không phải** key Google.

### 4.4 Khai báo key cho code — Terminal mới

```bash
echo 'export NINEROUTER_API_KEY="sk-DAN_KEY_9ROUTER_VAO_DAY"' >> ~/.bashrc
echo 'export NINEROUTER_MODEL="gemini/gemini-2.5-flash"' >> ~/.bashrc
source ~/.bashrc
```

### 4.5 Kiểm tra 9Router gọi được Gemini

```bash
# Danh sách model Gemini mà 9Router có
curl -s http://localhost:20128/v1/models | python3 -m json.tool | grep '"id": "gemini/'

# Gọi thử 1 câu
curl -s http://localhost:20128/v1/chat/completions \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $NINEROUTER_API_KEY" \
  -d '{"model":"gemini/gemini-2.5-flash","messages":[{"role":"user","content":"Reply with exactly: OK"}]}'
```

Kết quả đúng có dạng `{"choices":[{"message":{"content":"OK", ...`.
Nếu lỗi, xem bảng xử lý sự cố ở Bước 9.

Đổi model (không cần build lại): `export NINEROUTER_MODEL="gemini/gemini-2.5-pro"` rồi chạy lại node.

---

## Bước 5 — Kiểm tra thông tin sinh viên

```bash
nano ~/ur3_ws/src/ur3_llm_control/config/student_config.yaml
```

Sửa `student_name`, `student_id` cho đúng **của bạn** (hiện đang là `Le Trong Nghia`, `23020754`
→ P = 54 mod 6 = 0 → Zone A = đỏ, Zone B = vàng, Zone C = xanh). Lưu: `Ctrl+O`, `Enter`, `Ctrl+X`.
Không điền `api_key` vào file này (đã dùng biến môi trường ở Bước 4.4).

```bash
cd ~/ur3_ws && colcon build --symlink-install && source install/setup.bash
```

---

## Bước 6 — Kiểm tra offline (chưa cần Gazebo)

```bash
# 6.1 Toàn bộ kiểm tra tự động (68 test + tầm với + world + cấu hình)
~/ur3_ws/src/ur3_llm_control/tools/run_all_checks.sh
# cuối cùng phải in: ALL CHECKS PASSED

# 6.2 Mọi vị trí gắp/đặt đều với tới được
ros2 run ur3_llm_control check_scene

# 6.3 Gemini thật + robot giả lập (kiểm tra LLM hiểu lệnh trước khi bật Gazebo)
ros2 run ur3_llm_control offline_cli "Đưa khối màu đỏ vào vùng B."
ros2 run ur3_llm_control offline_cli "Hãy lấy khối màu vàng và đặt nó vào ô A."
ros2 run ur3_llm_control offline_cli "Move the blue cube to zone C."
ros2 run ur3_llm_control offline_cli "Arrange all objects according to my student ID."
ros2 run ur3_llm_control offline_cli "Move the green cube to zone A."     # phải bị TỪ CHỐI
```

Nếu 6.3 in `TASK SUCCESS` (và câu cuối in `TASK REJECTED`) thì phần LLM đã sẵn sàng.

---

## Bước 7 — Chạy mô phỏng đầy đủ (cách khuyến nghị: 2 terminal)

### Terminal 0 — 9Router (nếu chưa chạy)
```bash
9router --no-browser
```

### Terminal 1 — Gazebo + UR3e + MoveIt 2 + RViz
```bash
ros2 launch ur3_llm_control sim.launch.py
# Máy yếu / WSL2 chậm: tắt cửa sổ Gazebo, vẫn xem robot trong RViz
#   ros2 launch ur3_llm_control sim.launch.py gazebo_gui:=false
# Dùng UR3 thay vì UR3e:
#   ros2 launch ur3_llm_control sim.launch.py ur_type:=ur3
```

Đợi đến khi log hiện lần lượt:
```
[sim.launch] controllers up -> starting MoveIt 2
... You can start planning now!
```
và RViz hiện robot. Lần đầu có thể mất 1–2 phút.

(Tuỳ chọn) kiểm tra trong một terminal khác:
```bash
ros2 control list_controllers          # joint_state_broadcaster, joint_trajectory_controller: active
ros2 action list | grep move_action    # /move_action
```

### Terminal 2 — Node điều khiển bằng ngôn ngữ tự nhiên
```bash
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true
```

Node in thông tin sinh viên, model LLM, dựng planning scene (sàn, bàn, 3 khối), đưa robot về
`home()`, rồi hiện dấu nhắc `Command>`. Gõ lệnh:

```
Command> Put the red cube in zone B.
Command> state
Command> quit
```

Trong RViz, thêm display **MarkerArray** với topic `/llm_robot/markers` để thấy nhãn Zone A/B/C.

---

## Bước 8 — Kịch bản demo

Mỗi lần chạy lại `llm_robot_node`, các khối trong Gazebo được đặt lại vị trí ban đầu,
nên muốn "làm lại từ đầu" chỉ cần `quit` rồi chạy lại lệnh ở Terminal 2.

### 8.1 Mức cơ bản (mỗi lệnh một vật, nhiều cách diễn đạt)
```
Command> Please put the red cube in zone B.
Command> Đưa vật màu đỏ sang vùng B.
Command> Hãy lấy khối màu vàng và đặt nó vào ô A.
Command> Move the blue cube to zone C.
```
Terminal in đúng khuôn: `USER COMMAND` → `LLM PLAN` → `EXECUTION` (từng skill ... SUCCESS) → `TASK SUCCESS`.

### 8.2 Mức nâng cao (theo MSSV)
```
Command> Arrange all objects according to my student ID.
Command> Sắp xếp tất cả các khối theo mã sinh viên của tôi.
```
Để thấy xử lý vùng bị chiếm, chạy trước một lệnh làm "rối" rồi mới sắp xếp:
```
Command> Put the blue cube in zone A.
Command> Arrange all objects according to my student ID.
```
Terminal sẽ in thêm `EXECUTION PLAN (reordered / zone conflicts resolved)`.

### 8.3 Chứng minh validator chặn kế hoạch sai
```
Command> Move the green cube to zone A.
Command> Put the red cube in zone D.
Command> Rotate the shoulder joint to 90 degrees.
```
Kết quả: `TASK REJECTED (nothing was executed)`, robot đứng yên.

### 8.4 Các cách chạy khác
```bash
# Chạy đúng 1 lệnh rồi thoát
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true \
  -p command:="Đưa vật màu đỏ sang vùng B."

# Chỉ lập + kiểm tra kế hoạch, robot KHÔNG chạy
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true -p dry_run:=true

# Đổi model ngay khi chạy
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true \
  -p llm_model:=gemini/gemini-2.5-pro

# Gộp tất cả trong 1 launch (node nhận lệnh qua topic, không có dấu nhắc Command>)
ros2 launch ur3_llm_control llm_robot.launch.py
#   ... và ở terminal khác:
ros2 run ur3_llm_control send_command "Move the blue cube to zone C."
ros2 topic echo /llm_robot/result
```

### 8.5 Tắt hệ thống
`Ctrl+C` ở Terminal 2 rồi Terminal 1. Nếu Gazebo còn treo:
```bash
pkill -9 -f gzserver; pkill -9 -f gzclient
```

---

## Bước 9 — Xử lý sự cố

| Hiện tượng | Nguyên nhân / cách xử lý |
|---|---|
| `Unable to locate package ros-humble-ur-simulation-gazebo` | Đúng — gói này không có trên apt. Làm theo Bước 3.1 (clone source). |
| `colcon build` báo thiếu `package.xml` / `launch` | Bạn đang dùng bản cũ bị thiếu file. Giải nén lại `ur3_llm_control_fixed.tar.xz`. |
| `Package 'ur3_llm_control' not found` | Chưa `source ~/ur3_ws/install/setup.bash` trong terminal đó. |
| `npm: command not found` hoặc 9Router báo lỗi cú pháp JS | Node quá cũ. Làm lại Bước 4.1 (`node -v` phải ≥ 18). |
| `no 9Router API key` | Chưa `export NINEROUTER_API_KEY=...` (Bước 4.4) hoặc chưa mở terminal mới sau khi sửa `~/.bashrc`. |
| `cannot reach 9Router at http://localhost:20128/...` | Terminal 0 chưa chạy `9router`. Nếu 9Router chạy trên **Windows** còn ROS chạy trong **WSL**: hoặc cài 9Router ngay trong WSL (khuyến nghị), hoặc bật `networkingMode=mirrored` trong `.wslconfig`. |
| `9Router HTTP 401` | Sai API key **9Router** (lấy ở trang Endpoint, không phải key Google). |
| `9Router HTTP 400/404 ... model` | Tên model sai. Xem danh sách bằng lệnh ở Bước 4.5 và đặt lại `NINEROUTER_MODEL`. |
| `9Router HTTP 429` / `quota` / `(reset after 2m)` | Hết hạn mức free tier hoặc 9Router tạm khoá kết nối sau lỗi. Đợi 1–2 phút, hoặc dùng `gemini/gemini-2.5-flash-lite`. |
| Gọi Gemini báo key không hợp lệ | Key AI Studio sai/bị thu hồi → tạo key mới, sửa ở Providers → Gemini trong dashboard. |
| `LLM PLAN: REJECTED BY VALIDATOR` với lệnh hợp lệ | Model trả sai định dạng. Thử `gemini/gemini-2.5-pro`; giữ `temperature: 0`. |
| `[setup] ERROR: not available: ['/move_action' ...]` | MoveIt chưa lên. Đợi `You can start planning now!` ở Terminal 1 rồi chạy lại node. |
| Spawner báo timeout / controller không active | Gazebo khởi động quá chậm. Tắt hết (8.5), chạy lại với `gazebo_gui:=false`. |
| Gazebo đen màn hình / crash ngay (WSL2, máy ảo) | `export LIBGL_ALWAYS_SOFTWARE=1` rồi chạy lại Terminal 1; hoặc `gazebo_gui:=false`. |
| Gazebo đứng ở "Preparing your world" rất lâu | Lần đầu Gazebo tải model. Đợi; nếu mạng chặn, tắt và chạy lại. |
| `PLANNING_FAILED` ở `pick` | Tăng `planning_time` trong `config/scene.yaml` (ví dụ 10.0), build lại; kiểm tra `ros2 run ur3_llm_control check_scene`. |
| `Cartesian path only xx% feasible` | Giảm `approach_height` hoặc dời vật gần robot hơn trong `scene.yaml`. |
| Khối không bám theo tool trong Gazebo | `ros2 service list \| grep set_entity_state` phải có kết quả (world phải do `sim.launch.py` sinh ra). |
| Robot trong RViz và Gazebo lệch nhau | Quên `-p use_sim_time:=true` khi chạy node. |

---

## Tóm tắt những gì đã sửa so với bản cũ

1. **File nén bị cắt cụt** (~28 MB, do nén kèm thư mục `tools/urdf_check/_deps` gồm cả repo git
   38 MB) → mất `package.xml` và `launch/`. Đã viết lại `package.xml`, `launch/sim.launch.py`,
   `launch/llm_robot.launch.py`, `tools/run_all_checks.sh`, và bỏ `_deps`, `__pycache__`.
2. **Hướng dẫn cài sai:** `ros-humble-ur-simulation-gazebo` không tồn tại trên apt → chuyển sang build source.
   Node.js của Ubuntu 22.04 quá cũ cho 9Router → dùng nvm.
3. **Thiếu sàn trong MoveIt:** bản cũ bỏ vật cản sàn vì tưởng MoveIt đã có link `ground_plane`;
   thực tế `move_group` dùng URDF không có link này → robot có thể lập quỹ đạo xuyên sàn.
   Đã thêm lại sàn ở z = −5 mm (không chạm đế robot) và test chống tái phát.
4. **Chuỗi khởi động chắc chắn hơn:** spawn robot → joint_state_broadcaster → joint_trajectory_controller
   → MoveIt, mỗi bước đợi bước trước (tránh timeout trên WSL2); tắt MoveIt Servo không dùng tới.
5. **LLM chuyển sang Gemini** (`gemini/gemini-2.5-flash`) qua 9Router; bỏ API key cứng khỏi file cấu hình;
   client bỏ qua proxy khi gọi localhost; thông báo lỗi 401/404/429 kèm hướng xử lý.
