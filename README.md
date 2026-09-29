# ur3_llm_control — Điều khiển UR3/UR3e bằng ngôn ngữ tự nhiên
ROS 2 Humble · MoveIt 2 · Gazebo Classic · 9Router (Gemini)

> **Cách cài và chạy từng lệnh một: xem [`HUONG_DAN_CHAY.md`](HUONG_DAN_CHAY.md).**
> README này giải thích kiến trúc, luồng hoạt động và các quyết định thiết kế (dùng khi bảo vệ bài).

```
Natural Language Command ──► LLM Planner (9Router → Gemini) ──► JSON Plan ──► Plan Validator
      ──► Skill Executor ──► Robot Skills ──► MoveIt 2 ──► UR3/UR3e (Gazebo)
```

LLM **chỉ** chọn và sắp xếp skill trong danh sách trắng (`home`, `pick`, `place`,
`move_above`, `move_to_zone`). LLM không bao giờ thấy tên khớp, giá trị khớp hay
quỹ đạo; mọi hình học (độ cao tiếp cận, hướng tool, toạ độ) nằm trong code và
`config/scene.yaml`; mọi chuyển động do MoveIt 2 lập kế hoạch và thực thi.

---

## 1. Cấu trúc package

```
ur3_llm_control/
├── package.xml
├── setup.py / setup.cfg / resource/
├── HUONG_DAN_CHAY.md          hướng dẫn cài đặt + chạy từng lệnh
├── launch/
│   ├── sim.launch.py          Gazebo (world sinh từ scene.yaml) + UR3/UR3e + ros2_control + MoveIt 2 + RViz
│   └── llm_robot.launch.py    sim.launch.py + llm_robot_node (nhận lệnh qua topic)
├── config/
│   ├── scene.yaml             bàn, 3 khối, 3 vùng, 2 ô tạm, home, tham số chuyển động
│   └── student_config.yaml    họ tên, MSSV, cấu hình 9Router (URL, model Gemini)
├── ur3_llm_control/
│   ├── llm_planner.py         prompt + client OpenAI-compatible (9Router) + retry có phản hồi lỗi
│   ├── task_validator.py      cổng an toàn: cú pháp, whitelist, object/zone, ngữ nghĩa
│   ├── skill_executor.py      sắp xếp lại / giải xung đột vùng, chạy từng skill, dừng khi lỗi
│   ├── robot_skills.py        home, pick, place, move_above, move_to_zone, open/close_gripper
│   ├── moveit_backend.py      MoveIt 2: /move_action, /compute_cartesian_path, /execute_trajectory,
│   │                          /apply_planning_scene; Gazebo: /gazebo/set_entity_state
│   ├── skill_spec.py          danh sách skill + mã trạng thái (dùng chung cho prompt/validator/executor)
│   ├── scene_model.py         đọc scene.yaml + trạng thái thế giới (vật nào ở đâu, đang cầm gì)
│   ├── student_task.py        P = XX mod 6 và bảng gán màu → vùng
│   ├── pipeline.py            in báo cáo USER COMMAND / LLM PLAN / EXECUTION / TASK ...
│   ├── llm_robot_node.py      node ROS 2 (terminal tương tác + topic)
│   ├── send_command.py        gửi 1 lệnh lên /llm_robot/command
│   ├── world_gen.py           sinh file world Gazebo từ scene.yaml
│   ├── check_scene.py         kiểm tra offline: mọi pose gắp/đặt có với tới được không
│   ├── offline_cli.py         thử LLM + validator + executor KHÔNG cần ROS (robot giả lập IK)
│   ├── fake_backend.py        backend giả lập (dùng cho test / offline_cli)
│   └── ur_kinematics.py       FK/IK số UR3/UR3e (chỉ để kiểm tra offline, không điều khiển robot)
├── test/test_core.py          68 test pytest (không cần ROS)
└── tools/
    ├── run_all_checks.sh      chạy toàn bộ kiểm tra offline
    └── urdf_check/            đối chiếu ur_kinematics.py với URDF thật dựng bằng xacro
```

Phụ thuộc ngoài package (xem `HUONG_DAN_CHAY.md`): `ur_simulation_gazebo` **không có bản apt
cho Humble**, phải clone nhánh `humble` vào cùng workspace (package chỉ dùng file
`config/ur_controllers.yaml` của nó).

## 2. Chạy nhanh (khi đã cài xong)

```bash
# Terminal 0: 9Router
9router --no-browser
# Terminal 1: mô phỏng
source /opt/ros/humble/setup.bash && source ~/ur3_ws/install/setup.bash
ros2 launch ur3_llm_control sim.launch.py
# Terminal 2: node LLM (đợi RViz hiện robot)
source /opt/ros/humble/setup.bash && source ~/ur3_ws/install/setup.bash
ros2 run ur3_llm_control llm_robot_node --ros-args -p use_sim_time:=true
Command> Put the red cube in zone B.
```

Topic: `/llm_robot/command` (vào), `/llm_robot/plan`, `/llm_robot/result` (ra, JSON),
`/llm_robot/markers` (nhãn Zone A/B/C trong RViz — thêm display MarkerArray).

## 3. LLM: Gemini qua 9Router

- 9Router là proxy OpenAI-compatible chạy local (`http://localhost:20128/v1`). Code gọi
  `POST /v1/chat/completions` với `model` dạng `<alias provider>/<model>`.
- Mặc định: **`gemini/gemini-2.5-flash`** — provider *Gemini* của 9Router, xác thực bằng
  **API key Google AI Studio** (miễn phí, lấy ở https://aistudio.google.com/app/apikey).
- Gói **Google AI Plus / Gemini app không kèm quyền gọi API**. Nếu muốn dùng chính tài khoản
  Google qua đăng nhập OAuth thì 9Router có provider *Gemini CLI* (`gc/gemini-2.5-flash`),
  nhưng chính 9Router gắn cảnh báo: phiên OAuth không được cấp phép cho router, tài khoản
  *có thể bị hạn chế hoặc khoá*. Bài này khuyến nghị dùng API key AI Studio.
- Có 3 cách chọn model (ưu tiên từ trên xuống): tham số node `-p llm_model:=...`,
  biến môi trường `NINEROUTER_MODEL`, `llm.model` trong `student_config.yaml`.
- API key của **9Router** (không phải key Google) đặt bằng `NINEROUTER_API_KEY`.
  Package không chứa key nào.
- Client bỏ qua `http_proxy` khi gọi `localhost` (tránh lỗi trên mạng trường/công ty) và
  in gợi ý cụ thể cho lỗi 401 / 404 model / 429 quota.

## 4. Kết quả terminal mẫu

Mức cơ bản:
```
USER COMMAND:
Put the red cube in zone B.

LLM PLAN:
pick(red_cube)
place(red_cube, zone_b)
home()

EXECUTION:
pick(red_cube) ................ SUCCESS
place(red_cube, zone_b) ....... SUCCESS
home() ........................ SUCCESS

TASK SUCCESS
```

Mức nâng cao — MSSV 23020754 → P = 54 mod 6 = 0 → A = Red, B = Yellow, C = Blue.
Giả sử trước đó đã chạy "put the blue cube in zone A" (xanh đang chiếm A):
```
LLM PLAN:
pick(red_cube)
place(red_cube, zone_a)
pick(yellow_cube)
place(yellow_cube, zone_b)
pick(blue_cube)
place(blue_cube, zone_c)
home()

EXECUTION PLAN (reordered / zone conflicts resolved):
pick(yellow_cube)
place(yellow_cube, zone_b)
pick(blue_cube)            <- chuyển xanh sang C trước vì A đang bị xanh chiếm
place(blue_cube, zone_c)
pick(red_cube)
place(red_cube, zone_a)
home()
```
Khi hoán đổi thật sự (vòng lặp, ví dụ đỏ ở B và vàng ở A) thì không thể chỉ sắp xếp lại;
executor tự chèn bước `[auto]` đưa một khối sang ô tạm `buffer_1` rồi hoàn thành:
```
pick(yellow_cube)  [auto: clear target zone]
place(yellow_cube, buffer_1)  [auto: clear target zone]
pick(red_cube)
place(red_cube, zone_a)
pick(yellow_cube)
place(yellow_cube, zone_b)
home()
```

Kế hoạch sai bị từ chối, robot không chuyển động:
```
[validator] attempt 1/2 REJECTED: step 1: INVALID_OBJECT "green_cube" (valid: red_cube, yellow_cube, blue_cube)
LLM PLAN: REJECTED BY VALIDATOR
  - LLM returned no plan: green_cube does not exist
TASK REJECTED (nothing was executed)
```

## 5. Luồng hoạt động và các lớp an toàn

1. **LLM Planner** (`llm_planner.py`): system prompt gồm danh sách skill + tham số,
   object (kèm màu), zone, bảng gán cá nhân theo MSSV (P do code tính, LLM không phải làm
   phép mod), luật định dạng, và yêu cầu trả `{"plan": [], "error": ...}` khi lệnh không
   thực hiện được. User prompt gồm trạng thái hiện tại + câu lệnh. Không có câu lệnh nào
   bị hard-code: mọi cách diễn đạt (Việt/Anh) đều do LLM hiểu.
2. **Validator** (`task_validator.py`) — không qua thì không chạy:
   cú pháp JSON → skill thuộc whitelist → đúng/đủ tham số, cấm khoá lạ
   (`joints`, `trajectory`, `position`, ... bị báo "low-level robot control is forbidden")
   → object/zone tồn tại (`INVALID_OBJECT`, `INVALID_ZONE`; ô tạm không cho LLM dùng)
   → mô phỏng ngữ nghĩa (không pick khi tay đang cầm, không place vật không cầm, không kết
   thúc khi còn cầm vật, không move_above vật đang cầm) → tự thêm `home()` nếu thiếu.
   Nếu bị từ chối, lỗi được gửi lại cho LLM **một** lần, kèm yêu cầu không được tự thay
   vật/vùng khác so với lệnh; lần 2 vẫn sai → TASK REJECTED.
3. **Executor** (`skill_executor.py`): sắp lại các cặp pick/place để tránh xung đột,
   bỏ qua vật đã đúng chỗ (`SKIPPED`), dùng ô tạm khi có vòng lặp, chạy tuần tự, dừng ở
   bước lỗi đầu tiên (các bước sau `NOT EXECUTED`).
4. **Robot skills** (`robot_skills.py`): mỗi skill kiểm tra đầu vào rồi gọi MoveIt, trả về
   `SUCCESS / FAILED / INVALID_OBJECT / INVALID_ZONE / INVALID_STATE / PLANNING_FAILED / EXECUTION_FAILED`.
   `pick` = move_above → hạ thẳng (Cartesian) → close_gripper → nâng thẳng;
   `place` = move_to_zone → hạ thẳng → open_gripper → nâng thẳng.
5. **MoveIt 2** (`moveit_backend.py`): OMPL cho di chuyển tự do (ràng buộc tool hướng
   xuống, yaw tự do), Cartesian path cho đoạn lên/xuống với `avoid_collisions=true`.
   Planning scene có **sàn, bàn, 3 khối**; khối đang gắp được *attach* vào `tool0`. Vì thế
   MoveIt đảm bảo **không vượt joint limit** (giới hạn URDF), **không tự va chạm**
   (SRDF) và **không va chạm môi trường**, kể cả khối đang mang theo.

## 6. Quyết định thiết kế và giới hạn (nên nói rõ khi bảo vệ)

- **Gripper ảo (kiểu hút chân không).** Bộ UR Gazebo Humble không có gripper; thêm
  Robotiq vào Gazebo Classic tốn công và dễ lỗi vật lý. Ở đây "đóng gripper" = attach
  khối vào `tool0` trong MoveIt + khối trong Gazebo bám theo `tool0` qua
  `/gazebo/set_entity_state` (khối là link *kinematic*). Tool dừng cách mặt khối 8 mm.
  Đây là mô phỏng hành vi gắp, **không** mô phỏng lực kẹp/ma sát.
- **Bàn thấp 10 cm cạnh đế robot.** Robot trong URDF gắn cố định tại gốc `world`; đặt bàn
  thấp phía trước giữ cho mọi pose nằm gọn trong tầm với UR3 (~0.5 m) mà không phải dịch
  toạ độ giữa Gazebo và MoveIt.
- **Vị trí vật lấy từ `scene.yaml`** (được phép theo đề), và được cập nhật sau mỗi `place`.
  Khi khởi động, node đặt lại các khối trong Gazebo về vị trí ban đầu cho khớp trạng thái.
- **Ô tạm (`buffer_1`, `buffer_2`)** chỉ executor dùng; LLM không được phép gọi.
- **Sàn trong planning scene là bắt buộc.** Mô hình robot trong Gazebo có link
  `ground_plane` (vì xacro chạy với `sim_gazebo:=true`), nhưng `move_group` lấy
  `robot_description` từ `ur_moveit.launch.py`, vốn chạy xacro **không** có `sim_gazebo`
  → MoveIt không biết có sàn. Vì vậy `setup_scene()` thêm hộp `floor` có mặt trên ở
  z = −5 mm: đủ thấp để không chạm đế robot (điểm thấp nhất mesh va chạm của đế UR3 là
  −0.7 mm), đủ cao để chặn mọi quỹ đạo đâm xuống đất.
- **Khối đang gắp nhỏ hơn 8 mm (`attached_shrink`)**: khi vừa attach, đáy khối trùng mặt
  bàn (khe hở 0); MoveIt coi tiếp xúc là va chạm và bước nâng thẳng sẽ thất bại. Hộp va
  chạm của khối *đang gắp* được thu nhỏ để có khe 4 mm (> `position_tolerance` 3 mm).
  Khi thả, khối được trả lại kích thước thật.
- **Không dùng ACM** (AllowedCollisionMatrix): trong MoveIt Humble, ACM gửi kèm
  planning-scene diff sẽ **thay thế** ACM hiện có chứ không merge, làm mất các
  `disable_collisions` của SRDF.
- `check_scene` / `ur_kinematics.py` chỉ kiểm tra tầm với offline (IK số, tool hướng
  xuống, trong joint limit); va chạm thật do MoveIt kiểm tra lúc chạy. Động học này đã
  được đối chiếu với URDF thật (sai lệch 6.1e-10).

## 7. Kiểm thử

| Nhóm | Lệnh | Nội dung |
|---|---|---|
| Unit + integration (68 test) | `python3 -m pytest test -q` | bảng P, tầm với UR3/UR3e, world SDF, 22 kế hoạch sai phải bị từ chối, trích JSON (markdown/`<think>`), retry, sắp xếp lại/ô tạm/hoán đổi, pipeline cơ bản + nâng cao, lỗi planning dừng đúng chỗ, HTTP client với 9Router giả (200/401/không kết nối/bỏ qua proxy), sàn trong planning scene không chạm đế, tham số include MoveIt |
| Tổng hợp offline | `./tools/run_all_checks.sh` | cú pháp, pytest, check_scene, world XML, cấu hình LLM, (nếu có ROS) các package phụ thuộc |
| URDF thật | `python3 tools/urdf_check/verify_urdf_fk.py --from-ros` | so FK `world → tool0` với URDF dựng bằng xacro trên 500 cấu hình |
| LLM thật, robot giả | `ros2 run ur3_llm_control offline_cli "..."` | gọi 9Router/Gemini thật, robot là IK giả lập |

Đã kiểm tra thêm khi sửa lỗi: chạy `sim.launch.py` và `llm_robot.launch.py` bằng thư viện
`launch` của Humble (chỉ giả lập `launch_ros.Node`) với `ur_description`, `ur_moveit_config`,
`ur_simulation_gazebo` bản Humble thật: chuỗi khởi động Gazebo → spawn → controllers →
MoveIt chạy đúng thứ tự, xacro dựng được URDF, `move_group` nhận `joint_trajectory_controller`
làm mặc định, `ur_type:=ur5e` bị từ chối. Đã xác nhận trên 9Router 0.5.91 thật rằng
`gemini/gemini-2.5-flash` được định tuyến tới `generativelanguage.googleapis.com`.

**Giới hạn trung thực:** chưa chạy trên Gazebo/MoveIt thật (máy kiểm tra không cài được
ROS 2 Humble) và chưa gọi Gemini thật. Lần chạy đầu cần theo dõi: thời gian giải OMPL
(`planning_time`), độ bám của controller trong Gazebo, và việc model trả JSON đúng cho cả
câu tiếng Việt lẫn tiếng Anh (thử trước bằng `offline_cli`).
