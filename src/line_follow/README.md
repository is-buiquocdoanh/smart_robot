# Line follow & Joystick direction helper

Tài liệu ngắn cho phần line-follow mới (sensor RS485 -> topic) và tiện ích chuyển nút joystick -> chế độ đi tiến/lùi.

Hai node chính nằm trong package `line_follow` và `robot_joy`:

- `rs485_dual_sensor_pub` (package `line_follow`)
  - Đọc 2 cảm biến RS485 (16 kênh mỗi cảm biến) và publish:
    - `/sensor1/analog16` (std_msgs/UInt16MultiArray)
    - `/sensor2/analog16` (std_msgs/UInt16MultiArray)

- `line_follower_cmdvel_mag` (package `line_follow`)
  - Subscribe 2 topic cảm biến trên, tính lỗi vị trí line và publish `geometry_msgs/Twist` tới `/cmd_vel_mag`.
  - Chỉ sử dụng 1 cảm biến tại một thời điểm: chế độ `forward` dùng `sensor1`, chế độ `backward` dùng `sensor2`.
  - Lắng nghe topic `/line_follow/direction` (std_msgs/String) để đổi chế độ runtime. Giá trị hợp lệ: `forward` hoặc `backward`.
  - Publish thêm `/line_error` (std_msgs/Float32) để debug.

- `joy_to_direction` (package `robot_joy`)
  - Tiện ích: subscribe `/joy` (sensor_msgs/Joy) và publish `/line_follow/direction` (std_msgs/String) khi phát hiện nhấn nút (rising edge).
  - Params (có thể config runtime): `forward_button`, `backward_button`, `topic` (mặc định `0`, `2`, `/line_follow/direction` trong file hiện tại).

-----------------------------------------

Hướng dẫn nhanh

1) Build (nếu bạn chưa build sau khi thay đổi code):

```bash
colcon build --packages-select line_follow robot_joy
source install/setup.bash
```

2) Chạy các node (ví dụ đơn giản):

# Nếu muốn chạy publisher RS485 (thực tế trên robot):
```bash
ros2 run line_follow rs485_dual_sensor_pub
```

# Chạy node chuyển từ /joy -> /line_follow/direction (nếu đã build robot_joy):
```bash
ros2 run robot_joy joy_to_direction
```

# Chạy line follower (sử dụng `/sensor1/analog16` hoặc `/sensor2/analog16` dựa trên direction):
```bash
ros2 run line_follow line_follower_cmdvel_mag
```

Lưu ý: bạn có thể chạy file Python trực tiếp để test nhanh (không cần build):

```bash
python3 src/line_follow/line_follow/line_follower_cmdvel_mag.py
python3 src/robot_joy/robot_joy/joy_to_direction.py
```

Thay đổi chế độ runtime (manual):

```bash
ros2 topic pub /line_follow/direction std_msgs/String "data: 'forward'" -1
ros2 topic pub /line_follow/direction std_msgs/String "data: 'backward'" -1
```

3) Remap /cmd_vel_mag -> /cmd_vel (nếu bạn muốn line follower điều khiển trực tiếp robot qua mux)

Bạn có thể remap khi chạy node:

```bash
ros2 run line_follow line_follower_cmdvel_mag --ros-args -r /cmd_vel_mag:=/cmd_vel
```

Hoặc remap trong launch file tương ứng (recommended khi tích hợp vào hệ thống lớn).

-----------------------------------------

Cấu hình nút joystick

- Mặc định `joy_to_direction` dùng chỉ số nút `forward_button` = `0` và `backward_button` = `2`. Tuy nhiên các tay cầm khác nhau mapping khác nhau.
- Xem index nút bằng cách echo topic `/joy`:

```bash
ros2 topic echo /joy
```

- Thay đổi tham số khi chạy node:

```bash
ros2 run robot_joy joy_to_direction --ros-args -p forward_button:=4 -p backward_button:=5
```

Hoặc set param runtime:

```bash
ros2 param set /joy_to_direction forward_button 4
ros2 param set /joy_to_direction backward_button 5
```

Gợi ý: nếu bạn muốn 1 nút toggle giữa forward/backward, mình có thể cập nhật `joy_to_direction` để hỗ trợ chế độ toggle.

-----------------------------------------

Thông số (parameters) trên `line_follower_cmdvel_mag` (có thể set bằng `--ros-args -p` hoặc `ros2 param set`):

- `linear_x` (double) — tốc độ tuyến tính mặc định (m/s). Mặc định: `0.15`.
- `k_ang` (double) — hệ số chuyển lỗi -> góc. Mặc định: `0.35`.
- `max_ang` (double) — giới hạn góc quay (rad/s). Mặc định: `1.2`.
- `threshold_sum` (double) — ngưỡng tổng tín hiệu để coi là phát hiện line.
- `lost_timeout` (double) — timeout (s) để coi sensor đã mất liên lạc.
- `reverse_sensor_1`, `reverse_sensor_2` (bool) — đảo dữ liệu sensor nếu lắp ngược.
- `direction` (string) — giá trị khởi tạo `forward` hoặc `backward`.

Ví dụ khởi chạy với param tùy chỉnh:

```bash
ros2 run line_follow line_follower_cmdvel_mag --ros-args -p linear_x:=0.2 -p direction:=backward
```

-----------------------------------------

Kiểm tra / Debug

- Echo các topic để kiểm tra dữ liệu:

```bash
ros2 topic echo /sensor1/analog16
ros2 topic echo /sensor2/analog16
ros2 topic echo /line_follow/direction
ros2 topic echo /line_error
ros2 topic echo /cmd_vel_mag
```

- Nếu không thấy `/sensor1/analog16` hoặc `/sensor2/analog16`, kiểm tra `rs485_dual_sensor_pub` đang chạy và cổng serial đúng.
- Nếu `line_follower_cmdvel_mag` publish linear=0 liên tục, có thể do sensor không phát hiện line (tổng tín hiệu < `threshold_sum`) hoặc timeout (`lost_timeout`) quá ngắn.

-----------------------------------------

Tiếp theo / Tùy chọn nâng cao

- Thêm toggle mode (nhấn 1 nút để chuyển qua lại) — mình có thể cập nhật `joy_to_direction` nhanh.
- Kết hợp vào launch (`robot_bringup/launch/control_robot.launch.py`) để tự khởi cùng hệ thống.
- Thêm service hoặc action để bật/tắt line-follow an toàn.

Nếu bạn muốn, mình sẽ:
- thêm chế độ toggle vào `joy_to_direction` (1 nút chuyển đổi),
- hoặc cập nhật launch để remap `/cmd_vel_mag` vào `twist_mux` tự động.

---

Tôi có thể sửa README này theo ý bạn (ví dụ thêm ảnh sơ đồ kết nối cảm biến, sơ đồ nút joystick, hoặc launch mẫu).
