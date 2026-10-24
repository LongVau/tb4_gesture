# Package ROS 2: `tb4_gesture`
**Bài tập 6 – Giao tiếp phi ngôn ngữ và điều khiển TurtleBot 4 bằng cử chỉ tay**

Package ROS 2 Humble độc lập nhận diện cử chỉ bàn tay từ webcam thông qua Google MediaPipe Gesture Recognizer và OpenCV, sau đó điều khiển TurtleBot 4 trong môi trường Ignition Gazebo thông qua topic `/cmd_vel` với cơ chế lọc nhiễu, xác nhận hành vi, dừng khẩn cấp và bảo vệ mất tín hiệu.

---

## 1. Yêu cầu hệ thống & Cài đặt phụ thuộc

### Môi trường
- **Hệ điều hành:** Ubuntu 22.04 LTS (Native hoặc WSL2).
- **ROS 2:** Humble Hawksbill.
- **Mô phỏng:** TurtleBot 4 Ignition Gazebo (`turtlebot4_ignition_bringup`).

### Cài đặt thư viện Python
```bash
pip install mediapipe opencv-python
```

### Tải Model nhận diện cử chỉ (nếu chưa có)
Model Google MediaPipe Gesture Recognizer `gesture_recognizer.task` đã được đặt sẵn trong thư mục `models/`. Nếu cần tải lại:
```bash
mkdir -p ~/ros2_ws/src/tb4_gesture/models
curl -sSL https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task -o ~/ros2_ws/src/tb4_gesture/models/gesture_recognizer.task
```

---

## 2. Biên dịch Package (Build)

Tại thư mục workspace ROS 2:
```bash
cd ~/ros2_ws
colcon build --packages-select tb4_gesture
source install/setup.bash
```

Chạy kiểm thử unit test:
```bash
PYTHONPATH=~/ros2_ws/src/tb4_gesture python3 -m pytest -p no:anyio ~/ros2_ws/src/tb4_gesture/test/ -v
```

---

## 3. Bảng ánh xạ cử chỉ & Hành vi Robot

| Cử chỉ tay | Ý nghĩa nhận diện | Hành vi Robot (qua `/cmd_vel`) |
| :--- | :--- | :--- |
| **Wave** (Vẫy tay / `ILoveYou`) | Chào | Chọn hành vi: Xoay trái – phải để chào lại |
| **Pointing_Up** (Giơ 1 ngón) | Chọn lệnh 1 | Chọn hành vi: Di chuyển tiến – lùi 4 lần |
| **Victory** (Giơ 2 ngón) | Chọn lệnh 2 | Chọn hành vi: Di chuyển theo quỹ đạo tròn |
| **Thumb_Up** (Giơ ngón cái) | **Xác nhận** | **Bắt đầu thực thi** hành vi đã chọn ở trên |
| **Open_Palm** (Xòe bàn tay) | **Dừng khẩn cấp** | Robot dừng ngay lập tức mọi chuyển động |
| **Closed_Fist** (Nắm tay) | **Hủy lệnh** | Hủy hành vi hiện tại và trở về trạng thái chờ (`IDLE`) |

> **Cơ chế an toàn:**
> 1. Robot **chỉ bắt đầu di chuyển** sau khi nhận được cử chỉ xác nhận `Thumb_Up`.
> 2. Lọc nhiễu chống kích hoạt lặp: Cử chỉ phải xuất hiện ổn định 5 frame liên tiếp (hoặc 3 frame cho lệnh dừng khẩn cấp `Open_Palm`).
> 3. Tự động dừng an toàn (Watchdog): Nếu mất tín hiệu từ webcam quá `1.5s`, robot tự động ngắt động cơ về `IDLE`.

---

## 4. Hướng dẫn chạy chương trình

### Bước 1: Khởi động mô phỏng TurtleBot 4 (Terminal 1)
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

### Bước 2: Khởi chạy hệ thống điều khiển cử chỉ (Terminal 2)
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb4_gesture gesture_control.launch.py
```

---

## 5. Lưu ý quan trọng cho môi trường WSL2

Mặc định WSL2 không tự động nhận webcam của máy Windows. Có 2 cách kết nối webcam vào hệ thống:

### Cách 1: Gắn trực tiếp qua `usbipd-win` (Khuyên dùng cho USB Webcam)
1. Mở **PowerShell (Administrator)** trên Windows:
   ```powershell
   # Liệt kê thiết bị USB và tìm BUSID của Webcam
   usbipd list
   # Bind và attach vào WSL (thay <BUSID> bằng ID tìm được, ví dụ 1-4)
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```
2. Kiểm tra trong terminal WSL2: `ls /dev/video*` (thấy `/dev/video0`).
3. Khởi chạy bình thường: `ros2 launch tb4_gesture gesture_control.launch.py`.

---

### Cách 2: Stream camera qua HTTP từ Windows (Tiện nhất cho Webcam tích hợp laptop)
Không cần cài driver hay `usbipd`. Đây cũng là cách được chọn để thực hiện bài tập này (điểm yếu là độ delay rất cao)

1. Sử dụng file `stream_cam.py` trên máy Windows, chạy bằng Python:



2. Kiểm tra IP host Windows từ WSL2:
   ```bash
   ip route show | grep default
   # Kết quả có dạng: default via 172.xx.xx.1 ...
   ```

3. Khởi chạy launch file trong WSL2 truyền URL camera:
   ```bash
   ros2 launch tb4_gesture gesture_control.launch.py camera_source:=http://172.23.64.1:5005/video
   ```
---

## 6. Cấu trúc mã nguồn

```text
src/tb4_gesture/
├── CMakeLists.txt / setup.py     # Cấu hình cài đặt package ROS 2
├── package.xml                   # Khai báo metadata và dependencies
├── README.md                     # Tài liệu hướng dẫn sử dụng
├── models/
│   └── gesture_recognizer.task   # Trọng số AI Google MediaPipe
├── launch/
│   └── gesture_control.launch.py # Khởi chạy gesture_recognition_node + robot_behavior_node
├── tb4_gesture/
│   ├── __init__.py
│   ├── gesture_recognition_node.py # Đọc webcam, nhận diện cử chỉ, vẽ UI & publish topic
│   └── robot_behavior_node.py     # FSM điều khiển hành vi TurtleBot 4 qua /cmd_vel
└── test/
    └── test_tb4_gesture.py        # Unit test cho logic nhận diện và state machine
```

