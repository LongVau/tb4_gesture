#!/usr/bin/env python3
"""ROS 2 Node: gesture_recognition_node.

Reads webcam frames with OpenCV, classifies 6 hand gestures using MediaPipe
GestureRecognizer (gesture_recognizer.task), draws gesture name + confidence
on the webcam window, and publishes std_msgs/msg/String to /detected_gesture.
"""

from collections import deque
import os
from pathlib import Path
import time
from typing import Deque, Optional, Tuple

GESTURE_LABELS = {
    "Wave": "Wave (Chao: Xoay trai-phai)",
    "Pointing_Up": "Pointing Up (Chon: Tien-Lui 4 lan)",
    "Victory": "Victory (Chon: Quy dao tron)",
    "Thumb_Up": "Thumbs Up (XAC NHAN THUC HIEN)",
    "Open_Palm": "Open Palm (DUNG NGAY LAP TUC)",
    "Closed_Fist": "Closed Fist (HUY -> Cho)",
    "None": "None (Khong co cu chi)",
}


class GestureFilter:
    """Stabilizes MediaPipe raw hand labels and detects horizontal hand Wave."""

    def __init__(
        self,
        min_confidence: float = 0.60,
        confirm_frames: int = 5,
        stop_confirm_frames: int = 3,
        wave_span_threshold: float = 0.08,
        wave_window_sec: float = 0.60,
    ) -> None:
        self.min_confidence = min_confidence
        self.confirm_frames = confirm_frames
        self.stop_confirm_frames = stop_confirm_frames
        self.wave_span_threshold = wave_span_threshold
        self.wave_window_sec = wave_window_sec

        self._palm_history: Deque[Tuple[float, float]] = deque()
        self._candidate: str = "None"
        self._streak: int = 0
        self._current: str = "None"
        self._confidence: float = 0.0

    def _is_waving(self, now: float) -> bool:
        while self._palm_history and (now - self._palm_history[0][0]) > self.wave_window_sec:
            self._palm_history.popleft()
        if len(self._palm_history) < 4:
            return False
        xs = [x for _, x in self._palm_history]
        if (max(xs) - min(xs)) < self.wave_span_threshold:
            return False
        reversals = 0
        prev_dir = 0
        for i in range(1, len(xs)):
            dx = xs[i] - xs[i - 1]
            if abs(dx) < 0.012:
                continue
            cur_dir = 1 if dx > 0 else -1
            if prev_dir != 0 and cur_dir != prev_dir:
                reversals += 1
            prev_dir = cur_dir
        return reversals >= 1

    def update(
        self,
        raw_label: Optional[str],
        score: float,
        palm_x: Optional[float],
        now: float,
    ) -> Tuple[str, float]:
        if palm_x is not None:
            self._palm_history.append((now, float(palm_x)))
        else:
            self._palm_history.clear()

        mapped = "None"
        if raw_label and score >= self.min_confidence:
            if raw_label == "ILoveYou":
                mapped = "Wave"
            elif raw_label == "Open_Palm":
                mapped = "Wave" if self._is_waving(now) else "Open_Palm"
            elif raw_label in ("Pointing_Up", "Victory", "Thumb_Up", "Closed_Fist"):
                mapped = raw_label

        if mapped == "Wave" and self._is_waving(now) and raw_label == "Open_Palm":
            self._current = "Wave"
            self._confidence = score
            self._candidate = "Wave"
            self._streak = self.confirm_frames
            return self._current, self._confidence

        if mapped == self._candidate:
            self._streak += 1
        else:
            self._candidate = mapped
            self._streak = 1

        needed = self.stop_confirm_frames if mapped == "Open_Palm" else self.confirm_frames
        if self._streak >= needed:
            self._current = mapped
            self._confidence = score if mapped != "None" else 0.0
        elif mapped == self._current and mapped != "None":
            self._confidence = score

        return self._current, self._confidence


def parse_camera_source(val: object) -> object:
    if isinstance(val, int):
        return val
    s = str(val).strip()
    return int(s) if s.isdigit() else s


def _default_model_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("tb4_gesture")) / "models" / "gesture_recognizer.task"
        if p.is_file():
            return str(p)
    except Exception:
        pass
    return str(Path(__file__).resolve().parent.parent / "models" / "gesture_recognizer.task")


def main(args=None) -> None:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    from rcl_interfaces.msg import ParameterDescriptor
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class GestureRecognitionNode(Node):
        def __init__(self) -> None:
            super().__init__("gesture_recognition_node")
            self.declare_parameter(
                "camera_source",
                "0",
                ParameterDescriptor(dynamic_typing=True),
            )
            self.declare_parameter("model_path", _default_model_path())
            self.declare_parameter("min_confidence", 0.60)
            self.declare_parameter("show_window", True)

            self.camera_source = parse_camera_source(
                self.get_parameter("camera_source").value
            )
            model_path = str(self.get_parameter("model_path").value)
            self.show_window = bool(self.get_parameter("show_window").value)

            if not os.path.isfile(model_path):
                raise FileNotFoundError(f"Model not found: {model_path}")

            self.filter = GestureFilter(
                min_confidence=float(self.get_parameter("min_confidence").value)
            )
            options = mp_vision.GestureRecognizerOptions(
                base_options=mp_python.BaseOptions(model_asset_path=model_path),
                running_mode=mp_vision.RunningMode.VIDEO,
                num_hands=1,
            )
            self.recognizer = mp_vision.GestureRecognizer.create_from_options(options)
            self.pub = self.create_publisher(String, "/detected_gesture", 10)
            self.cap = None
            self.last_open_try = 0.0
            self.last_ts_ms = 0
            self.last_logged = ""

            self._open_camera()
            self.timer = self.create_timer(0.05, self._tick)
            self.get_logger().info(f"gesture_recognition_node ready (source={self.camera_source})")

        def _open_camera(self) -> None:
            now = time.monotonic()
            if now - self.last_open_try < 2.0:
                return
            self.last_open_try = now
            if self.cap is not None:
                self.cap.release()
            src = self.camera_source
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
            if not cap.isOpened() and isinstance(src, int):
                cap = cv2.VideoCapture(src)
            if cap.isOpened():
                self.cap = cap
                self.get_logger().info(f"Webcam opened: {src}")
            else:
                self.get_logger().warn(f"Cannot open webcam ({src}). Waiting...")

        def _tick(self) -> None:
            if self.cap is None or not self.cap.isOpened():
                self._open_camera()
                return
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.cap.release()
                self.cap = None
                return

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = max(self.last_ts_ms + 1, int(time.monotonic() * 1000))
            self.last_ts_ms = ts_ms

            res = self.recognizer.recognize_for_video(mp_img, ts_ms)
            raw_label, score, palm_x = None, 0.0, None
            landmarks = None
            if res.gestures and res.gestures[0]:
                raw_label = res.gestures[0][0].category_name
                score = float(res.gestures[0][0].score)
            if res.hand_landmarks and res.hand_landmarks[0]:
                landmarks = res.hand_landmarks[0]
                palm_x = float(landmarks[9].x)

            gesture, conf = self.filter.update(raw_label, score, palm_x, time.monotonic())
            msg = String()
            msg.data = gesture
            self.pub.publish(msg)

            if gesture != self.last_logged:
                self.last_logged = gesture
                self.get_logger().info(f"Gesture: {gesture} ({conf * 100:.1f}%)")

            if self.show_window:
                h, w = frame.shape[:2]
                if landmarks:
                    xs = [int(lm.x * w) for lm in landmarks[1:]]
                    ys = [int(lm.y * h) for lm in landmarks[1:]]
                    cv2.rectangle(
                        frame,
                        (max(0, min(xs) - 12), max(0, min(ys) - 12)),
                        (min(w - 1, max(xs) + 12), min(h - 1, max(ys) + 12)),
                        (0, 0, 255) if gesture == "Open_Palm" else (0, 255, 0),
                        2,
                    )
                cv2.rectangle(frame, (0, 0), (w, 72), (25, 25, 25), -1)
                conf_txt = f"{conf * 100:.1f}%" if gesture != "None" else "--"
                cv2.putText(
                    frame,
                    f"Gesture: {gesture} ({conf_txt})",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 128) if gesture != "Open_Palm" else (80, 80, 255),
                    2,
                )
                cv2.putText(
                    frame,
                    GESTURE_LABELS.get(gesture, gesture),
                    (12, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (230, 230, 230),
                    1,
                )
                try:
                    cv2.imshow("TurtleBot 4 Gesture Recognition", frame)
                    cv2.waitKey(1)
                except cv2.error:
                    self.show_window = False

        def destroy_node(self) -> bool:
            if self.cap is not None:
                self.cap.release()
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            self.recognizer.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

