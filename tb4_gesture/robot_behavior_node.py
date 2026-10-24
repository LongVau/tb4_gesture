#!/usr/bin/env python3
"""ROS 2 Node: robot_behavior_node.

Subscribes to /detected_gesture (std_msgs/msg/String) and drives TurtleBot 4
via /cmd_vel (geometry_msgs/msg/Twist) with confirmation, anti-noise edge
triggering, immediate stop, cancel, and signal-loss watchdog.
"""

import time
from typing import Optional, Tuple

SELECTION_MAP = {
    "Wave": "WAVE",
    "Pointing_Up": "FORWARD_BACK_4X",
    "Victory": "CIRCLE",
}


class BehaviorFSM:
    """State machine for gesture-controlled TurtleBot 4 behaviors."""

    def __init__(self, watchdog_timeout: float = 1.5) -> None:
        self.watchdog_timeout = watchdog_timeout
        self.state: str = "IDLE"
        self.selected: Optional[str] = None
        self.active: Optional[str] = None
        self.start_time: float = 0.0
        self.last_msg_time: Optional[float] = None
        self.last_gesture: str = "None"

    def on_gesture(self, gesture: str, now: float) -> None:
        self.last_msg_time = now
        if gesture == "Open_Palm":
            self.state = "STOPPED"
            self.selected = None
            self.active = None
            self.last_gesture = gesture
            return

        if gesture == "Closed_Fist":
            self.state = "IDLE"
            self.selected = None
            self.active = None
            self.last_gesture = gesture
            return

        if gesture == self.last_gesture:
            return
        self.last_gesture = gesture

        if gesture in SELECTION_MAP and self.state != "EXECUTING":
            self.selected = SELECTION_MAP[gesture]
            self.state = "SELECTED"
        elif gesture == "Thumb_Up" and self.state == "SELECTED" and self.selected:
            self.active = self.selected
            self.selected = None
            self.state = "EXECUTING"
            self.start_time = now

    def step(self, now: float) -> Tuple[float, float]:
        if self.last_msg_time is None or (now - self.last_msg_time) > self.watchdog_timeout:
            self.state = "IDLE"
            self.selected = None
            self.active = None
            return 0.0, 0.0

        if self.state != "EXECUTING" or not self.active:
            return 0.0, 0.0

        t = max(0.0, now - self.start_time)

        if self.active == "WAVE":
            # 2 cycles of left-right-left greeting (8.0s total)
            if t >= 8.0:
                self._finish()
                return 0.0, 0.0
            phase = t % 4.0
            if phase < 1.0:
                return 0.0, 0.8
            if phase < 3.0:
                return 0.0, -0.8
            return 0.0, 0.8

        if self.active == "FORWARD_BACK_4X":
            # 4 cycles: 2.0s forward, 0.4s pause, 2.0s backward, 0.4s pause (19.2s total)
            cycle = 4.8
            if t >= 4 * cycle:
                self._finish()
                return 0.0, 0.0
            phase = t % cycle
            if phase < 2.0:
                return 0.25, 0.0
            if phase < 2.4:
                return 0.0, 0.0
            if phase < 4.4:
                return -0.25, 0.0
            return 0.0, 0.0

        if self.active == "CIRCLE":
            # Full circle at v=0.22 m/s, w=0.44 rad/s (R=0.5m, ~14.3s)
            if t >= 14.3:
                self._finish()
                return 0.0, 0.0
            return 0.22, 0.44

        return 0.0, 0.0

    def _finish(self) -> None:
        self.state = "IDLE"
        self.active = None


def main(args=None) -> None:
    from geometry_msgs.msg import Twist
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class RobotBehaviorNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_behavior_node")
            self.declare_parameter("watchdog_timeout", 1.5)
            timeout = float(self.get_parameter("watchdog_timeout").value)
            self.fsm = BehaviorFSM(watchdog_timeout=timeout)
            self.last_state_log = ""

            self.sub = self.create_subscription(
                String, "/detected_gesture", self._on_msg, 10
            )
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.timer = self.create_timer(0.05, self._tick)
            self.get_logger().info("robot_behavior_node ready (listening on /detected_gesture)")

        def _on_msg(self, msg: String) -> None:
            prev = (self.fsm.state, self.fsm.selected, self.fsm.active)
            self.fsm.on_gesture(msg.data.strip(), time.monotonic())
            cur = (self.fsm.state, self.fsm.selected, self.fsm.active)
            if cur != prev:
                self.get_logger().info(
                    f"Gesture='{msg.data}' -> state={cur[0]}, selected={cur[1]}, active={cur[2]}"
                )

        def _tick(self) -> None:
            lin, ang = self.fsm.step(time.monotonic())
            cmd = Twist()
            cmd.linear.x = float(lin)
            cmd.angular.z = float(ang)
            self.pub.publish(cmd)

    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

