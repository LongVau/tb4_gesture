from tb4_gesture.gesture_recognition_node import GestureFilter, parse_camera_source
from tb4_gesture.robot_behavior_node import BehaviorFSM


def test_parse_camera_source_accepts_int_and_str():
    assert parse_camera_source(0) == 0
    assert parse_camera_source("0") == 0
    assert parse_camera_source("/dev/video0") == "/dev/video0"
    assert parse_camera_source("http://127.0.0.1:5005/video") == "http://127.0.0.1:5005/video"


def test_gesture_filter_debounce_and_wave():
    gf = GestureFilter(min_confidence=0.6, confirm_frames=5, stop_confirm_frames=3)
    for i in range(4):
        g, _ = gf.update("Pointing_Up", 0.85, 0.5, i * 0.05)
        assert g == "None"
    g, conf = gf.update("Pointing_Up", 0.85, 0.5, 0.25)
    assert g == "Pointing_Up" and conf == 0.85

    # Low confidence ignored
    for i in range(5):
        g, _ = gf.update("Victory", 0.40, 0.5, 0.5 + i * 0.05)
    assert g == "None"

    # Static Open_Palm in 3 frames
    for i in range(3):
        g, _ = gf.update("Open_Palm", 0.9, 0.5, 1.0 + i * 0.05)
    assert g == "Open_Palm"

    # Oscillating palm_x triggers Wave
    for i, x in enumerate([0.40, 0.50, 0.40, 0.50, 0.40]):
        g, _ = gf.update("Open_Palm", 0.9, x, 2.0 + i * 0.08)
    assert g == "Wave"

    # ILoveYou maps to Wave after 5 frames
    for i in range(5):
        g, _ = gf.update("ILoveYou", 0.88, 0.5, 3.0 + i * 0.05)
    assert g == "Wave"


def test_behavior_fsm_confirm_stop_cancel_watchdog():
    fsm = BehaviorFSM(watchdog_timeout=1.5)
    # Select Pointing_Up -> still zero velocity until Thumb_Up
    fsm.on_gesture("Pointing_Up", 0.0)
    assert fsm.state == "SELECTED" and fsm.selected == "FORWARD_BACK_4X"
    assert fsm.step(0.05) == (0.0, 0.0)

    # Confirm with Thumb_Up -> starts moving forward
    fsm.on_gesture("Thumb_Up", 0.1)
    assert fsm.state == "EXECUTING"
    lin, ang = fsm.step(0.2)
    assert lin > 0.0 and ang == 0.0

    # Open_Palm stops immediately
    fsm.on_gesture("Open_Palm", 0.3)
    assert fsm.state == "STOPPED"
    assert fsm.step(0.35) == (0.0, 0.0)

    # Select Circle + Confirm, then Closed_Fist cancels to IDLE
    fsm.on_gesture("Victory", 1.0)
    fsm.on_gesture("Thumb_Up", 1.1)
    assert fsm.step(1.2)[0] > 0.0 and fsm.step(1.2)[1] > 0.0
    fsm.on_gesture("Closed_Fist", 1.3)
    assert fsm.state == "IDLE" and fsm.step(1.35) == (0.0, 0.0)

    # Watchdog timeout stops robot on signal loss
    fsm.on_gesture("Wave", 2.0)
    fsm.on_gesture("Thumb_Up", 2.1)
    assert fsm.step(2.2)[1] != 0.0
    assert fsm.step(4.0) == (0.0, 0.0)
    assert fsm.state == "IDLE"

