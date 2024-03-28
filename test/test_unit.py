# import pytest
import time
import cv2
from util import openFlirCamera
from common import Robot


def test_open_filr_camera():
    cap = openFlirCamera()
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (960, 540))
        # frame = cv2.resize(frame, None, fx=0.25, fy=0.25)
        cv2.imshow("press q to quit", frame)
        key = cv2.waitKey(30)
        if key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def test_one_robot():
    # Open camera. Parameter: camera_index
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)
    # Parameter: camera_index frame_width frame_height
    frame_width, frame_height = 640, 480
    robot = Robot(cap=cap, frame_width=frame_width, frame_height=frame_height)
    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (frame_width, frame_height))
        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            cv2.waitKey(10)
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


def test_multiple_robot():
    import time
    # Open camera. Parameter: camera_index frame_width frame_height
    cap = cv2.VideoCapture(0)
    # Wait for camera init
    time.sleep(3)

    # Parameter: camera_index
    frame_width, frame_height = 640, 480
    robot = Robot(cap, frame_width=frame_width, frame_height=frame_height)
    robot1 = Robot(cap, frame_width=frame_width, frame_height=frame_height)

    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.resize(frame, (frame_width, frame_height))
        if not ret:
            break
        try:
            robot.process_frame(frame)
            robot.show_robot_frame(frame)
            robot1.process_frame(frame)
            robot1.show_robot_frame(frame)
            # Retrieve and print the current robot position
            robot_position = robot.get_robot_position()
            print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")
            robot_position1 = robot1.get_robot_position()
            print(f"Robot Position: {robot_position1[0]}  {robot_position1[1]}")
            # Get robot show_frame for visualization
            cv2.imshow("imgShow", frame)
            key = cv2.waitKey(10)
            if key == ord("q"):
                break
        except Exception as e:
            raise RuntimeError("Robot Tracking Running Error: Error processing frame.") from e


if __name__ == '__main__':
    # test_open_filr_camera()
    # test_one_robot()
    test_multiple_robot()