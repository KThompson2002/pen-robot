#!usr/bin/env python3
import numpy as np
from stream import Stream 
from robot import Robot
import pyrealsense2 as rs
import time
import cv2
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from scipy.spatial.transform import Rotation

filename = "calibration.txt"

positions = np.array([[0.2, 0.15, 0.15], [0.15, -0.1, 0.18], [0.1, -0.15, 0.21],
                        [0.2, 0, 0.12], [0.25, 0, 0.18], [0.2, 0.1, 0.15],
                        [0.18, 0.15, 0.12], [0.23, 0.18, 0.11]])

def take_camera_point():
    with Stream() as f:
        f.set_scale()
        f.align_self()
        while True:
            frame = f.capture_frame()
            frame_HSV = f.convert_color(frame)
            frame_purple, mask = f.threshold_purple(frame_HSV)
            (cx, cy, cz), pnt = f.find_pen(mask)
            if cx != -1 and cx != 0 and cy != 0 and cz != 0 and cx < 1 and cy < 1 and cz < 1:

                # print(cx, cy, cz)
                return np.array([cx, cy, cz])
                

def take_robot_point(robo):
    pos = robo.arm.get_ee_pose()
    # print(np.array([pos[0][2], pos[1][2], pos[2][2]]))
    return np.array([pos[0][3], pos[1][3], pos[2][3]])


def main():
    with Robot("px100", "arm", "gripper") as robot:
        robot.arm.go_to_home_pose()
        robot.gripper.release()
        while True:
            ans = input("Give me the Pen and enter c to close: ")
            if (ans == 'c'):
                robot.gripper.grasp()
                time.sleep(3)
                break

        camera_points = np.array([])
        robot_points = np.array([])
        robot.arm.go_to_sleep_pose()
        robot_points = np.append(robot_points, take_robot_point(robot))
        camera_points = np.append(camera_points, take_camera_point())
        robot.arm.go_to_home_pose()
        robot_points = np.vstack((robot_points, take_robot_point(robot)))
        camera_points = np.vstack((camera_points, take_camera_point()))
        for i in range(len(positions)):
            robot.arm.set_ee_pose_components(positions[i][0], positions[i][1], positions[i][2])
            robot_points = np.vstack((robot_points, take_robot_point(robot)))
            camera_points = np.vstack((camera_points, take_camera_point()))

        print(robot_points)
        print(camera_points)

        Qhead = np.mean(robot_points, axis = 0)
        Phead = np.mean(camera_points, axis = 0)
        print(Phead)
        print(Qhead)
        p_avg = Phead
        q_avg = Qhead
        P_i = robot_points - Phead[np.newaxis, :]
        Q_i = camera_points - Qhead[np.newaxis, :]
        print(P_i)
        print(Q_i)
        R, rssd = Rotation.align_vectors(Q_i, P_i, return_sensitivity=False)
        print(R.as_matrix())
        t = q_avg - np.dot(R.as_matrix(), p_avg)
        print(t)

        with open(filename, 'w') as file:
            file.write(str(R.as_matrix()) + "\n")
            file.write(str(t)+ "\n")

        

if __name__ == "__main__":
    main()