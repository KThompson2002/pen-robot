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

def MY_PEN(f):
        f.set_scale()
        f.align_self()
        while True:
            frame = f.capture_frame()
            frame_HSV = f.convert_color(frame)
            frame_purple, mask = f.threshold_purple(frame_HSV)
            (cx, cy, cz), pnt = f.find_pen(mask)
            if cx != -1:
                # print(cx, cy, cz)
                return np.array([cx, cy, cz])

def convert_robot(rot, t, pnt):
    R_obj = Rotation.from_matrix(rot)
    rotated_point = R_obj.apply(pnt)
    return rotated_point + t


def main():
    rot =  np.loadtxt(filename)
    t = np.loadtxt("t.txt")
    with Robot("px100", "arm", "gripper") as robot:
        with Stream() as f:
            while True:
                robot.gripper.release()
                location = MY_PEN(f)
                robot_coord = convert_robot(rot, t, location)
                print(robot_coord)
                robot.arm.set_ee_pose_components(robot_coord[0]+0.055, robot_coord[1]-0.04,  robot_coord[2] +0.01)
                robot.gripper.grasp()
                time.sleep(3)
                robot.arm.go_to_sleep_pose()


    


if __name__ == "__main__":
    main()