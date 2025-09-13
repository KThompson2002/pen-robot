from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

robot_startup()

mode = 'h'
# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode == "g":
        robot.gripper.grasp()
    elif mode == "o":
        robot.gripper.release()
    elif mode == "r":
        pos = robot.arm.get_single_joint_command("waist")
        robot.arm.set_single_joint_position("waist", pos + 0.1)
    elif mode == "w":
        wrist = robot.arm.get_single_joint_command("wrist_angle")
        robot.arm.set_single_joint_position("wrist_angle", wrist - (np.pi/2))
    elif mode == "p":
        robot.arm.go_to_home_pose()
        robot.gripper.release()
        pos = robot.arm.get_single_joint_command("waist")
        robot.arm.set_single_joint_position("waist", pos - (np.pi/2))
        robot.arm.set_ee_cartesian_trajectory(-.02, 0, -.02)
        wrist = robot.arm.get_single_joint_command("wrist_angle")
        robot.arm.set_single_joint_position("wrist_angle", wrist + (np.pi/4))
        robot.gripper.grasp()
        robot.arm.go_to_home_pose()
        robot.arm.go_to_sleep_pose()
    elif mode == "pose1":
        # pos = robot.arm.get_single_joint_command("waist")
        # robot.arm.set_single_joint_position("waist", pos + 0.25)
        robot.arm.set_ee_pose_components(0.2, 0.15, 0.15)
    elif mode == "pose2":
        robot.arm.set_ee_pose_components(0.15, -0.1, 0.18)
    elif mode == "print":
        print(robot.arm.get_ee_pose())
    elif mode == "pose3":
        robot.arm.set_ee_pose_components(0.1, -0.15, 0.21)
    elif mode == "pose4":
        robot.arm.set_ee_pose_components(0.2, 0, 0.12)
    elif mode == "pose5":
        robot.arm.set_ee_pose_components(0.25, 0, 0.18)
    elif mode == "pose6":
        robot.arm.set_ee_pose_components(0.2, 0.1, 0.15)
    elif mode == "pose7":
        robot.arm.set_ee_pose_components(0.18, 0.15, 0.12)
    elif mode == "pose8":
        robot.arm.set_ee_pose_components(0.23, 0.18, 0.11)

robot_shutdown()
