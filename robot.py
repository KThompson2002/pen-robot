from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup

class Robot():
    def __init__(self, robo, arm, grip):
        self.robot = InterbotixManipulatorXS(robo, arm, grip)
        
    def __enter__(self):
        robot_startup()
        self.robot.arm.go_to_sleep_pose()
        return self.robot
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.arm.go_to_sleep_pose
        robot_shutdown()