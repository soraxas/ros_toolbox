import rospy

import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from . import MoveitRobotROS

use_real_jaco = False


# use_real_jaco = True


def set_robot_type(real_or_not: True):
    global use_real_jaco
    use_real_jaco = real_or_not


def initialise(name: str = "ipython"):
    if use_real_jaco:
        # needs to map the real jaco driver states to the typeical root /joint_states
        # to make it work with moveit
        joint_state_topic = ["joint_states:=/j2n6s300_driver/out/joint_state"]
        moveit_commander.roscpp_initialize(joint_state_topic)

    rospy.init_node(name, anonymous=False)

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    arm = robot.get_group("arm")
    gripper = robot.get_group("gripper")

    arm.set_max_acceleration_scaling_factor(0.8)
    arm.set_max_velocity_scaling_factor(0.8)

    return scene, robot, arm, gripper


class JacoRobotROS(MoveitRobotROS):
    def __init__(self):
        super().__init__()

    def get_joint_state_msg(self, pt):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = [
            "j2n6s300_joint_1",
            "j2n6s300_joint_2",
            "j2n6s300_joint_3",
            "j2n6s300_joint_4",
            "j2n6s300_joint_5",
            "j2n6s300_joint_6",
        ]
        msg.position = pt
        return msg
