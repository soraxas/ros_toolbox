import numpy as np
import rospy
from genpy.message import Message
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from typing import Iterator
from collections import OrderedDict

from ..main import construct_publisher


def ensure_configurations_in_bound(confs: np.ndarray, bounds: np.ndarray):
    """Given
        confs:  [n x d]
        bounds: [n x d] x 2
    Returns:
        confs (list) that are within the bounds (by +/- 2*PI)
    """
    assert confs.shape == bounds[0].shape
    assert confs.shape == bounds[1].shape
    new_confs = confs.copy()
    error = False
    for i, bound in enumerate(bounds):
        assert bound[0] < bound[1]
        while True:
            within_bound = True
            # deal with smaller than lower bound
            idxes = np.where(new_confs[:, i] < bound[0])
            within_bound = idxes[0].shape[0] == 0 and within_bound
            new_confs[idxes, i] += np.pi * 2
            if (new_confs[idxes, i] > bound[1]).any():
                raise ValueError(
                    "ERROR +=PI*2: Not possible to wrap the followings (joint idx:{}):"
                    "\n{}\n to fit within bound {}".format(i, confs[idxes, :], bound)
                )
            # deal with larger than upper bound
            idxes = np.where(new_confs[:, i] > bound[1])
            within_bound = idxes[0].shape[0] == 0 and within_bound
            new_confs[idxes, i] -= np.pi * 2
            if (new_confs[idxes, i] < bound[0]).any():
                raise ValueError(
                    "ERROR -=PI*2: Not possible to wrap the followings (joint idx:{}):"
                    "\n{}\n to fit within bound {}".format(i, confs[idxes, :], bound)
                )
            if within_bound:
                break
    return new_confs


#########################


class MoveitRobotROS:
    def __init__(self):
        super().__init__()

    def get_robot_state_msg(self, pt):
        robot_state = RobotState()
        robot_state.joint_state = self.get_joint_state_msg(pt)
        return robot_state

    def get_joint_state_msg(self, pt):
        raise NotImplementedError()

    def get_pub_for_setting_start_state(self):
        pub = rospy.Publisher(
            "/move_group/fake_controller_joint_states", JointState, queue_size=1
        )
        return pub
