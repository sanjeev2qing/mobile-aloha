import os
import time
import h5py
import argparse
import h5py_cache
import numpy as np
from tqdm import tqdm
import cv2

from constants import DT, START_ARM_POSE, TASK_CONFIGS, FPS
from constants import MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE, PUPPET_GRIPPER_JOINT_OPEN, MASTER_GRIPPER_JOINT_OPEN
from robot_utils import Recorder, ImageRecorder, get_arm_gripper_positions
from robot_utils import move_arms, torque_on, torque_off, move_grippers
from real_mobile_env import make_real_env, get_action

from interbotix_xs_modules.arm import InterbotixManipulatorXS

import IPython
e = IPython.embed
def opening_ceremony():
    master_bot_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper",
                                              robot_name=f'master_left', init_node=True)

    env = make_real_env(init_node=False, setup_robots=False)
    puppet_bot_left = env.puppet_bot_left
    """ Move all 4 robots to a pose where it is easy to start demonstration """
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot_left.dxl.robot_reboot_motors("single", "gripper", True)
    puppet_bot_left.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot_left.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot_left.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot_left.dxl.robot_set_operating_modes("single", "gripper", "position")
    #puppet_bot_left.dxl.robot_set_motor_registers("single", "gripper", 'current_limit', 1000) # TODO(tonyzhaozh) figure out how to set this limit

    torque_on(puppet_bot_left)
    torque_on(master_bot_left)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([master_bot_left, puppet_bot_left], [start_arm_qpos] * 2, move_time=1.5)
    # move grippers to starting position
    move_grippers([master_bot_left, puppet_bot_left], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE],  move_time=0.5)

if __name__ == '__main__':
    opening_ceremony()
