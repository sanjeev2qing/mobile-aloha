# IMU传感器数据记录
import time
import numpy as np

import IPython
e = IPython.embed

import rospy
import tf
from tf.transformations import *
from sensor_msgs.msg import Imu

import IPython
e = IPython.embed


# def callback(data):
#     # 这个函数是tf中的,可以将四元数转成欧拉角
#     (r, p, y) = tf.transformations.euler_from_quaternion(
#         (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
#     # 由于是弧度制，下面将其改成角度制看起来更方便
    # rospy.loginfo("Roll = %f, Pitch = %f, Yaw = %f", r * 180 / 3.1415926, p * 180 / 3.1415926, y * 180 / 3.1415926)


class IMURecorder:
    def __init__(self, init_node=True, is_debug=False):
        self.is_debug = is_debug
        self.orientation = []
        self.angular_velocity = []
        self.linear_acceleration = []

        callback_func = self.IMU

        if init_node:
            rospy.init_node('imu_recorder', anonymous=True)
        rospy.Subscriber(f"/handsfree/imu", Imu, callback_func)
        time.sleep(0.5)

    def IMU(self, imu):
        self.orientation = np.array(imu.orientation)
        self.angular_velocity = np.array(imu.angular_velocity)
        self.linear_acceleration = np.array(imu.linear_acceleration)

    def get_imu_vel(self):
        return self.orientation, self.angular_velocity, self.linear_acceleration


if __name__ == '__main__':
    record = IMURecorder()
    for i in range(1000):
        print(record.get_imu_vel())
        time.sleep(0.5)
