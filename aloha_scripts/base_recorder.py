import numpy as np
import time
# from constants import DT
from collections import deque
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


import IPython
e = IPython.embed


class BaseRecorder:
    def __init__(self, init_node=True, is_debug=False):

        self.is_debug = is_debug
        self.linear = 0
        self.angular = 0
        callback_func = self.update
        if init_node:
            rospy.init_node('base_recorder', anonymous=True)
        rospy.Subscriber(f"/odom", Odometry, callback_func)
        time.sleep(0.5)
    def update(self,data):
        self.linear = round(data.twist.twist.linear.x,4)
        self.angular = round(data.twist.twist.angular.z,4)
    def get_vel(self):
        return self.linear, self.angular
#
# if __name__ == '__main__':
#     record = BaseRecorder()
#     for i in range(1000):
#         print(record.get_vel()[0])
#         print(record.get_vel()[1])
#         time.sleep(0.5)



