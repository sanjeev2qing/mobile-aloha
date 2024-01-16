import numpy as np
import time
# from constants import DT
from collections import deque
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import IPython
e = IPython.embed


class SCANRecorder:
    def __init__(self, init_node=True, is_debug=False):

        self.is_debug = is_debug
        self.ranges = []

        callback_func = self.SCAN
        if init_node:
            rospy.init_node('scan_recorder', anonymous=True)
        rospy.Subscriber(f"/scan", LaserScan, callback_func)
        time.sleep(0.5)
    def SCAN(self, scan):
        # print("hello")
        self.ranges = np.array(scan.ranges)

    def get_scan_vel(self):
        return self.ranges

if __name__ == '__main__':
    record = SCANRecorder()
    for i in range(1000):
        print(record.get_scan_vel())
        time.sleep(0.5)



