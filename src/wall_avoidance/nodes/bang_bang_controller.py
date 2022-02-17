from time import time
import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

import sys, os
sys.path.insert(0, '..')
from modules.wall_detection import WallDetector

class BangBangController:
    def __init__(self):
        self.WallDetector = WallDetector()

    def runController(self):
        closepoint = self.WallDetector.get_closepoint(laserscan_data)
        wallangle = self.WallDtector.get_wallangle(laserscan_data)