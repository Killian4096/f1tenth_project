import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

ERROR_THRESHOLD = (3.14)/16
CONSECUTIVE_POINTS = 10

class WallDetector:
    def __init__(self):
        pass

    def get_wallangle(self, laserscan_data):
        ranges = self.get_ranges(laserscan_data)
        angles = self.get_angles(laserscan_data)

        closepointIndex = ranges.where(self.get_closepoint(laserscan_data))
        return angles[closepointIndex]

    def get_closepoint(self, laserscan_data):
        return self.get_ranges(laserscan_data).amin()

    def get_ranges(self, laserscan_data):
        ranges = np.array(laserscan_data.ranges)
        ranges[ranges >= laserscan_data.range_max] = laserscan_data.range_max * 2
        return ranges

    def get_angles(self, laserscan_data):
        angles = np.arange(laserscan_data.angle_min, laserscan_data.angle_max, laserscan_data.angle_increment)
        return angles
    
    def get_positions(self, laserscan_data):
        ranges = self.get_ranges(laserscan_data)
        angles = self.get_angles(laserscan_data)
        positions_x = np.multiply(ranges, np.sin(angles))
        positions_y = np.multiply(ranges, np.cos(angles))
        positions = (positions_x, positions_y)
        return positions