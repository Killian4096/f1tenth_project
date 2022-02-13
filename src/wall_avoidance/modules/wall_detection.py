from time import time
import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

ERROR_THRESHOLD = (3.14)/16

class wall_detection:
    def __init__(self):
        pass

    def wall_detect(self, laserscan_data):
        errors = self.get_errors(laserscan_data)
        if(np.greater(errors, ERROR_THRESHOLD)):
            return True
        return False
    
    def get_errors(self, laserscan_data):
        positions = self.get_positions()
        positions_x = positions[0]
        positions_y = positions[1]
        errors = np.sum(np.sum(
                np.add(np.roll(positions_x,  0), np.subtract(np.roll(positions_y, -1), np.roll(positions_y -2))),
                np.add(np.roll(positions_x, -1), np.subtract(np.roll(positions_y, -2), np.roll(positions_y  0)))
            )
                np.add(np.roll(positions_x, -2), np.subtract(np.roll(positions_y,  0), np.roll(positions_y  -1)))
        )
        return errors

    

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

class range_wall:
    def __init__():
        self.point1 = None
        self.point2 = None