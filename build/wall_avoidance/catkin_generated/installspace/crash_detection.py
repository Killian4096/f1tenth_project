#!/usr/bin/env python3
from time import time
#from turtle import distance
#from xml.etree.ElementTree import TreeBuilder
import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

import numpy as np

TIME_THRESHOLD = 0.6
DIST_THRESHOLD = 5
CAR_SIZE = 0.08

MIN_DR = 0.01

rate = None

class crash_detection:
    def __init__(self):

        self.odom_data = Odometry()

        self.brake_bool_pub = rospy.Publisher('/brake_bool', std_msgs.msg.Bool, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake', ackermann_msgs.msg.AckermannDriveStamped, queue_size=10)
        self.LaserScan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.Odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
    
    def laserscan_callback(self, laserscan_data):
        if(self.detect_potential_collision(laserscan_data, TIME_THRESHOLD)):
            self.brake_bool_pub.publish(True)
        else:
            self.brake_bool_pub.publish(False)

    def detect_potential_collision(self, laserscan_data, time_threshold):
        time_impact_field = self.get_time_impact_field(laserscan_data)
        if min(time_impact_field) <= rospy.rostime.Duration(time_threshold,0).to_sec():
            return True
        return False

    def get_time_impact_field(self, laserscan_data):
        #Return time till impact s
        speed_field = self.get_speed_field(laserscan_data)
        distance_field = self.get_distance_field(laserscan_data)
        time_impact_field = np.divide(distance_field, speed_field)
        return time_impact_field

    def get_speed_field(self, laserscan_data):
        #Returns speed field m/s
        speed = self.get_speed(self.odom_data)
        angle_field = np.arange(laserscan_data.angle_min, laserscan_data.angle_max, laserscan_data.angle_increment)
        speed_field = speed * np.cos(angle_field)
        speed_field[speed_field <= 0] = 0.00001
        return speed_field
    
    def get_distance_field(self, laserscan_data):
        #Return ranges m
        distance_field = np.array(laserscan_data.ranges)
        return distance_field

    def get_speed(self, odom_data):
        #Return speeds m/s
        speed = odom_data.twist.twist.linear.x
        return speed


    def odometry_callback(self, odom_data):
        #Dont want the 2 having the same value, hence the buffer
        self.odom_data = odom_data
        self.brake_pub.publish(self.create_brake_ackermann_message(odom_data))

    def create_brake_ackermann_message(self, message_origin):
        message = ackermann_msgs.msg.AckermannDriveStamped()
        message.drive.speed = 0.0
        return message

def main():
    rospy.init_node('crash_detection')
    rate = rospy.Rate(10)
    crash_detection_obj = crash_detection()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass