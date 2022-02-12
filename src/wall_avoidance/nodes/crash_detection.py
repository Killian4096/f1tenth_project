#!/usr/bin/env python3
from time import time
from xml.etree.ElementTree import TreeBuilder
import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

TIME_THRESHOLD = 0.3
DIST_THRESHOLD = 5
CAR_SIZE = 0.08

MIN_DR = 0.01

rate = None

class crash_detection:
    def __init__(self):

        self.odom_data_current = Odometry()
        self.odom_data_past = Odometry()

        self.brake_bool_pub = rospy.Publisher('/brake_bool', std_msgs.msg.Bool, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake', ackermann_msgs.msg.AckermannDriveStamped, queue_size=10)
        self.LaserScan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.Odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.Debug_Vector_pub = rospy.Publisher('/debug_vector', LaserScan, queue_size=10)
    
    def laserscan_callback(self, laserscan_data):
        if(self.detect_potential_collision(laserscan_data, TIME_THRESHOLD)):
            self.brake_bool_pub.publish(True)
        else:
            self.brake_bool_pub.publish(False)

    def detect_potential_collision(self, laserscan_data, time_threshold):
        time_impact_field = self.get_time_impact_field(laserscan_data)
        for i in range(0, len(time_impact_field)):
            if (time_impact_field[i] <= rospy.rostime.Duration(time_threshold,0).to_nsec()) and (time_impact_field[i] != 0):
                return True
        return False

    def get_time_impact_field(self, laserscan_data):
        #Return time till impact s
        time_impact_field = []
        speed_field = self.get_speed_field(laserscan_data)
        distance_field = self.get_distance_field(laserscan_data)
        for i in range(0, len(distance_field)):
            time_impact_field.append(distance_field[i] / speed_field[i])
            if(time_impact_field[i]<0):
                time_impact_field[i]=0
        debug_scan = self.create_laserscan_message(laserscan_data, time_impact_field)
        self.Debug_Vector_pub.publish(debug_scan)
        return time_impact_field

    def get_speed_field(self, laserscan_data):
        #Returns speed field m/s
        speed = self.get_speed(self.odom_data_current, self.odom_data_past)
        speed_field = []
        for i in range(0, len(laserscan_data.ranges)):
            speed_field.append(speed * math.cos(laserscan_data.angle_min + (i * laserscan_data.angle_increment)))
        return speed_field
    
    def get_distance_field(self, laserscan_data):
        #Return ranges m
        distance_field = []
        for i in range(0, len(laserscan_data.ranges)):
            distance_field.append(laserscan_data.ranges[i] - CAR_SIZE)
        return distance_field

    def get_speed(self, odom_data_current, odom_data_past):
        #Return speeds m/s
        dx = (odom_data_current.pose.pose.position.x - odom_data_past.pose.pose.position.x)
        dt = (odom_data_current.header.stamp - odom_data_past.header.stamp)
        speed=(dx+0.001)/(dt.to_nsec() + rospy.rostime.Duration(0,1).to_nsec())
        return speed


    def odometry_callback(self, odom_data):
        #Dont want the 2 having the same value, hence the buffer
        odom_buffer = self.odom_data_current
        self.odom_data_current = odom_data
        self.odom_data_past = odom_buffer

        self.brake_pub.publish(self.create_brake_ackermann_message(odom_data))

    def create_brake_ackermann_message(self, message_origin):
        return ackermann_msgs.msg.AckermannDriveStamped(message_origin.header, ackermann_msgs.msg.AckermannDrive(0,0,0,0,0))
    
    def create_laserscan_message(self, laserscan_data, ranges):
        return LaserScan(laserscan_data.header, laserscan_data.angle_min, laserscan_data.angle_max, laserscan_data.angle_increment, laserscan_data.time_increment, laserscan_data.scan_time, laserscan_data.range_min, laserscan_data.range_max, ranges, laserscan_data.intensities)



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