#!/usr/bin/env python3
from xml.etree.ElementTree import TreeBuilder
import rospy
import std_msgs.msg
from std_msgs.msg import Header
import ackermann_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

TIME_THRESHOLD = 1.3
DIST_THRESHOLD = 1.3

rate = None

class crash_detection:
    def __init__(self):

        self.odom_data_current = Odometry()
        self.odom_data_past = Odometry()

        self.brake_bool_pub = rospy.Publisher('/brake_bool', std_msgs.msg.Bool, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake', ackermann_msgs.msg.AckermannDriveStamped, queue_size=10)
        self.LaserScan_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.Odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
    
    def laserscan_callback(self, laserscan_data):
        dx = (self.odom_data_current.pose.pose.position.x - self.odom_data_past.pose.pose.position.x)
        dt = (self.odom_data_current.header.stamp - self.odom_data_past.header.stamp)

        if dt.to_nsec() != 0:
            speed=dx/dt.to_nsec()
        else:
            speed=0
        for i in range(0, len(laserscan_data.ranges)):
            dr = (speed * math.cos(laserscan_data.angle_min + (i * laserscan_data.angle_increment)))
            r = laserscan_data.ranges[i]
            if dr != 0:
                if ((r/dr) <= rospy.rostime.Duration(TIME_THRESHOLD).to_nsec() and (r/dr)>0) and r<DIST_THRESHOLD:
                    self.brake_bool_pub.publish(True)
                else:
                    self.brake_bool_pub.publish(False)
            else:
                self.brake_bool_pub.publish(False)



    def odometry_callback(self, odom_data):
        #Dont want the 2 having the same value, hence the buffer
        odom_buffer = self.odom_data_current
        self.odom_data_current = odom_data
        self.odom_data_past = odom_buffer

        self.brake_pub.publish(self.create_brake_ackermann_message(odom_data))

    def create_brake_ackermann_message(self, message_origin):
        return ackermann_msgs.msg.AckermannDriveStamped(message_origin.header, ackermann_msgs.msg.AckermannDrive(0,0,0,0,0))



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