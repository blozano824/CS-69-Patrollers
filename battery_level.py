#!/usr/bin/python

#import python libs
import numpy
import math

# import ros libs
import rospy
import tf
# import messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String

#can figure out how to deal with this later
NUM_ROBOTS = 10

class battery_level:

    def __init__(self):
        self.robot_name = ""
        self.robot_odom = None
        self.battery_level = 0
        self.battery_string = ""
        self.pub = rospy.Publisher("battery_level", String, queue_size=10)

    def get_robot_name(self):
        self.robot_name = rospy.get_namespace()

    def base_vel_truth(self, odom_msg):
        self.robot_odom = odom_msg

    def subscriber(self):
        rospy.Subscriber("/robot_" + self.robot_name + "/base_pose_ground_vel_truth", Odometry, self.base_vel_truth)

    def init_batttery(self):
        self.battery_level = 100

    def adjust_battery(self):
        if self.robot_odom.twist.twist.linear.x != 0 or self.robot_odom.twist.twist.angular.z != 0:
            self.battery_level = self.battery_level - math.abs(self.robot_odom.twist.twist.linear.x)
            self.battery_level = self.battery_level - math.abs(self.robot_odom.twist.twist.angular.z)

    def update_battery_level(self):
        self.battery_string = self.robot_name + "/" + str(self.battery_level)

        self.pub.publish(self.battery_string)

def main():
    rospy.init_node("battery_level")

if __name__ == '__main__':
    main()
