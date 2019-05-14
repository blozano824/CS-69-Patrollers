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
        self.robot_odom = {}
        self.battery_level = {}
        self.battery_string = ""
        self.pub = rospy.Publisher("battery_level", String, queue_size=10)

    def base_vel_truth(self, odom_msg):
        robot_name = odom_msg.header.frame_id.split("/")[1]
        self.robot_odom[robot_name] = odom_msg

    def subscriber(self):
        for i in range(NUM_ROBOTS):
            rospy.Subscriber("/robot_" + str(i) + "/base_pose_ground_vel_truth", Odometry, self.base_vel_truth)

    def init_batttery(self):
        for robot in self.robot_odom.keys():
            self.battery_level[robot] = 100

    def adjust_battery(self):
        for robot in self.robot_odom.keys():
            if self.robot_odom[robot].twist.twist.linear.x > 0 or self.robot_odom[robot].twist.twist.angular.z:
                self.battery_level[robot] = self.battery_level[robot] - self.robot_odom[robot].twist.twist.linear.x
                self.battery_level[robot] = self.battery_level[robot] - self.robot_odom[robot].twist.twist.angular.z

    def update_battery_level(self):
        for robot in self.robot_odom.keys():
            self.battery_string = self.battery_string + robot + "/" + str(self.battery_level[robot]) + " "

        self.pub.publish(self.battery_string)

def main():
    rospy.init_node("battery_level")

if __name__ == '__main__':
    main()
