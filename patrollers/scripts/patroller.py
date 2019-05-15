#!/usr/bin/env python
import rospy
import numpy
import math
from datetime import datetime

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Patroller:
    """
    Patroller class for controlling a
    single patrolling robot's behavior

    Attributes:
        cmd_pub (:obj:Publisher): publisher for command velocities.
        laserscan_msg (:obj:laser_scan.msg): variable to store latest laser scan sensor data

    """
    def __init__(self):
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.laserscan_msg = None
        self.odometry_msg = None
        self.searching = True

        # Speed Variables
        self.turn_speed = 1
        self.move_speed = 1

        # Position Variables
        self.current_x = None
        self.current_y = None

        # Target Position Variables
        self.target_error = 0.25
        self.target_x = None
        self.target_y = None

        # Timing Variables
        self.timestamp = datetime.now()
        self.turn_duration = None
        self.move_duration = None

        rospy.init_node("follower")

    def store_incoming_odometry_msg(self, odometry_msg):
        self.odometry_msg = odometry_msg

    def store_incoming_laserscan_msg(self, laserscan_msg):
        self.laserscan_msg = laserscan_msg

    def subscribe_to_channels(self):
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, self.store_incoming_leader_odometry_msg, queue_size=1)
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.store_incoming_odometry_msg, queue_size=1)
        rospy.Subscriber("base_scan", LaserScan, self.store_incoming_laserscan_msg, queue_size=1)
        print("I am " + rospy.get_namespace())

    def is_near_target(self, current_val, target_val):
        return abs(current_val - target_val) < self.target_error

    def patrol(self):
        curr_state = "AWAIT"
        patrol_state = "START"
        while not rospy.is_shutdown():
            timedelta = datetime.now() - self.timestamp
            cmd_msg = Twist()

            if curr_state == "AWAIT":
                if self.target_x and self.target_y:
                    curr_state = "PATROL"
                    patrol_state = "START"

            if curr_state == "PATROL":

                if patrol_state == "START":
                    self.timestamp = datetime.now()
                    patrol_state = "CALCULATE_TURN_TIME"

                if patrol_state == "CALCULATE_TURN_TIME":
                    # TODO Calculate angle difference between current position and target position
                    angle_diff = 200

                    radian_diff = angle_diff * math.pi / 180
                    self.turn_duration = radian_diff / self.turn_speed
                    self.timestamp = datetime.now()
                    patrol_state = "PERFORM_TURN"

                if patrol_state == "PERFORM_TURN":
                    cmd_msg.angular.z = self.turn_speed
                    if timedelta.total_seconds() >= self.turn_duration:
                        self.timestamp = datetime.now()
                        patrol_state = "MOVE_TOWARDS"

                if patrol_state == "MOVE_TOWARDS":
                    cmd_msg.linear.x = self.move_speed
                    if self.is_near_target(self.current_x, self.target_x) and self.is_near_target(self.current_y, self.target_y):
                        self.target_x = self.target_y = None
                        curr_state = "AWAIT"
                        patrol_state = "START"

            if self.laserscan_msg:
                ranges = numpy.array(self.laserscan_msg.ranges)
                min_laser_scan_reading = ranges.min()
                min_laser_scan_angle = float(numpy.argmin(ranges) + 1) / 2
                if min_laser_scan_reading < 0.75:
                    if 110 < min_laser_scan_angle < 250:
                        cmd_msg.linear.x = 0

            self.cmd_vel_publisher.publish(cmd_msg)

        rospy.spin()


if __name__ == '__main__':
    patroller = Patroller()
    patroller.subscribe_to_channels()
    patroller.patrol()
