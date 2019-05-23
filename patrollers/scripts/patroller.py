#!/usr/bin/env python
import rospy
import math
import tf
from datetime import datetime

from geometry_msgs.msg import Twist
from std_msgs.msg import String
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
        self.current_vertex_publisher = rospy.Publisher("current_vertex", String, queue_size=10)
        self.laserscan_msg = None
        self.odometry_msg = None
        self.searching = True

        # Speed Variables
        self.turn_speed = 0.4
        self.move_speed = 1

        # Position Variables
        self.current_vertex = None
        self.current_x = None
        self.current_y = None
        self.current_direction = None

        # Target Position Variables
        self.target_error = 0.1
        self.target_vertex = None
        self.target_x = None
        self.target_y = None
        self.target_direction = None

        # Timing Variables
        self.timestamp = datetime.now()
        self.turn_duration = None
        self.move_duration = None

        rospy.init_node("follower")

    def store_incoming_odometry_msg(self, odometry_msg):
        self.current_x = odometry_msg.pose.pose.position.x
        self.current_y = odometry_msg.pose.pose.position.y

        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_direction = euler[2] * 180 / math.pi

    def store_incoming_laserscan_msg(self, laserscan_msg):
        self.laserscan_msg = laserscan_msg

    def store_incoming_target_vertex_msg(self, target_vertex_msg):
        target_info = target_vertex_msg.data.split(",")
        self.current_vertex, self.target_vertex, self.target_x, self.target_y = target_info[0], target_info[1], float(target_info[2]), float(target_info[3])
        print(self.target_vertex, self.target_x, self.target_y)
        self.target_direction = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x) * 180 / math.pi
        if self.target_direction < 0:
            self.target_direction = 360 + self.target_direction

    def subscribe(self):
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.store_incoming_odometry_msg, queue_size=1)
        rospy.Subscriber("base_scan", LaserScan, self.store_incoming_laserscan_msg, queue_size=1)
        rospy.Subscriber("target_vertex", String, self.store_incoming_target_vertex_msg, queue_size=1)

    def is_near_target(self, current_val, target_val):
        return abs(current_val - target_val) < self.target_error

    def patrol(self):
        curr_state = "AWAIT"
        patrol_state = "START"
        while not rospy.is_shutdown():
            timedelta = datetime.now() - self.timestamp
            cmd_msg = Twist()

            if curr_state == "AWAIT":
                if self.target_direction:
                    curr_state = "PATROL"
                    patrol_state = "START"

            if curr_state == "PATROL":

                if patrol_state == "START":
                    self.timestamp = datetime.now()
                    patrol_state = "CALCULATE_TURN_TIME"

                if patrol_state == "CALCULATE_TURN_TIME":
                    angle_diff = self.target_direction - self.current_direction
                    if angle_diff > 180:
                        angle_diff = -1 * (360 - angle_diff)
                    radian_diff = angle_diff * math.pi / 180

                    if radian_diff < 0:
                        self.turn_speed = -1 * abs(self.turn_speed)
                    else:
                        self.turn_speed = abs(self.turn_speed)

                    self.turn_duration = radian_diff / self.turn_speed

                    self.timestamp = datetime.now()
                    timedelta = self.timestamp - self.timestamp
                    print(angle_diff, self.target_x, self.target_y)
                    patrol_state = "PERFORM_TURN"

                if patrol_state == "PERFORM_TURN":
                    cmd_msg.angular.z = self.turn_speed
                    if timedelta.total_seconds() >= self.turn_duration:
                        self.timestamp = datetime.now()
                        timedelta = self.timestamp - self.timestamp
                        patrol_state = "MOVE_TOWARDS"

                if patrol_state == "MOVE_TOWARDS":
                    cmd_msg.linear.x = self.move_speed
                    if self.is_near_target(self.current_x, self.target_x) and self.is_near_target(self.current_y, self.target_y):

                        current_vertex_msg = String()
                        current_vertex_msg.data = self.current_vertex + "," + self.target_vertex + "," + str(self.target_x) + "," + str(self.target_y)
                        self.current_vertex_publisher.publish(current_vertex_msg)

                        self.current_vertex = self.target_vertex = self.target_x = self.target_y = self.target_direction = None
                        curr_state = "AWAIT"
                        patrol_state = "START"
                    if timedelta.total_seconds() >= 0.5:
                        curr_state = "PATROL"
                        patrol_state = "START"



            # if self.laserscan_msg:
            #     ranges = numpy.array(self.laserscan_msg.ranges)
            #     min_laser_scan_reading = ranges.min()
            #     min_laser_scan_angle = float(numpy.argmin(ranges) + 1) / 2
            #     if min_laser_scan_reading < 0.75:
            #         if 110 < min_laser_scan_angle < 250:
            #             cmd_msg.linear.x = 0

            self.cmd_vel_publisher.publish(cmd_msg)


if __name__ == '__main__':
    patroller = Patroller()
    patroller.subscribe()
    patroller.patrol()
