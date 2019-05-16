#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class BatteryLevel:

    def __init__(self):
        self.robot_linear_velocity = None
        self.robot_rotational_velocity = None
        self.charging = False
        self.battery_level = 100.0
        self.publisher = rospy.Publisher("battery_level", String, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.init_node("battery_level")

    def subscribe(self):
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.store_robot_velocities)
        rospy.Subscriber("charging", Bool, self.store_robot_charging)

    def store_robot_charging(self, str_msg):
        self.charging = str_msg.data

    def store_robot_velocities(self, odom_msg):
        self.robot_linear_velocity = odom_msg.twist.twist.linear.x
        self.robot_rotational_velocity = odom_msg.twist.twist.angular.z
        self.update_battery_level()
        self.publish_battery_level()

    def update_battery_level(self):
        self.battery_level -= abs(self.robot_linear_velocity) * 0.1
        self.battery_level -= abs(self.robot_rotational_velocity) * 0.1
        if self.charging is True:
            self.battery_level += 0.01
        if self.battery_level < 0:
            self.battery_level = 0
        if self.battery_level > 100.0:
            self.battery_level = 100.0

    def publish_battery_level(self):
        str_msg = String()
        str_msg.data = str(self.battery_level)
        self.publisher.publish(str_msg)

    def monitor(self):
        while not rospy.is_shutdown():
            # cmd_vel_msg = Twist()
            # cmd_vel_msg.linear.x = 1
            # cmd_vel_msg.angular.z = 1
            # self.cmd_vel_publisher.publish(cmd_vel_msg)
            print(str(self.battery_level))


if __name__ == '__main__':
    battery_level = BatteryLevel()
    battery_level.subscribe()
    battery_level.monitor()
