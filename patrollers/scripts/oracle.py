#!/usr/bin/env python
import rospy
from datetime import datetime
from std_msgs.msg import String
from .graph import Graph
ROBOT_COUNT = 4


class Oracle:

    def __init__(self):
        print("Hello World. I am O.R.A.C.L.E")
        self.graph = Graph()
        self.graph.load_from_file('maplist.txt')
        self.graph.set_edge_distances()
        self.robot_topics = ["/robot_" + str(x) + "/" for x in range(ROBOT_COUNT)]
        self.robot_target_publishers = []
        for robot_topic in self.robot_topics:
            self.robot_target_publishers.append(rospy.Publisher(robot_topic + "target_vertex", String, queue_size=10))
        for robot_i in range(ROBOT_COUNT):
            vertex_name = "v_" + str(robot_i)
            vertex_obj = self.graph.vertices()[vertex_name]
            target_vertex_msg = String()
            target_vertex_msg.data = vertex_name + "," + vertex_name + "," + str(vertex_obj.x) + "," + str(vertex_obj.y)
            self.robot_target_publishers[robot_i].publish(target_vertex_msg)
            self.graph.set_vertex_occupied_state(vertex_name, True, "robot_" + str(robot_i))
        rospy.init_node("oracle")

    def subscribe(self):
        for robot_topic in self.robot_topics:
            rospy.Subscriber(robot_topic + "current_vertex", String, self.respond_to_robot, queue_size=1)

    def respond_to_robot(self, current_vertex_msg):
        current_info = current_vertex_msg.data.split(",")
        old_vertex, current_vertex, current_x, current_y = current_info[0], current_info[1], float(current_info[2]), float(current_info[3])
        current_vertex_obj = self.graph.vertices()[current_vertex]
        current_robot = current_vertex_obj.occupied_by
        self.graph.set_vertex_occupied_state(current_vertex_obj.vertex_name, False, None)
        self.graph.set_edge_occupied_state(old_vertex + "->" + current_vertex, False, None)
        self.graph.set_edge_occupied_state(current_vertex + "->" + old_vertex, False, None)

        current_time = datetime.now()
        new_target_vertex_obj = None
        for neighbor_vertex in current_vertex_obj.neighbors:
            neighbor_vertex_obj = self.graph.vertices()[neighbor_vertex]
            if neighbor_vertex_obj.occupied is False:
                edge_to, edge_from = current_vertex + "->" + neighbor_vertex, neighbor_vertex + "->" + current_vertex
                edge_to_obj, edge_from_obj = self.graph.edges()[edge_to], self.graph.edges()[edge_from]
                if edge_to_obj.occupied is False and edge_from_obj.occupied is False:
                    if new_target_vertex_obj:
                        if current_time - neighbor_vertex_obj.timer > current_time - new_target_vertex_obj.timer:
                            new_target_vertex_obj = neighbor_vertex_obj
                    else:
                        new_target_vertex_obj = neighbor_vertex_obj

        # If a new target location has not been found, stay in the same location
        if new_target_vertex_obj is None:
            new_target_vertex_obj = current_vertex_obj

        self.graph.set_vertex_occupied_state(new_target_vertex_obj.vertex_name, True, current_robot)
        self.graph.set_edge_occupied_state(current_vertex_obj.vertex_name + "->" + new_target_vertex_obj.vertex_name, True, current_robot)
        self.graph.set_edge_occupied_state(new_target_vertex_obj.vertex_name + "->" + current_vertex_obj.vertex_name, True, current_robot)

        # Publish new target to robot
        current_robot_num = int(current_robot.split("_")[1])
        new_target_vertex_msg = String()
        new_target_vertex_msg.data = current_vertex_obj.vertex_name + "," + new_target_vertex_obj.vertex_name \
                                     + "," + str(new_target_vertex_obj.x) + "," + str(new_target_vertex_obj.y)
        self.robot_target_publishers[current_robot_num].publish(new_target_vertex_msg)

    def administer(self):
        while not rospy.shutdown():
            print(self.robot_topics)


if __name__ == '__main__':
    oracle = Oracle()
    oracle.administer()
