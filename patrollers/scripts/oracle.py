#!/usr/bin/env python
import rospy
from datetime import datetime
from std_msgs.msg import String
# from .graph import Graph
from datetime import datetime
from math import sqrt
import os
import time
import rospkg
rospack = rospkg.RosPack()


class Node(object):
    """A node class for A* path finding"""

    def __init__(self, parent=None, vertex=None):
        self.parent = parent
        self.vertex = vertex

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.vertex == other.vertex


class Vertex(object):
    """A Vertex class for designated locations in a graph"""

    def __init__(self, vertex_name, x, y):
        self.vertex_name = vertex_name
        self.x = x
        self.y = y
        self.timer = datetime.now()
        self.neighbors = []
        self.occupied = False
        self.occupied_by = None

    def add_neighbor(self, other_vertex):
        self.neighbors.append(other_vertex)

    def __repr__(self):
        return "<Vertex_obj: " + self.vertex_name + ">"


class Edge(object):
    """An Edge class for designated paths in a graph"""

    def __init__(self, start_vertex, end_vertex):
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.distance = None
        self.occupied = False
        self.occupied_by = None

    def __repr__(self):
        return "<Edge_obj: " + self.start_vertex + "->" + self.end_vertex + ">"

    def __str__(self):
        return self.start_vertex + "->" + self.end_vertex + ": " + str(self.occupied) + "\n"


class Graph(object):
    """
    A Graph class for designated paths and location in a map
    """

    def __init__(self):
        self.__vertex_dict = {}
        self.__edge_dict = {}

    def load_from_file(self, file_name):
        """
        Loads edges and vertices of a graph from a text file
        """
        file_dir = os.path.join(rospack.get_path('patrollers'), "scripts")
        file_path = os.path.join(file_dir, file_name)
        with open(file_path) as file:
            line = file.readline()
            while line:
                data = line.split('|')
                self.add_vertex(
                    data[0],            # Vertex Name
                    float(data[1]),     # Vertex X Coordinate
                    float(data[2])      # Vertex Y Coordinate
                )
                adjacency_list = eval(data[3])
                for other_vertex in adjacency_list:
                    self.add_edge(data[0], other_vertex)

                line = file.readline()

    def vertices(self):
        """
        Returns the vertices of a graph
        """
        return self.__vertex_dict

    def edges(self):
        """
        Returns the edges of a graph
        """
        return self.__edge_dict

    def add_vertex(self, vertex, vertex_x, vertex_y):
        """
        Creates new Vertex object and is added to
        the vertex dictionary with the vertex name
        as the key and the object as the value.
        """
        vertex_obj = Vertex(vertex, vertex_x, vertex_y)
        self.__vertex_dict[vertex] = vertex_obj

    def add_edge(self, start_vertex, end_vertex):
        """
        Creates new Edge object and is added to the
        list of edges in graph. End Vertex object is
        added to the neighbor list of Start Vertex object.
        """
        self.__vertex_dict[start_vertex].add_neighbor(end_vertex)
        edge_obj = Edge(start_vertex, end_vertex)
        self.__edge_dict[str((start_vertex, end_vertex))] = edge_obj

    def set_edge_distances(self):
        """
        Modifies existing Edge objects by setting the
        distance between the start vertex and end vertex.
        """
        for edge in self.__edge_dict.values():
            start_vertex_obj = self.__vertex_dict[edge.start_vertex]
            end_vertex_obj = self.__vertex_dict[edge.end_vertex]
            distance = sqrt((start_vertex_obj.x - end_vertex_obj.x) ** 2 + (start_vertex_obj.y - end_vertex_obj.y) ** 2)
            edge.distance = distance

    def set_vertex_occupied_state(self, vertex, state, robot_name):
        """
        Modifies existing Vertex objects by setting the
        occupied boolean to a specified state (True or False)
        """
        if vertex in self.__vertex_dict:
            vertex_obj = self.__vertex_dict[vertex]
            vertex_obj.occupied = state
            vertex_obj.occupied_by = robot_name
            if state is False:
                vertex_obj.timer = datetime.now()
            self.__vertex_dict[vertex] = vertex_obj

    def set_edge_occupied_state(self, edge, state, robot_name):
        """
        Modifies existing Edge objects by setting the
        occupied boolean to a specified state (True or False)
        """
        if edge in self.__edge_dict:
            edge_obj = self.__edge_dict[edge]
            edge_obj.occupied = state
            edge_obj.occupied_by = robot_name
            self.__edge_dict[edge] = edge_obj

    def __str__(self):
        res = "vertices:"
        for key, value in self.__vertex_dict.items():
            res += "\n" + str(key) + ", x: " + str(value.x) + ", y: " + str(value.y) + ": " + str(value.timer) + ", neighbors: " + str(value.neighbors)
        return res

    def a_star(self, start_vertex, end_vertex):
        """
        Returns a list of vertex names as a path from
        the given start vertex to the given end vertex.
        """

        # Create start and end node
        start_node = Node(None, start_vertex)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end_vertex)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end node
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list and add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the end node
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.vertex)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            children = []
            for neighbor in self.__vertex_dict[current_node.vertex].neighbors:
                # Create new node
                new_node = Node(current_node, neighbor)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                current_to_child_distance = self.__edge_dict[str((current_node.vertex, child.vertex))].distance
                child.g = current_node.g + current_to_child_distance
                child_vertex_obj = self.__vertex_dict[child.vertex]
                end_vertex_obj = self.__vertex_dict[end_node.vertex]
                child.h = ((child_vertex_obj.x - end_vertex_obj.x) ** 2) + ((child_vertex_obj.y - end_vertex_obj.y) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)
ROBOT_COUNT = 4


class Oracle:

    def __init__(self):
        rospy.init_node("oracle")
        time.sleep(10)
        print("Hello World. I am O.R.A.C.L.E")
        self.graph = Graph()
        self.graph.load_from_file('maplist.txt')
        self.graph.set_edge_distances()
        self.robot_topics = ["/robot_" + str(x) + "/" for x in range(ROBOT_COUNT)]
        self.robot_busy = [False for x in range(ROBOT_COUNT)]
        self.robot_msg = [None for x in range(ROBOT_COUNT)]
        self.robot_target_publishers = []
        for robot_topic in self.robot_topics:
            self.robot_target_publishers.append(rospy.Publisher(robot_topic + "target_vertex", String, queue_size=10))
        for robot_i in range(ROBOT_COUNT):
            vertex_name = "v_" + str(robot_i)
            vertex_obj = self.graph.vertices()[vertex_name]
            target_vertex_msg = String()
            target_vertex_msg.data = vertex_name + "," + vertex_name + "," + str(vertex_obj.x) + "," + str(vertex_obj.y)
            self.robot_target_publishers[robot_i].publish(target_vertex_msg)
            print("published message for robot_" + str(robot_i))
            self.graph.set_vertex_occupied_state(vertex_name, True, "robot_" + str(robot_i))

        test_pub = rospy.Publisher("/robot_0/target_vertex", String, queue_size=10)
        target_vertex_msg = String()
        target_vertex_msg.data = "v_0,v_6,0,-5"
        test_pub.publish(target_vertex_msg)

    def subscribe(self):
        rospy.Subscriber("/current_vertex", String, self.respond_to_robot, queue_size=1000)

    def respond_to_robot(self, current_vertex_msg):
        # Store message in array
        # Mark robot as not busy in array

        current_info = current_vertex_msg.data.split(",")
        current_robot, old_vertex, current_vertex, current_x, current_y = current_info[0], current_info[1], current_info[2], float(current_info[3]), float(current_info[4])
        current_robot_num = int(current_robot.split("_")[1])

        self.robot_msg[current_robot_num] = current_vertex_msg
        self.robot_busy[current_robot_num] = False
        #
        #
        #
        #
        # current_vertex_obj = self.graph.vertices()[current_vertex]
        # self.graph.set_vertex_occupied_state(current_vertex_obj.vertex_name, False, None)
        # self.graph.set_edge_occupied_state(str((old_vertex, current_vertex)), False, None)
        # self.graph.set_edge_occupied_state(str((current_vertex, old_vertex)), False, None)
        # # print(current_robot)
        #
        # current_time = datetime.now()
        # new_target_vertex_obj = None
        # for neighbor_vertex in current_vertex_obj.neighbors:
        #     neighbor_vertex_obj = self.graph.vertices()[neighbor_vertex]
        #     if neighbor_vertex_obj.occupied is False:
        #         edge_to, edge_from = str((current_vertex, neighbor_vertex)), str((neighbor_vertex, current_vertex))
        #         edge_to_obj, edge_from_obj = self.graph.edges()[edge_to], self.graph.edges()[edge_from]
        #         if edge_to_obj.occupied is False and edge_from_obj.occupied is False:
        #             # print(neighbor_vertex_obj, current_time - neighbor_vertex_obj.timer)
        #             if new_target_vertex_obj:
        #                 # print(neighbor_vertex_obj, current_time - neighbor_vertex_obj.timer, current_time - new_target_vertex_obj.timer)
        #                 if current_time - neighbor_vertex_obj.timer > current_time - new_target_vertex_obj.timer:
        #                     new_target_vertex_obj = neighbor_vertex_obj
        #             else:
        #                 new_target_vertex_obj = neighbor_vertex_obj
        #
        # # If a new target location has not been found, stay in the same location
        # if new_target_vertex_obj is None:
        #     new_target_vertex_obj = current_vertex_obj
        #
        # self.graph.set_vertex_occupied_state(new_target_vertex_obj.vertex_name, True, current_robot)
        # self.graph.set_edge_occupied_state(current_vertex_obj.vertex_name + "," + new_target_vertex_obj.vertex_name, True, current_robot)
        # self.graph.set_edge_occupied_state(new_target_vertex_obj.vertex_name + "," + current_vertex_obj.vertex_name, True, current_robot)
        #
        # # Publish new target to robot
        # current_robot_num = int(current_robot.split("_")[1])
        # new_target_vertex_msg = String()
        # new_target_vertex_msg.data = current_vertex_obj.vertex_name + "," + new_target_vertex_obj.vertex_name \
        #                              + "," + str(new_target_vertex_obj.x) + "," + str(new_target_vertex_obj.y)
        # print(new_target_vertex_msg.data)
        # self.robot_target_publishers[current_robot_num].publish(new_target_vertex_msg)

    def administer(self):
        while not rospy.is_shutdown():
            # Iterate through all messages from robot
            for current_vertex_msg in self.robot_msg:
                if current_vertex_msg:
                    current_info = current_vertex_msg.data.split(",")
                    current_robot, old_vertex, current_vertex, current_x, current_y = current_info[0], current_info[1], current_info[2], float(current_info[3]), float(current_info[4])
                    current_robot_num = int(current_robot.split("_")[1])

                    if self.robot_busy[current_robot_num] is False:
                        current_vertex_obj = self.graph.vertices()[current_vertex]
                        self.graph.set_vertex_occupied_state(current_vertex_obj.vertex_name, False, None)
                        self.graph.set_edge_occupied_state(str((old_vertex, current_vertex)), False, None)
                        self.graph.set_edge_occupied_state(str((current_vertex, old_vertex)), False, None)
                        # print(current_robot)

                        current_time = datetime.now()
                        new_target_vertex_obj = None
                        for neighbor_vertex in current_vertex_obj.neighbors:
                            neighbor_vertex_obj = self.graph.vertices()[neighbor_vertex]
                            if neighbor_vertex_obj.occupied is False:
                                edge_to, edge_from = str((current_vertex, neighbor_vertex)), str((neighbor_vertex, current_vertex))
                                edge_to_obj, edge_from_obj = self.graph.edges()[edge_to], self.graph.edges()[edge_from]
                                if edge_to_obj.occupied is False and edge_from_obj.occupied is False:
                                    # print(neighbor_vertex_obj, current_time - neighbor_vertex_obj.timer)
                                    if new_target_vertex_obj:
                                        # print(neighbor_vertex_obj, current_time - neighbor_vertex_obj.timer, current_time - new_target_vertex_obj.timer)
                                        if current_time - neighbor_vertex_obj.timer > current_time - new_target_vertex_obj.timer:
                                            new_target_vertex_obj = neighbor_vertex_obj
                                    else:
                                        new_target_vertex_obj = neighbor_vertex_obj

                        # If a new target location has not been found, stay in the same location
                        if new_target_vertex_obj is not None:
                            self.graph.set_vertex_occupied_state(new_target_vertex_obj.vertex_name, True, current_robot)
                            self.graph.set_edge_occupied_state(current_vertex_obj.vertex_name + "," + new_target_vertex_obj.vertex_name, True, current_robot)
                            self.graph.set_edge_occupied_state(new_target_vertex_obj.vertex_name + "," + current_vertex_obj.vertex_name, True, current_robot)

                            # Publish new target to robot
                            new_target_vertex_msg = String()
                            new_target_vertex_msg.data = current_vertex_obj.vertex_name + "," + new_target_vertex_obj.vertex_name \
                                                         + "," + str(new_target_vertex_obj.x) + "," + str(new_target_vertex_obj.y)
                            print(new_target_vertex_msg.data)
                            self.robot_target_publishers[current_robot_num].publish(new_target_vertex_msg)

                            self.robot_busy[current_robot_num] = True


            # If message exists and robot is not busy
            pass


if __name__ == '__main__':
    print(os.listdir("."))
    oracle = Oracle()
    oracle.subscribe()
    oracle.administer()
