from datetime import datetime
from math import sqrt


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, vertex=None):
        self.parent = parent
        self.vertex = vertex

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.vertex == other.vertex


class Vertex(object):
    def __init__(self, vertex_name, x, y):
        self.vertex_name = vertex_name
        self.x = x
        self.y = y
        self.timer = datetime.now()
        self.neighbors = []

    def add_neighbor(self, other_vertex):
        self.neighbors.append(other_vertex)

    def __repr__(self):
        return "<Vertex_obj: " + self.vertex_name + ">"


class Edge(object):
    def __init__(self, start_vertex, end_vertex):
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
        self.distance = None
        self.occupied = False

    def __repr__(self):
        return "<Edge_obj: " + self.start_vertex + "->" + self.end_vertex + ">"

    def __str__(self):
        return self.start_vertex + "->" + self.end_vertex + ": " + str(self.occupied) + "\n"


class Graph(object):

    def __init__(self):
        self.__vertex_dict = {}
        self.__edge_dict = {}

    def load_from_file(self, file_name):
        with open(file_name) as file:
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
        for edge in self.__edge_dict.values():
            start_vertex_obj = self.__vertex_dict[edge.start_vertex]
            end_vertex_obj = self.__vertex_dict[edge.end_vertex]
            distance = sqrt((start_vertex_obj.x - end_vertex_obj.x) ** 2 + (start_vertex_obj.y - end_vertex_obj.y) ** 2)
            edge.distance = distance

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

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
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


if __name__ == "__main__":
    # Graph Class Test
    graph = Graph()
    graph.load_from_file('maplist.txt')
    graph.set_edge_distances()
    print(graph.edges())
    print(graph.vertices())
    print(graph)

    # A Star Algorithm Test
    key, edge = list(graph.edges().items())[0]
    print(edge.start_vertex)
    print(edge.end_vertex)
    print(edge.distance)
    print(graph.a_star("v_6", "v_8"))

