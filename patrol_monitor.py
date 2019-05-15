import rospy
import graph.py

class PatrolMonitor():

    """
    Patrol Monitor Class for publishing velocity commands to all robots. This monitor
     will track all robots movements and make specific vertex moveent command for
     individual robot actions
     
     Attributes:
         publisherList (:obj: Publishers): List of publishers for each one of the 
         robots that we include in our simulation
         graph (:obj: Graph): Graphical representation of our map loaded via a local
         Graph function and then stored in our Monitor graph variable
    """
    def __init__(self, ROBOT_COUNT, filepath):
        self.graph = graph.load_from_file(filepath)
        self.vertex_dict = graph.vertices()
        self.publisherList = []
        self.ROBOT_COUNT = ROBOT_COUNT


        # For each Robot in our simulation, we want to create a publisher to access their
        # controls and then store those publishers into a list of publishers, allowing us
        # easy access to each individual Robot's actions

        for i in self.ROBOT_COUNT:
            newPub = rospy.Publisher("/robot_" + i + "/cmd_vel", Twist, queue_size=10)

            self.publisherList.append(newPub)


    def publish_new_target(self, robot_num, next_vertex):

        """
        Publish New Target: This function takes in a specific robot and informs it
        about the location of it's next vertex to visit through a publisher
        :param robot_num: Number of Robots within the system
        :param next_vertex: Next Vertex for the robot to visit
        """

        robot_pub = self.publisherList[robot_num]

        new_target_msg = "New Target|" + next_vertex.x + "|" + next_vertex.y

        robot_pub.publish(new_target_msg)

        rospy.loginfo("Monitor Message: " + new_target_msg)

    def graph(self):
        """
        Function to return the Monitor's graph
        """
        return self.graph

    def publisher_list(self):
        """
        Function to return the list of publishers for all robots
        """
        return self.publisherList

    def __str__(self):

        graph_string = self.graph.__str__()
        return graph_string
