#! /usr/bin/env python3

""" ros """
import rospy
import cv2
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped, Quaternion, QuaternionStamped, Vector3

""" rrt star"""
import numpy as np
from rrt_star import RRTStar
from rrtstar.search_space import SearchSpace


class ObstacleAvoidance:

    def __init__(self, node: str, rate: int):
        """
        :param node: name of rosnode
        :param rate: node frequency
        """
        rospy.init_node(node)
        self.rate = rospy.Rate(rate)

        # Subscribers
        self.obstacles_sub = rospy.Subscriber("/local/obstacle_detection", PoseStamped, self.obstacles_cb)

        # Publishers
        self.path_pub = rospy.Publisher("/local/path", Path, queue_size=10)

        # params
        self.ogrid_threshold = float(rospy.get_param("~ogrid_threshold", "90"))
        self.car_width = rospy.get_param("~car_width", 0.5)

    def obstacles_cb(self, obstacles):
        """
        Expects an OccupancyGrid message.
        Stores the ogrid as numpy array
        :param msg: nav_msgs/OccupancyGrid
        """
        pass

    def nothing(self):
        pass

    def find_obstacles_in_map(self):

        self.X_dimensions = np.array([(0, 30), (0, 30)])
        self.Obstacles = np.array([(5, 5, 10, 10), (1, 2, 7, 5), (5, 5, 15, 19)])

        # Create Search Space
        X = SearchSpace(self.X_dimensions, self.Obstacles)

        x_init = (0, 0)  # starting location
        x_goal = (22, 22)  # goal location

        Q = np.array([(8, 4)])  # length of tree edges
        r = 1  # length of smallest edge to check for intersection with obstacles
        max_samples = 2024  # max number of samples to take before timing out
        rewire_count = 32  # optional, number of nearby branches to rewire
        prc = 0.1  # probability of checking for a connection to goal

        # create rrt* search
        rrt = RRTStar(X=X, Q=Q, x_init=x_init, x_goal=x_goal,
                      max_samples=max_samples, r=r, prc=prc, rewire_count=rewire_count)

        self.local_plan_body_frame = rrt.rrt_star()
        self.publish_path()
        print(self.local_plan_body_frame)

    def publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "base_link"

        for coord in self.local_plan_body_frame:
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "base_link"
            p.pose = Pose(Point(coord[0], coord[1], 0), Quaternion(0, 0, 0, 1))
            path.poses.append(p)

        self.path_pub.publish(path)

def main():
    plan = ObstacleAvoidance("motion_plan", 1)
    while not rospy.is_shutdown():
        plan.find_obstacles_in_map()
        plan.rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
