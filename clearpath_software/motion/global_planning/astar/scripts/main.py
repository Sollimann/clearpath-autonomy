#! /usr/bin/env python3

import numpy as np

""" ros """
import rospy
import cv2
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped, Quaternion, QuaternionStamped, Vector3

""" rviz """
from visualization_msgs.msg import Marker, MarkerArray

""" astar """
from astar_potential_field import AstarPotentialField


class RouteSelection:

    def __init__(self, node: str, rate: int):
        """
        :param node: name of rosnode
        :param rate: node frequency
        """
        rospy.init_node(node)
        self.rate = rospy.Rate(rate)

        # Subscribers
        self.obstacles_sub = rospy.Subscriber("/map", OccupancyGrid, self.ogrid_cb)

        # Publishers
        self.path_pub = rospy.Publisher("/global/path", Path, queue_size=10)
        self.rviz_path_pub = rospy.Publisher("/visual/global/path", Marker, queue_size=10)

        # init planner
        self.plan = AstarPotentialField(exploration_setting='8N', use_potential_field=True)

        # params
        self.ogrid_threshold = float(rospy.get_param("~ogrid_threshold", "90"))

    def ogrid_cb(self, msg):
        """
        :param occupancy_grid_map:
        :return:
        """

        self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.ogrid_cpm = 1 / msg.info.resolution

        map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.ogrid = 255 * np.greater(map, self.ogrid_threshold).astype(np.uint8)
        while True:
            self.compute_optimal_path((0, 0), (5, 30))

    def compute_optimal_path(self, start: (int, int), goal: (int, int)):

        self.optimal_path, global_cost_map = self.plan.compute_global_plan(start=start,
                                                                           goal=goal,
                                                                           map=self.ogrid)

        self.publish_rviz_path()

    def publish_path(self):
        pass

    def publish_rviz_path(self):
        marker = Marker(header=Header(stamp=rospy.Time.now(),
                                      frame_id="map"),
                        type=4,
                        scale=Vector3(x=0.2),
                        color=ColorRGBA(g=1, a=1))
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        for coord in self.optimal_path:
            p = Point(coord[0], coord[1], 0)
            marker.points.append(p)

        print(self.optimal_path)
        self.rviz_path_pub.publish(marker)


def main():
    route_selection = RouteSelection("astar_potential_field", 1)
    while not rospy.is_shutdown():
        route_selection.rate.sleep()

        rospy.spin()


if __name__ == '__main__':
    main()
