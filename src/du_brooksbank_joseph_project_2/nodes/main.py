#!/usr/bin/env python
import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
import nav_msgs.msg

from visualization_msgs.msg import MarkerArray, Marker

from map import Map, FrontierMap, ClusterMap


class Project2:
    """ Class for running project 2"""

    def __init__(self):

        rospy.init_node('project_2', anonymous=True)

        # Latest information from Map Topic
        self.current_raw_map = []

        # Publishing bot position
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)

        self.pos_post = rospy.Publisher('turtle_position', geometry_msgs.msg.TransformStamped, queue_size=10)
        self.map_post = rospy.Publisher('/frontiers_map', nav_msgs.msg.OccupancyGrid, queue_size=10)
        self.marker_post = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

        # Simple Action Client for moving to position
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.map_to_frontier_map_callback)

    def main(self):
        """ Main method for running project 2 """

        # FROM PART 1: Not currently in use
        # first_goal = generate_goal(-1.0, 1.0, 1.0)
        # client.send_goal(first_goal)
        # client.wait_for_result()
        # second_goal = generate_goal(-1.0, -1.0, 1.0)
        # client.send_goal(second_goal)
        # client.wait_for_result()

        while not rospy.is_shutdown():

            # noinspection PyBroadException
            try:
                trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            except:
                self.rate.sleep()
                continue

            self.pos_post.publish(trans)
            self.rate.sleep()

    def map_to_frontier_map_callback(self, data):
        """ Callback to convert /map topic info into frontier exploration data """

        if data != self.current_raw_map:
            self.current_raw_map = data
            frontier_map = FrontierMap(Map(data=self.current_raw_map))
            cluster_map = ClusterMap(frontier_map)

            # Clearing canvas
            delete_message = Marker()
            delete_message.action = 3  # Action 3: Delete All
            self.marker_post.publish([delete_message])

            # Publishing data
            self.map_post.publish(frontier_map.to_occupancy_grid())
            self.marker_post.publish(cluster_map.to_markers())


if __name__ == '__main__':
    project2 = Project2()
    project2.main()
