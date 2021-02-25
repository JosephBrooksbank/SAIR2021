#!/usr/bin/env python
import math

import actionlib
import geometry_msgs.msg
import nav_msgs.msg
import rospy
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from drawing_tools import DrawingTools
from map import Map, FrontierMap, ExploreMap


class Project2:
    """ Class for running project 2"""

    # How close to the goal the robot needs to be before it counts as arriving.
    TOLERANCE = 0.5

    def __init__(self):
        # type: () -> None

        rospy.init_node('project_2', anonymous=True)

        # set of tools to draw markers on screen
        self.drawing_tools = DrawingTools()

        # Current information from Map Topic
        self.current_raw_map = []  # type: [int]
        # State of exploration
        self.current_goal = None  # type: [float, float]
        # why even have namespaces for markers if we can't do selections based off of them ._.
        self.current_goal_id = None

        self.moving_to_goal = False

        self.give_up_timer = None  # type rospy.Timer (python 2 doesn't support Optional[] so I can't truly type it)

        self.cluster_map = None  # type ExploreMap

        # Publishing bot position
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)

        # Get coordinates of robot in map frame in main
        self.trans = None  # type StampedTransform

        self.pos_post = rospy.Publisher('turtle_position', geometry_msgs.msg.TransformStamped, queue_size=10)
        self.map_post = rospy.Publisher('/frontiers_map', nav_msgs.msg.OccupancyGrid, queue_size=10)
        self.marker_post = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)

        # Simple Action Client for moving to position
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.map_to_frontier_map_callback)

    def main(self):
        # type: () -> None
        """ Main method for running project 2 """

        while not rospy.is_shutdown():

            # noinspection PyBroadException
            try:
                self.trans = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            except:
                self.rate.sleep()
                continue

            # From part 1: publishing robot position
            self.pos_post.publish(self.trans)
            self.rate.sleep()

            xy = [
                self.trans.transform.translation.x,
                self.trans.transform.translation.y
            ]

            if not self.moving_to_goal:
                # If no map data yet
                if not self.cluster_map:
                    continue
                else:
                    try:

                        self.current_goal = self.cluster_map.closest_centroid(xy)
                        if not self.current_goal:  # if no goals left, exploration is done
                            # Clear markers
                            delete_message = Marker()
                            delete_message.action = 3  # Action 3: Delete All
                            self.marker_post.publish([delete_message])
                            # Stop process
                            rospy.loginfo("No more goals, exploration finished.")
                            rospy.signal_shutdown("Finished exploring")

                        else:
                            self.draw_current_goal()
                            self.try_goal(self.current_goal)
                            self.moving_to_goal = True
                            # If it tries to get to a goal for too long, try again
                            self.give_up_timer = rospy.Timer(rospy.Duration(15), self.cancel_route)

                    except rospy.ROSInterruptException:
                        rospy.loginfo("Failed to make to goal, try again")
            else:
                # Checking if robot is close enough to goal
                dist = math.hypot(xy[0] - self.current_goal[0], xy[1] - self.current_goal[1])
                if dist < Project2.TOLERANCE or self.move_client.get_result():
                    self.cancel_route()

    def map_to_frontier_map_callback(self, data):
        # type: (nav_msgs.msg.OccupancyGrid) -> None
        """ Callback to convert /map topic info into frontier exploration data """

        # If already going to a goal, don't update map
        if data != self.current_raw_map:
            self.current_raw_map = data
            frontier_map = FrontierMap(Map(data=self.current_raw_map))
            # both objects need to use the same drawing tools object, so that IDs are unique
            self.cluster_map = ExploreMap(frontier_map, self.drawing_tools)

            # Clearing canvas
            delete_message = Marker()
            delete_message.action = 3  # Action 3: Delete All
            self.marker_post.publish([delete_message])

            # Publishing data
            self.marker_post.publish(self.cluster_map.to_markers())
            self.draw_current_goal()
            self.map_post.publish(frontier_map.to_occupancy_grid())

    def cancel_route(self):
        # type: () -> None
        """ Cancels current pathfinder route and resets timer"""
        rospy.loginfo("Cancelling Route, trying again...")
        self.give_up_timer.shutdown()
        self.move_client.cancel_all_goals()
        self.moving_to_goal = False

    def draw_current_goal(self):
        # type: () -> None
        """ Deletes previous goal marker and creates new one """
        if not self.current_goal:
            return
        # This is where I would delete based on namespace, if I could :)
        if self.current_goal_id:
            delete_message = Marker()
            delete_message.action = 2  # Action 2: Delete One
            delete_message.id = self.current_goal_id
            delete_message.ns = "currentGoal"
            self.marker_post.publish([delete_message])

        marker = self.drawing_tools.make_sphere_marker(self.current_goal, 0.35, ColorRGBA(0, 1, 0, 1), "currentGoal")
        self.current_goal_id = marker.id
        self.marker_post.publish(MarkerArray([marker]))

    def try_goal(self, xy):
        # type: ([int,int]) -> None
        """ Creates a MoveBaseGoal() Object from x,y pair and attempts to navigate there"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xy[0]
        goal.target_pose.pose.position.y = xy[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.move_client.send_goal(goal)


if __name__ == '__main__':
    project2 = Project2()
    project2.main()
