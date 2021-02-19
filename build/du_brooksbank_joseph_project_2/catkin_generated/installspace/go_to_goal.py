#!/usr/bin/env python2

import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg
import nav_msgs.msg

# Generate a goal position and orientation
def generate_goal(x, y, w):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    return goal

def callback(data):
    rospy.loginfo(data.data)


if __name__ == '__main__':
    rospy.init_node('project_2', anonymous=True)

    # Publishing bot position
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    post = rospy.Publisher('turtle_position', geometry_msgs.msg.TransformStamped, queue_size="10")
    rate = rospy.Rate(10.0)

    # Simple Action Client for moving to position
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Subscriber to map topic
    rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, callback)

    # first_goal = generate_goal(-1.0, 1.0, 1.0)
    # client.send_goal(first_goal)
    # client.wait_for_result()
    # second_goal = generate_goal(-1.0, -1.0, 1.0)
    # client.send_goal(second_goal)
    # client.wait_for_result()

    while not rospy.is_shutdown():
        # noinspection PyBroadException
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())

        except:
            rate.sleep()
            continue

        post.publish(trans)
        rate.sleep()