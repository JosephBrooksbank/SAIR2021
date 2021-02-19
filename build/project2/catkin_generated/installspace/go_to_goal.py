#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# goal
x = -1.0
y = 1.0
z = 0.0

if __name__ == '__main__':
    rospy.init_node('go_to_goal')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.position.x = x
    goal.target_pose.position.y = y
    client.send_goal(goal)
