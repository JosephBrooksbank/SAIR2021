#!/usr/bin/env python
import math
import sys
from math import radians
from random import Random

import rospy
from enum import Enum
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion

from Actions import Actions
from Q_Table import QTable
from State import StateManager


def distance_2_points(point1, point2):
    return math.hypot(point2[0] - point1[0], point2[1] - point1[1])


def add_pose(x, y, o, z=0.01):
    temp = Pose()
    temp.position.x = x
    temp.position.y = y
    temp.position.z = z
    temp.orientation.z = o
    return temp


class Part1:

    def wait_for_models(self, data):
        # type: (ModelStates) -> None
        if len(data.name) >= 3:
            self.spawned = True
            for i in range(len(data.name)):
                if data.name[i] == 'turtlebot3_burger':
                    self.pose = data.pose[i]

    def laser_callback(self, data):
        # type: (LaserScan) -> None
        """Converts raw laser data into a current state"""
        scan_radius = 15

        def get_range(lower, upper):

            end_val = sys.maxint
            for i in range(lower, upper):
                if data.ranges[i] < end_val:
                    end_val = data.ranges[i]
            # end_val = end_val / (upper - lower)

            return end_val

        # These magic numbers specify the degree at which to sample the laser scan, with 0 as the front of the robot
        right_val = get_range(260, 280)
        right_front_val = get_range(315, 330)
        front_val = get_range(-30, 30)
        left_val = get_range(80, 100)
        self.state_manager.get_readings(right_val, right_front_val, front_val, left_val)
        # rospy.loginfo(str(right_val) + " " + str(right_front_val) + " " + str(front_val))
        temp = ""
        for val in self.state_manager.sensors:
            temp += val.current_state + " "
        rospy.loginfo(temp)
        # rospy.loginfo(self.state_manager.right_state.current_state + " " + self.state_manager.right_front_state.current_state\
        #               + " " + self.state_manager.front_state.current_state)
        self.q_table.update_state(self.state_manager.get_state_index())

    def __init__(self):
        rospy.init_node('project_3', anonymous=True)
        self.state_manager = StateManager()

        self.pose = None
        self.spawned = False
        self.actions = Actions()
        self.q_table = QTable(self.state_manager)
        self.stuck_count = 0
        self.max_stuck = 5
        self.prev_pose = None
        self.poses = []
        self.poses.append(add_pose(0.0, 2.0, 6.15))
        # self.poses.append(add_pose(-2.0, -1.0, radians(180)))
        self.poses.append(add_pose(-1.8, -1.8, 0.0))
        # Setting an action for each state

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.wait_for_models)
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.move_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def get_pose(self):
        return [self.pose.position.x, self.pose.position.y]

    # TODO add multiple respawn points
    def reset(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        command = Twist()
        command.linear.x = 0
        command.angular.z = 0
        pub.publish(command)
        pub.unregister()
        init_state = ModelState()
        init_state.model_name = 'turtlebot3_burger'
        index = Random().randrange(0, len(self.poses))
        init_state.pose = self.poses[index]
        init_state.reference_frame = 'map'

        init_velocity = Twist()
        init_velocity.linear.x = init_velocity.linear.y = init_velocity.linear.z = 0
        init_velocity.angular.z = init_velocity.angular.x = init_velocity.angular.y = 0
        init_state.twist = init_velocity

        moved = False

        while not moved:
            if not self.spawned:
                continue
            try:
                self.move_model(init_state)
                moved = True
            except:
                continue

    def main(self):
        self.reset()
        self.prev_pose = self.get_pose()

        count = 0
        total_count = 0
        while not rospy.is_shutdown():
            while count < 10:
                self.q_table.next_action()
                count += 1
                total_count += 1
                if distance_2_points(self.get_pose(), self.prev_pose) < 0.05:
                    self.stuck_count += 1
                if self.stuck_count >= self.max_stuck or total_count > 1000:
                    rospy.loginfo("Stuck for too long! resetting...")
                    self.reset()
                    self.stuck_count = 0
                    total_count = 0
                    count = 0
                self.prev_pose = self.get_pose()
            count = 0
            self.q_table.save_table()


if __name__ == '__main__':
    project3 = Part1()
    project3.main()
