#!/usr/bin/env python
import math
import sys
from math import radians
import rospy
from enum import Enum
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from Actions import Actions
from Q_Table import QTable
from State import StateManager


def distance_2_points(point1, point2):
    return math.hypot(point2[0] - point1[0], point2[1] - point1[1])


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
        right_val = get_range(-120, -60)
        right_front_val = get_range(-60, -30)
        front_val = get_range(-30, 30)
        self.state_manager.get_readings(right_val, right_front_val, front_val)
        # rospy.loginfo(str(right_val) + " " + str(right_front_val) + " " + str(front_val))
        # rospy.loginfo(self.state_manager.right_state.current_state + " " + self.state_manager.right_front_state.current_state\
        #               + " " + self.state_manager.front_state.current_state)
        self.q_table.current_state = self.state_manager.get_state_index()

    def __init__(self):
        rospy.init_node('project_3', anonymous=True)
        self.state_manager = StateManager()

        self.pose = None
        self.spawned = False
        self.actions = Actions()
        self.q_table = QTable(self.state_manager)
        self.stuck_count = 0
        self.max_stuck = 40
        self.prev_pose = None
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
        # Throw away any unsaved changed to Q table
        # self.q_table.load_table()
        init_state = ModelState()
        init_state.model_name = 'turtlebot3_burger'
        init_state.pose.position.x = 0.0
        init_state.pose.position.y = 2.0
        init_state.pose.position.z = 0.1
        init_state.pose.orientation.z = 6.15
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

        num_iterations = 10
        count = 0
        total_count = 0
        while not rospy.is_shutdown():
            while count < num_iterations:
                self.q_table.next_action()
                count += 1
                total_count += 1
                if distance_2_points(self.get_pose(), self.prev_pose) < 0.01:
                    rospy.loginfo("Stuck!")
                    self.stuck_count += 1
                if self.stuck_count >= self.max_stuck or total_count > 1000:
                    rospy.loginfo("Stuck for too long! resetting...")
                    self.reset()
                    self.stuck_count = 0
                    count = 0
                    total_count = 0
                self.prev_pose = self.get_pose()

            count = 0
            self.q_table.save_table()


if __name__ == '__main__':
    project3 = Part1()
    project3.main()
