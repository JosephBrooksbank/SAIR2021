#!/usr/bin/env python
import math
import sys
from random import Random

import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Twist, Pose

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


class Project3:

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
            # type: (int, int) -> float

            end_val = sys.maxint
            for i in range(lower, upper):
                if data.ranges[i] < end_val:
                    end_val = data.ranges[i]

            return end_val

        # These magic numbers specify the degree at which to sample the laser scan, with 0 as the front of the robot
        right_val = get_range(260, 280)
        right_front_val = get_range(315, 330)
        front_val = get_range(-15, 15)
        left_val = get_range(80, 100)
        self.state_manager.get_readings(right_val, right_front_val, front_val, left_val)
        temp = ""
        for val in self.state_manager.sensors:
            temp += val.current_state + " "
        self.q_table.update_state(self.state_manager.get_state_index())

    def __init__(self):
        rospy.init_node('project_3', anonymous=True)
        self.state_manager = StateManager()

        self.Pos1 = True
        self.pose = None
        self.spawned = False
        self.actions = Actions()
        self.q_table = QTable(self.state_manager)
        self.stuck_count = 0
        self.max_stuck = 10
        self.prev_pose = None
        self.poses = []
        self.Episode = 1
        self.poses.append(add_pose(0.0, 2.0, 6.15))
        self.poses.append(add_pose(-1.8, -1.8, 0.0))

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.wait_for_models)
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.move_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def get_pose(self):
        return [self.pose.position.x, self.pose.position.y]

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
        rospy.loginfo("Episode: " + str(self.Episode))
        if self.Episode % 50 == 0:
            self.q_table.decrease_epsilon()

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
        prev_prev_pose = self.get_pose()
        while not rospy.is_shutdown():
            if self.Episode % 10 == 0:
                self.Pos1 = not self.Pos1

            while count < 10:
                if distance_2_points(self.get_pose(), prev_prev_pose) < 0.01:
                    self.stuck_count += 1
                # else:
                self.q_table.next_action()
                prev_prev_pose = self.prev_pose
                self.prev_pose = self.get_pose()
                count += 1
                total_count += 1

                if self.stuck_count >= self.max_stuck or total_count > 1000:
                    # rospy.loginfo("Stuck for too long! resetting...")
                    self.Episode += 1
                    self.reset()
                    self.stuck_count = 0
                    total_count = 0
                    count = 0

            count = 0
            self.q_table.save_table()


if __name__ == '__main__':
    project3 = Project3()
    project3.main()
