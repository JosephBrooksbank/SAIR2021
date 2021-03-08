#!/usr/bin/env python
from math import radians
import rospy
from enum import Enum
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import numpy as np
from geometry_msgs.msg import Twist


class Actions(Enum):
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    FINISHED = 3


class States(Enum):
    # SIDE_FRONT states
    CLOSE_CLOSE = 0
    CLOSE_FAR = 1
    GOOD_CLOSE = 2
    GOOD_FAR = 3
    FAR_CLOSE = 4
    FAR_FAR = 5
    # Will have to remove this state and handle it some other way when learning
    UNKNOWN = -1


class Distances(Enum):
    CLOSE = 0
    GOOD = 1
    FAR = 2
    UNKNOWN = -1


class DistanceBreaks(Enum):
    # The distance at which 'close' turns into 'good'
    CLOSE_GOOD = 0.3
    # The distance at which 'good' turns into 'far'
    GOOD_FAR = 0.5


class Part1:

    currentState = States.UNKNOWN

    def call_command(self, r, x=0.2, a=0.0):
        command = Twist()
        command.linear.x = x
        command.angular.z = a
        for i in range(0, r):
            self.move_robot.publish(command)
            self.r.sleep()

    def forward(self):
        self.call_command(5)

    def turn_right(self):
        self.call_command(5, 0, radians(-20))
        self.call_command(5)

    def turn_left(self):
        self.call_command(5, 0, radians(20))
        self.call_command(5)

    def finished(self):
        pass

    def wait_for_models(self, data):
        # type: (ModelStates) -> None
        if len(data.name) >= 3:
            self.spawned = True

    def laser_callback(self, data):
        # type: (LaserScan) -> None
        """Converts raw laser data into a current state"""
        scan_radius = 2

        def get_range(mid):
            lower_edge = mid - scan_radius
            upper_edge = mid + scan_radius
            end_val = 0
            for i in range(lower_edge, upper_edge):
                end_val += data.ranges[i]
            end_val = end_val / (upper_edge - lower_edge)
            # Farther than readable data is still in far
            if end_val == "inf":
                end_val = DistanceBreaks.GOOD_FAR.value + 1
            if end_val < DistanceBreaks.CLOSE_GOOD.value:
                return Distances.CLOSE
            elif DistanceBreaks.CLOSE_GOOD.value < end_val < DistanceBreaks.GOOD_FAR.value:
                return Distances.GOOD
            elif end_val > DistanceBreaks.GOOD_FAR.value:
                return Distances.FAR
            else:
                return Distances.UNKNOWN

        # These magic numbers specify the degree at which to sample the laser scan, with 0 as the front of the robot
        right_val = get_range(270)
        front_val = get_range(0)
        if right_val == Distances.UNKNOWN or front_val == Distances.UNKNOWN:
            return None
        if front_val == Distances.GOOD:
            front_val = Distances.FAR

        if right_val == Distances.CLOSE:
            if front_val == Distances.CLOSE:
                self.currentState = States.CLOSE_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.CLOSE_FAR
        elif right_val == Distances.GOOD:
            if front_val == Distances.CLOSE:
                self.currentState = States.GOOD_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.GOOD_FAR
        elif right_val == Distances.FAR:
            if front_val == Distances.CLOSE:
                self.currentState = States.FAR_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.FAR_FAR

    def __init__(self):
        self.spawned = False
        self.q_table = np.zeros((len(Actions), len(States)))
        # Setting an action for each state
        self.q_table[Actions.FORWARD.value][States.GOOD_FAR.value] = 100
        # This is a corner, for part 2
        self.q_table[Actions.FINISHED.value][States.GOOD_CLOSE.value] = 100
        self.q_table[Actions.TURN_RIGHT.value][States.FAR_FAR.value] = 100
        self.q_table[Actions.TURN_LEFT.value][States.FAR_CLOSE.value] = 100
        self.q_table[Actions.TURN_LEFT.value][States.CLOSE_FAR.value] = 100
        # This is also a corner
        self.q_table[Actions.FINISHED.value][States.CLOSE_CLOSE.value] = 100

        self.action_list = {Actions.FORWARD: self.forward,
                            Actions.TURN_RIGHT: self.turn_right,
                            Actions.TURN_LEFT: self.turn_left,
                            Actions.FINISHED: self.finished
                            }

        rospy.init_node('project_3', anonymous=True)
        self.r = rospy.Rate(10)
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.wait_for_models)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.move_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def main(self):
        init_state = ModelState()
        init_state.model_name = 'turtlebot3_burger'
        init_state.pose.position.x = 0.8
        init_state.pose.position.y = 2.0
        init_state.pose.position.z = 1.0
        init_state.pose.orientation.z = 6.15
        init_state.reference_frame = 'map'

        moved = False

        while not moved:
            if not self.spawned:
                continue
            try:
                self.move_model(init_state)
                moved = True
            except:
                continue

        temp = Twist()
        temp.linear.x = 10
        self.move_robot.publish(temp)

        while not rospy.is_shutdown():
            possible_actions = self.q_table[:, self.currentState.value]
            reward = -100
            next_action = -1
            for i in range(len(possible_actions)):
                if possible_actions[i] > reward:
                    next_action = Actions(i)
                    reward = self.q_table[i, self.currentState.value]
            rospy.loginfo("%s, %s", self.currentState, next_action)
            self.action_list[next_action]()


if __name__ == '__main__':
    project3 = Part1()
    project3.main()
