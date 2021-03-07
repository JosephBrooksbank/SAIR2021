#!/usr/bin/env python
import rospy
from enum import Enum
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates


class Actions(Enum):
    FORWARD = 0
    TURN_GOOD = 1
    TURN_LEFT = 2


class States(Enum):
    # SIDE_FRONT states
    CLOSE_CLOSE = 0
    CLOSE_FAR = 1
    GOOD_CLOSE = 2
    GOOD_FAR = 3
    FAR_CLOSE = 4
    FAR_FAR = 5
    UNKNOWN = -1


class Distances(Enum):
    CLOSE = 0
    GOOD = 1
    FAR = 2
    UNKNOWN = -1


class DistanceBreaks(Enum):
    # TODO Refine these
    CLOSE_GOOD = 0.3
    GOOD_FAR = 1.5


class Part1:
    currentState = States.UNKNOWN

    def laser_callback(self, data):
        # type: (LaserScan) -> None
        """Converts raw laser data into a current state"""
        scan_radius = 2

        def get_range(mid, radius):
            lower_edge = mid - radius
            upper_edge = mid + radius
            end_val = 0
            for i in range(lower_edge, upper_edge):
                end_val += data.ranges[i]
            end_val = end_val / (upper_edge - lower_edge)
            if end_val < DistanceBreaks.CLOSE_GOOD.value:
                rospy.loginfo("getting close")
                return Distances.CLOSE
            elif DistanceBreaks.CLOSE_GOOD.value < end_val < DistanceBreaks.GOOD_FAR.value:
                return Distances.GOOD
            elif end_val > DistanceBreaks.GOOD_FAR.value:
                return Distances.FAR
            else:
                return Distances.UNKNOWN

        left_val = get_range(270, scan_radius)
        front_val = get_range(0, scan_radius)
        if left_val == Distances.UNKNOWN or front_val == Distances.UNKNOWN:
            return None
        if front_val == Distances.GOOD:
            front_val = Distances.FAR
        if left_val == Distances.CLOSE:
            if front_val == Distances.CLOSE:
                self.currentState = States.CLOSE_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.CLOSE_FAR
        elif left_val == Distances.GOOD:
            if front_val == Distances.CLOSE:
                self.currentState = States.GOOD_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.GOOD_FAR
        elif left_val == Distances.FAR:
            if front_val == Distances.CLOSE:
                self.currentState = States.FAR_CLOSE
            elif front_val == Distances.FAR:
                self.currentState = States.FAR_FAR

    def __init__(self):
        rospy.init_node('project_3', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.move_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def main(self):
        init_state = ModelState()
        init_state.model_name = 'turtlebot3_burger'
        init_state.pose.position.x = 0.8
        init_state.pose.position.y = 2
        init_state.pose.orientation.w = 1
        init_state.reference_frame = 'map'

        moved = False
        while not moved:
            try:
                self.move_model(init_state)
                moved = True
            except:
                continue
        while not rospy.is_shutdown():
            rospy.loginfo(self.currentState)


if __name__ == '__main__':
    project3 = Part1()
    project3.main()
