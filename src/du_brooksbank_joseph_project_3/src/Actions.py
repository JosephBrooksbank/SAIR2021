from math import radians

import rospy
from geometry_msgs.msg import Twist


class Actions:
    # The current state of this Action object
    current_state = -1
    # Constants (indexes) for the Q_Table
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    SHARP_LEFT = 3
    SHARP_RIGHT = 4

    def __init__(self):
        self.move_robot = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(20)
        self.action_list = {
            self.FORWARD: self.forward,
            self.TURN_LEFT: self.turn_left,
            self.TURN_RIGHT: self.turn_right,
            self.SHARP_LEFT: self.sharp_left,
            self.SHARP_RIGHT: self.sharp_right

        }

    def __len__(self):
        return len(self.action_list)

    def call_command(self, r, x=0.2, a=0.0):
        command = Twist()
        command.linear.x = x
        command.angular.z = a
        for i in range(0, r):
            self.move_robot.publish(command)
            self.r.sleep()

    def forward(self):
        self.call_command(10, 0.5)

    def turn_right(self):
        self.call_command(10, 0.5, radians(-50))

    def turn_left(self):
        self.call_command(10, 0.5, radians(50))

    def sharp_right(self):
        self.call_command(20, 0.1, radians(-75))

    def sharp_left(self):
        self.call_command(20, 0.1, radians(75))
