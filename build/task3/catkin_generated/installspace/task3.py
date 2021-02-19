#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.msg import Pose
from math import radians

lastData = None


def callback(data):
    global lastData
    lastData = data


def drawer():
    post = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drawer', anonymous=True)
    rate = rospy.Rate(5)
    rospy.Subscriber('/turtle1/pose', Pose, callback)

    # helper function to move turtle, defaults to moving at 0.5m/s forward
    def call_command(r, x=0.5, a=0.0):
        command = Twist()
        command.linear.x = x
        command.angular.z = a
        for i in range(0, r):
            post.publish(command)
            rate.sleep()

    # Helper function to turn 45 degrees in either direction
    def turn45(ccw=True):
        # 5Hz for 5 cycles = 1 second of turning
        if ccw:
            call_command(5, 0, radians(45))
        else:
            call_command(5, 0, radians(-45))

    # To my eye, the picture of the D looks like it has these ratios
    def long_straight():
        call_command(30)

    def corner_straight():
        call_command(6)

    def feet_straight():
        call_command(5)

    def small_corner_straight():
        call_command(2)

    def front_vertical():
        call_command(20)

    def back_vertical():
        call_command(18)

    def corner():
        turn45()
        corner_straight()
        turn45()

    def turn90(ccw=True):
        turn45(ccw)
        turn45(ccw)

    # Bottom
    long_straight()

    corner()

    # front of D
    front_vertical()

    corner()

    # Top
    long_straight()

    turn90()

    feet_straight()

    turn90()

    small_corner_straight()

    turn90(False)

    back_vertical()

    turn90(False)

    small_corner_straight()

    turn90()

    feet_straight()


if __name__ == '__main__':
    drawer()
