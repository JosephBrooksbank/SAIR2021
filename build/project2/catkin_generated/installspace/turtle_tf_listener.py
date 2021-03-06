#!/usr/bin/env python2

import rospy
import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    post = rospy.Publisher('turtle_position', geometry_msgs.msg.TransformStamped, queue_size="10")
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        post.publish(trans)

        rate.sleep()
