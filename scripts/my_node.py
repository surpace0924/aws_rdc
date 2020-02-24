#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import Twist

def callback(vel):
    rospy.loginfo("cmd_vel [%.2f %.2f]", vel.linear.x, vel.angular.z)
    now = rospy.Time.now()
    rospy.loginfo("now: %f", now.to_sec())


if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber("/cmd_vel", Twist, callback)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (linear, angular) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])

        rate.sleep()
