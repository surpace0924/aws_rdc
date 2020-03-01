#!/usr/bin/env python
import roslib
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

class Trajectory:
    def __init__(self, x, y):
        tmp_num = 0
        tmp_pos = list(range(0))
        for i in range(len(x)):
            tmp_pos.append([x[i], y[i]])
            tmp_num += 1

        self.pos = np.array(tmp_pos)
        self.num = tmp_num
        self.id = None
        self.selected_id = 0;


def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def callback(vel):
    rospy.loginfo("cmd_vel [%.2f %.2f]", vel.linear.x, vel.angular.z)
    now = rospy.Time.now()
    rospy.loginfo("now: %f", now.to_sec())


if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_trajectory = rospy.Publisher('trajectory_arry', PoseArray, queue_size=100)
    listener = tf.TransformListener()

    t = np.arange(0, 3.14*2, 0.1)
    cx = [3 * math.cos(ix) for ix in t]
    cy = [2 * math.sin(ix) for ix in t]
    trajectory = Trajectory(cx,cy)


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (linear, angular) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        twist = Twist()
        twist.linear.x = 1.5
        twist.angular.z = -1.9 * angular[2]
        if linear[0] > 4.2:
            twist.angular.z += 2
        pub_vel.publish(twist)

        points = PoseArray()
        pose = Pose()
        points.header.frame_id = 'map'
        points.header.stamp = rospy.Time.now()

        pose.position.x = 0.5
        pose.position.y = 0.5
        pose.orientation = euler_to_quaternion(Vector3(0, 0, 0))
        points.poses.append(pose)

        pub_trajectory.publish(points)

        rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])

        rate.sleep()

