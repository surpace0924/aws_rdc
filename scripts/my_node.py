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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
import numpy as np

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
    # rospy.loginfo("cmd_vel [%.2f %.2f]", vel.linear.x, vel.angular.z)
    now = rospy.Time.now()
    # rospy.loginfo("now: %f", now.to_sec())



if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_trajectory = rospy.Publisher('trajectory_arry', PoseArray, queue_size=100)
    pub_terget_pos = rospy.Publisher('terget_pos', PoseStamped, queue_size=100)
    listener = tf.TransformListener()

    t = np.arange(0, 3.14*2, 0.1)
    cx = [3 * math.cos(ix) for ix in t]
    cy = [2 * math.sin(ix) for ix in t]
    trajectory = Trajectory(cx,cy)

    file_name = "/home/ryoga/catkin_ws/src/aws_rdc/scripts/route.csv"
    p2 = np.genfromtxt(file_name, delimiter=',', filling_values = 0)

    points = PoseArray()
    for i in range(len(p2)/2):
        pose = Pose()
        pose.position.x = p2[i][0]
        pose.position.y = p2[i][1]
        points.poses.append(pose)

    count = 0
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

        points.header.frame_id = 'map'
        points.header.stamp = rospy.Time.now()
        pub_trajectory.publish(points)

        terget_pos = PoseStamped()
        terget_pos.header.frame_id = 'map'
        terget_pos.header.stamp = rospy.Time.now()
        if count < len(p2)/2:
            terget_pos.pose.position.x = p2[count][0]
            terget_pos.pose.position.y = p2[count][1]
        else:
            terget_pos.pose.position.x = p2[len(p2)/2][0]
            terget_pos.pose.position.y = p2[len(p2)/2][1]

        points.poses.append(pose)
        pub_terget_pos.publish(terget_pos)



        # rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])
        count += 1
        rate.sleep()

