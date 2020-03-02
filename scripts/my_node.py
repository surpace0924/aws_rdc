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

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def callback(vel):
    # rospy.loginfo("cmd_vel [%.2f %.2f]", vel.linear.x, vel.angular.z)
    now = rospy.Time.now()
    # rospy.loginfo("now: %f", now.to_sec())


class PurePursuitControl:
    def __init__(self, trajectory):
            self.trajectory = trajectory

    def getLiner(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]
        L = math.sqrt((tx - pose[0])**2 + (ty - pose[1])**2)
        Kp = 1
        liner = Kp * L
        return liner

    def getOmega(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]

        error_angle = math.atan2(ty - pose[1], tx - pose[0]) - pose[2]
        return error_angle * 5


if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_trajectory = rospy.Publisher('trajectory_arry', PoseArray, queue_size=100)
    pub_terget_pos = rospy.Publisher('terget_pos', PoseStamped, queue_size=100)
    listener = tf.TransformListener()

    # t = np.arange(0, 3.14*2, 0.1)
    # cx = [3 * math.cos(ix) for ix in t]
    # cy = [2 * math.sin(ix) for ix in t]
    # trajectory = Trajectory(cx,cy)

    file_name = "/home/ryoga/catkin_ws/src/aws_rdc/scripts/route.csv"
    p2 = np.genfromtxt(file_name, delimiter=',', filling_values = 0)

    points = PoseArray()
    for i in range(len(p2)/2):
        pose = Pose()
        pose.position.x = p2[i][0]
        pose.position.y = p2[i][1]
        points.poses.append(pose)

    ppc = PurePursuitControl(p2)

    count = 0
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (linear, angular) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



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

        print(linear)
        now_pose = [0,0,0]
        now_pose[0] = linear[0]
        now_pose[1] = linear[1]
        now_pose[2] = angular[2]
        twist = Twist()
        if count < len(p2)/2:
            twist.linear.x = ppc.getLiner(now_pose, count)
            twist.angular.z = ppc.getOmega(now_pose, count)
        else:
            twist.linear.x = ppc.getLiner(now_pose, len(p2)/2)
            twist.angular.z = ppc.getOmega(now_pose, len(p2)/2)
        pub_vel.publish(twist)


        # rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])
        count += 1
        rate.sleep()

