#!/usr/bin/env python
# coding: utf-8

import os
import roslib
import rospy
import math
import numpy as np
import tf
import turtlesim.srv
import numpy as np
import geometry_msgs.msg as gm

def constrain(x, min, max):
    if x < min:
        x = min
    elif x > max:
        x = max
    return x

def euler_to_quaternion(euler):
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return gm.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class PurePursuitControl:
    def __init__(self, trajectory):
            self.trajectory = trajectory
            self.integral = 0
            self.error_angle = [0, 0]

    def getDistance(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]
        L = math.sqrt((tx - pose[0])**2 + (ty - pose[1])**2)
        return L

    def getLiner(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]
        L = math.sqrt((tx - pose[0])**2 + (ty - pose[1])**2)
        Kp = 2.0
        liner = Kp * L
        return liner

    def getOmega(self, pose, ind):
        Kp = 10.0
        Ki = 0.0
        Kd = -0.0

        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]

        self.error_angle[0] = math.atan2(ty - pose[1], tx - pose[0]) - pose[2]

        self.integral += self.error_angle[0]

        p = Kp * self.error_angle[0]
        i = Ki * self.integral
        d = Kd * (self.error_angle[1] - self.error_angle[0])
        self.error_angle[1] = self.error_angle[0]
        return p + d


if __name__ == '__main__':
    rospy.init_node('my_node')

    pub_initialpose = rospy.Publisher('initialpose', gm.PoseWithCovarianceStamped, queue_size=100)
    pub_vel = rospy.Publisher('cmd_vel', gm.Twist, queue_size=1)
    pub_trajectory = rospy.Publisher('trajectory_arry', gm.PoseArray, queue_size=100)
    pub_terget_pos = rospy.Publisher('terget_pos', gm.PoseStamped, queue_size=100)
    listener = tf.TransformListener()

    # 現在の同じディレクトリ内の「route.csv」を読み込み
    file_name = os.path.dirname(__file__) + "/route.csv"
    p2 = np.genfromtxt(file_name, delimiter=',', filling_values = 0)

    # 読み込んだ経路データをPoseArray型に変換
    points = gm.PoseArray()
    for i in range(len(p2)):
        pose = gm.Pose()
        pose.position.x = p2[i][0]
        pose.position.y = p2[i][1]
        points.poses.append(pose)

    ppc = PurePursuitControl(p2)

    init_pose = gm.PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = rospy.Time.now()
    # init_pose.pose.pose.position.x = 0
    # init_pose.pose.pose.position.y = 0
    # init_pose.pose.pose.position.z = 0
    init_pose.pose.pose.orientation.w = 1
    # init_pose.pose.covariance

    count = 0
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():

        # pub_initialpose.publish(init_pose)
        try:
            (linear, angular) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        points.header.frame_id = 'map'
        points.header.stamp = rospy.Time.now()
        pub_trajectory.publish(points)

        terget_pos = gm.PoseStamped()
        terget_pos.header.frame_id = 'map'
        terget_pos.header.stamp = rospy.Time.now()
        if count < len(p2):
            terget_pos.pose.position.x = p2[count][0]
            terget_pos.pose.position.y = p2[count][1]
        else:
            terget_pos.pose.position.x = p2[len(p2)-1][0]
            terget_pos.pose.position.y = p2[len(p2)-1][1]

        points.poses.append(pose)
        pub_terget_pos.publish(terget_pos)

        now_pose = [0,0,0]
        now_pose[0] = linear[0]
        now_pose[1] = linear[1]
        now_pose[2] = angular[2]

        twist = gm.Twist()
        if count < len(p2):
            twist.linear.x = ppc.getLiner(now_pose, count)
            twist.angular.z = ppc.getOmega(now_pose, count)
            if ppc.getDistance(now_pose, count) < 0.15:
                twist.linear.x = 0
                twist.angular.z = 0
        else:
            twist.linear.x = 0
            twist.angular.z = 0

        twist.linear.x = constrain(twist.linear.x, -0.22, 0.22)
        twist.angular.z = constrain(twist.angular.z, -2.84, 2.84)
        pub_vel.publish(twist)

        # rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])
        count += 1
        rate.sleep()

