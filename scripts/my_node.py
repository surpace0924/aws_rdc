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
from gazebo_msgs.srv import SetModelState, GetModelState

# 機体性能最高速度
MAX_SPD_LINEAR = 0.225  # [m/s]
MAX_SPD_ANGULAR = 2.84  # [rad/s]

# 制御周期
LOOP_HZ = 30.0

def constrain(x, min, max):
    if x < min:
        x = min
    elif x > max:
        x = max
    return x

def constrainAbs(x, max):
    return constrain(x, -max, max)

# 自己位置を初期化
def initPose():
    pub_initialpose = rospy.Publisher('initialpose', gm.PoseWithCovarianceStamped, queue_size=100)
    rospy.sleep(1.0)
    init_pose = gm.PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.pose.orientation.w = 1
    pub_initialpose.publish(init_pose)

# 経路追従
class PurePursuitControl:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.kp_linear = 2.0
        self.kp_angular = 6.0

    # リファレンス点までの距離
    def getDistance(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]
        return math.sqrt((tx - pose[0])**2 + (ty - pose[1])**2)

    # リファレンスとのなす角
    def getAngle(self, pose, ind):
        tx = self.trajectory[ind][0]
        ty = self.trajectory[ind][1]
        return math.atan2(ty - pose[1], tx - pose[0]) - pose[2]

    # 並進速度
    def getLinear(self, pose, ind):
        return self.kp_linear * self.getDistance(pose, ind)

    # 回転速度
    def getAngular(self, pose, ind):
        return  self.kp_angular * self.getAngle(pose, ind)

is_goal = False
if __name__ == '__main__':
    rospy.init_node('my_node')

    pub_vel = rospy.Publisher('cmd_vel', gm.Twist, queue_size=1)
    pub_terget_pos = rospy.Publisher('terget_pos', gm.PoseStamped, queue_size=100)
    pub_trajectory = rospy.Publisher('trajectory_arry', gm.PoseArray, queue_size=100)

    listener = tf.TransformListener()
    gazebo_model_get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

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

    initPose()
    rospy.sleep(3.0)
    print("Start")
    rospy.sleep(0.2)

    count = 0
    rate = rospy.Rate(LOOP_HZ)
    while not rospy.is_shutdown():
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
            twist.linear.x = ppc.getLinear(now_pose, count)
            twist.angular.z = ppc.getAngular(now_pose, count)
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            pub_vel.publish(twist)
            break

        # 機体性能以上の速度が出ないように抑制
        twist.linear.x = constrainAbs(twist.linear.x, MAX_SPD_LINEAR)
        twist.angular.z = constrainAbs(twist.angular.z, MAX_SPD_ANGULAR)

        # 速度司令
        pub_vel.publish(twist)

        # ゴールまでの距離を算出
        GOAL_POS_X = 5.0
        GOAL_POS_Y = 1.3
        model_state = gazebo_model_get_state("turtlebot3_burger", 'world')
        gazebo_pos = model_state.pose.position
        distance = math.sqrt((GOAL_POS_X - gazebo_pos.x)**2 + (GOAL_POS_Y - gazebo_pos.y)**2)

        # ゴール判定
        GOAL_TOLERANCE = 0.1
        if (distance <= GOAL_TOLERANCE) and is_goal == False:
            print(count / LOOP_HZ)
            is_goal = True

        count += 1
        rate.sleep()

