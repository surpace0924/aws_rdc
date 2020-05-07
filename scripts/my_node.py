#!/usr/bin/env python
# coding: utf-8

import os
import sys
import roslib
import rospy
import math
import numpy as np
import tf
import turtlesim.srv
import geometry_msgs.msg as gm
from gazebo_msgs.srv import SetModelState, GetModelState


# 機体性能最高速度
MAX_SPD_LINEAR = 0.225  # [m/s]
MAX_SPD_ANGULAR = 2.84  # [rad/s]

# 制御周期
LOOP_HZ = 30.0

is_goal = False
def main():
    rospy.init_node('my_node')
    pub_vel = rospy.Publisher('cmd_vel', gm.Twist, queue_size=1)
    pub_terget_pos = rospy.Publisher('terget_pos', gm.PoseStamped, queue_size=100)
    pub_trajectory = rospy.Publisher('trajectory_arry', gm.PoseArray, queue_size=100)

    listener = tf.TransformListener()
    gazebo_model_get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # 現在の同じディレクトリ内の「route.csv」を読み込み
    file_name = os.path.dirname(__file__) + "/route.csv"
    p2 = np.genfromtxt(file_name, delimiter=',', filling_values=0)

    path = []

    # 読み込んだ経路データをPoseArray型に変換
    points = gm.PoseArray()
    for i in range(len(p2)):
        pose = gm.Pose()
        pose.position.x = p2[i][0]
        pose.position.y = p2[i][1]
        points.poses.append(pose)
        path.append(Pose2D(p2[i][0], p2[i][1], 0))

    pid_linear_param = PID.param_t()
    pid_linear_param.gain = PID.gain_t(2, 0, 0)
    pid_linear = PID(pid_linear_param)
    pid_linear.setSaturation(-MAX_SPD_LINEAR, MAX_SPD_LINEAR)

    pid_angular_param = PID.param_t()
    pid_angular_param.gain = PID.gain_t(6, 0, 0)
    pid_angular = PID(pid_angular_param)
    pid_angular.setSaturation(-MAX_SPD_ANGULAR, MAX_SPD_ANGULAR)

    ppc_param = PPC.param_t()
    ppc_param.fbc_linear = pid_linear
    ppc_param.fbc_angular = pid_angular
    ppc = PPC(ppc_param, path)

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
            p = Pose2D(now_pose[0], now_pose[1], now_pose[2])
            ppc.update(count, p, 1 / LOOP_HZ)
            output = ppc.getControlVal()
            twist.linear.x = output.x
            twist.angular.z = output.theta
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



########## Library ##########

##
# @class Pose2D
# @brief 2次元の座標を扱う
class Pose2D():
    ##
    # @brief コンストラクタ
    # @param x: 2次元直交座標におけるx成分
    # @param y: 2次元直交座標におけるy成分
    # @param theta: 2次元直交座標における角度（向き）成分 [rad]
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x  # < 2次元直交座標におけるx成分
        self.y = y  # < 2次元直交座標におけるy成分
        self.theta = theta  # < 2次元直交座標における角度（向き）成分 [rad]

    ##
    # @brief 指定されたベクトルがこのベクトルと等しい場合にtrueを返す
    # @param v: 指定するベクトル
    def equals(self, v):
        return self == v

    ##
    # @brief 直交座標形式でこのベクトルを設定
    # @param x: 指定するベクトル
    # @param y: 指定するベクトル
    # @param theta: 指定するベクトル
    def set(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    ##
    # @brief 極座標形式でこのベクトルを設定
    # @param r: 原点からの距離
    # @param angle: 原点との角度
    # @param robottheta: ロボットの座標
    def setByPolar(self, r, angle, robottheta):
        self.x = r * np.cos(angle)
        self.y = r * np.sin(angle)
        self.theta = robottheta

    ##
    # @brief 座標oを中心にangleだけ回転
    # @param o: 回転中心の座標
    # @param angle: 回転させる角度[rad]
    def rotate(self, o, angle):
        p = Pose2D.Pose2D(self.x - o.x, self.y - o.y, self.theta)
        p.x = p.x * np.cos(angle) - p.y * np.sin(angle)
        p.y = p.x * np.sin(angle) + p.y * np.cos(angle)
        p.x += o.x
        p.y += o.y
        self.x = p.x
        self.y = p.y

    ##
    # @brief このベクターをフォーマットした文字列を返す
    # @return フォーマットした文字列
    def toString(self):
        return '(' + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + ')'

    ##
    # @brief このベクトルの長さを返す
    # @return このベクトルの長さ
    def length(self):
        return self.magnitude()

    ##
    # @brief このベクトルの長さを返す
    # @return このベクトルの長さ
    def magnitude(self):
        return np.sqrt(self.sqrMagnitude())

    ##
    # @brief このベクトルの長さの2乘を返す
    # @return このベクトルの長さ2乘
    def sqrLength(self):
        return self.qrMagnitude()

    ##
    # @brief このベクトルの長さの2乘を返す
    # @return このベクトルの長さ2乘
    def sqrMagnitude(self):
        return  self.x**2+self.y**2

    ##
    # @brief 2つのベクトルの内積を返す
    # @param a: 1つ目のベクトル
    # @param b: 2つ目のベクトル
    # @return 2つのベクトルの内積
    @staticmethod
    def getDot(a, b):
        return (a.x * b.x + a.y * b.y)

    ##
    # @brief 2つのベクトルのなす角を弧度法で返す
    # @param a: 1つ目のベクトル
    # @param b: 2つ目のベクトル
    # @return 2つのベクトルのなす角[rad]
    @staticmethod
    def getAngle(a, b):
        return np.arctan2(b.y - a.y, b.x - a.x)

    ##
    # @brief 2つのベクトルの距離を返す
    # @param a: 1つ目のベクトル
    # @param b: 2つ目のベクトル
    # @return 2つのベクトルの距離を返す
    @staticmethod
    def getDistance(a, b):
        return Pose2D(b.x - a.x, b.y - a.y, 0).magnitude()

    ##
    # @brief ベクトルaとbの間をtで線形補間
    # @param a: 1つ目のベクトル
    # @param b: 2つ目のベクトル
    # @param t: 媒介変数
    # @return 補間点
    @staticmethod
    def leap(a, b, t):
        if (t > 1):
            t = 1
        if (t < 0):
            t = 0

        v = a
        v.x += (b.x - a.x) * t
        v.y += (b.y - a.y) * t
        v.theta += (b.theta - a.theta) * t
        return v

    ##
    # @brief ベクトルの要素同士の和（スカラとの和の場合は全ての要素に対して加算）
    def __add__(self, other):
        if type(other) is Pose2D:
            return self.__class__(self.x + other.x, self.y + other.y, self.theta + other.theta)
        else:
            return self.__class__(self.x + other, self.y + other, self.theta + other)

    ##
    # @brief ベクトルの要素同士の和（スカラとの和の場合は全ての要素に対して加算）
    def __radd__(self, other):
        if type(other) is Pose2D:
            return self.__class__(other.x + self.x, other.y + self.y, other.theta + self.theta)
        else:
            return self.__class__(other + self.x, other + self.y, other + self.theta)

    ##
    # @brief ベクトルの要素同士の差（スカラとの差の場合は全ての要素に対して減算）
    def __sub__(self,  other):
        if type(other) is Pose2D:
            return self.__class__(self.x - other.x, self.y - other.y, self.theta - other.theta)
        else:
            return self.__class__(self.x - other, self.y - other, self.theta - other)

    ##
    # @brief ベクトルの要素同士の差（スカラとの差の場合は全ての要素に対して減算）
    def __rsub__(self, other):
        if type(other) is Pose2D:
            return self.__class__(other.x - self.x, other.y - self.y, other.theta - self.theta)
        else:
            return self.__class__(other - self.x, other - self.y, other - self.theta)

    ##
    # @brief 全ての要素にスカラ乗算
    # @attention ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用
    def __mul__(self, other):
        return self.__class__(self.x * other, self.y * other, self.theta * other)

    ##
    # @brief 全ての要素にスカラ乗算
    # @attention ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用
    def __rmul__(self, other):
        return self.__class__(other * self.x, other * self.y, other * self.theta)

    ##
    # @brief 全ての要素にスカラ除算
    # @attention ベクトル同士の除算は未定義
    def __truediv__(self, other):
        return self.__class__(self.x / other, self.y / other, self.theta / other)

    ##
    # @brief 全ての要素にスカラ除算
    # @attention ベクトル同士の除算は未定義
    def __rtruediv__(self, other):
        return self.__class__(other / self.x, other / self.y, other / self.theta)

    ##
    # @brief ベクトルの要素同士の和を代入（スカラとの和の場合は全ての要素に対して加算）
    def __iadd__(self, other):
        if type(other) is Pose2D:
            self.x += other.x
            self.y += other.y
            self.theta += other.theta
        else:
            self.x += other
            self.y += other
            self.theta += other

    ##
    # @brief ベクトルの要素同士の差を代入（スカラとの差の場合は全ての要素に対して減算）
    def __isub__(self, other):
        if type(other) is Pose2D:
            self.x -= other.x
            self.y -= other.y
            self.theta -= other.theta
        else:
            self.x -= other
            self.y -= other
            self.theta -= other

    ##
    # @brief 全ての要素に対してスカラ乗算して代入（ベクトル同士の乗算は未定義）
    def __imul__(self, other):
        self.x *= other
        self.y *= other
        self.theta *= other

    ##
    # @brief 全ての要素に対してスカラ除算して代入（ベクトル同士の除算は未定義）
    def __itruediv__(self, other):
        self.x /= other
        self.y /= other
        self.theta /= other

    ##
    # @brief 2つのベクトルが等しい場合にtrueを返す
    def __eq__(self, v):
        return self.x == Pose2D.x and self.y == Pose2D.y and self.theta == Pose2D.theta

    ##
    # @brief 2つのベクトルが等しい場合にfalseを返す
    def __ne__(self, other):
        return not(self == other)


##
# @class PID
# @brief PIDの計算
class PID:
    ##
    # @brief モードリスト
    class Mode:
        def __init__(self):
            self.pPID = 0  # < 位置型PID
            self.sPID = 1  # < 速度型PID
            self.PI_D = 2  # < 微分先行型PID
            self.I_PD = 3  # < 比例微分先行型PID

    ##
    # @brief ゲイン構造体
    class gain_t:
        def __init__(self, Kp=0, Ki=0, Kd=0):
            self.Kp = Kp  # < 比例ゲイン
            self.Ki = Ki  # < 積分ゲイン
            self.Kd = Kd  # < 微分ゲイン

    ##
    # @brief パラメータ構造体
    class param_t:
        def __init__(self):
            self.mode = PID.Mode().pPID   # < PIDモード
            self.gain = PID.gain_t()      # < PIDゲイン
            self.need_saturation = False  # < 出力制限を行うか
            self.output_min = 0           # < 出力制限時の最小値
            self.output_max = 0           # < 出力制限時の最大値

    ##
    # @brief コンストラクタ
    def __init__(self, param=None):
        self.__param = param
        self.__diff = [0] * 3  # 0: 現在, 1: 過去, 2: 大過去
        self.__prev_val = 0
        self.__prev_target = 0
        self.__integral = 0
        self.__output = 0

    ##
    # @brief リセット
    def reset(self):
        for i in range(len(self.__diff[i])):
            self.__diff[i] = 0
        prev_val = prev_target = 0
        integral = 0
        output = 0

    ##
    # @brief パラメータの設定
    # @param param: パラメータ構造体
    def setParam(self, param):
        self.__param = param

    ##
    # @brief ゲインの設定
    # @param gain: ゲイン構造体
    def setGain(self, gain):
        self.__param.gain = gain

    ##
    # @brief PIDモードの設定
    # @param mode: PIDモードenum
    def setMode(self, mode):
        self.__param.mode = mode

    ##
    # @brief 出力の最小，最大値の設定
    # @param min_v: 最小値
    # @param min_v: 最大値
    def setSaturation(self, min_v, max_v):
        self.__param.need_saturation = True
        self.__param.output_min = min_v
        self.__param.output_max = max_v

    ##
    # @brief 値の更新
    # @param target: 目標値
    # @param now_val: 現在値
    # @param dt: 前回この関数をコールしてからの経過時間
    def update(self, target, now_val, dt):
        self.__diff[0] = target - now_val  # 最新の偏差
        self.__integral += (self.__diff[0] + self.__diff[1]) * (dt / 2.0)  # 積分

        if (self.__param.mode == PID.Mode().pPID):
            self.__output = self.__calculate_pPID(target, now_val, dt)
        elif (self.__param.mode == PID.Mode().pPID):
            self.__output = self.__calculate_sPID(target, now_val, dt)
        elif (self.__param.mode == PID.Mode().pPID):
            self.__output = self.__calculate_PI_D(target, now_val, dt)
        elif (self.__param.mode == PID.Mode().pPID):
            self.__output = self.__calculate_I_PD(target, now_val, dt)

        # 次回ループのために今回の値を前回の値にする
        self.__diff[2] = self.__diff[1]
        self.__diff[1] = self.__diff[0]
        self.__prev_target = target
        self.__prev_val = now_val

        # ガード処理
        if (self.__param.need_saturation):
            if (self.__output > self.__param.output_max):
                self.__output = self.__param.output_max
            if (self.__output < self.__param.output_min):
                self.__output = self.__param.output_min

    ##
    # @brief 制御量（PIDの計算結果）の取得
    # @return 制御量（PIDの計算結果）
    # @attention update()を呼び出さないと値は更新されない
    def getControlVal(self):
        return self.__output

    # 位置型PID
    def __calculate_pPID(self, target, now_val, dt):
        p = self.__param.gain.Kp * self.__diff[0]
        i = self.__param.gain.Ki * self.__integral
        d = self.__param.gain.Kd * ((self.__diff[0] - self.__diff[1]) / dt)
        return p + i + d

    # 速度型PID
    def __calculate_sPID(self, target, now_val, dt):
        p = self.__param.gain.Kp * self.__diff[0] - self.__diff[1]
        i = self.__param.gain.Ki * self.__diff[0] * dt
        d = self.__param.gain.Kd * \
            (self.__diff[0] - 2 * self.__diff[1] + self.__diff[2]) / dt
        return self.__prev_val + p + i + d

    # 微分先行型PID
    def __calculate_PI_D(self, target, now_val, dt):
        p = self.__param.gain.Kp * self.__diff[0]
        i = self.__param.gain.Ki * self.__integral
        d = -self.__param.gain.Kd * ((now_val - self.__prev_val) / dt)
        return p + i + d

    # 比例微分先行型PID
    def __calculate_I_PD(self, target, now_val, dt):
        p = -self.__param.gain.Kp * now_val
        i = self.__param.gain.Ki * self.__integral
        d = -self.__param.gain.Kd * ((now_val - self.__prev_val) / dt)
        return p + i + d


##
# @class PPC
# @brief PurePursuit制御（単純追従制御）
class PPC:
    ##
    # @brief モードリスト
    class Mode:
        def __init__(self):
            self.diff = 0  # < 2DoF（差動二輪型） */
            self.omni = 1  # < 3DoF（全方位移動型） */

    ##
    # @brief パラメータ構造体
    class param_t:
        def __init__(self):
            self.mode = PPC.Mode().diff   # < モード */
            self.fbc_linear = PID()   # < 並進用のフィードバックコントローラ */
            self.fbc_angular = PID()  # < 回転用のフィードバックコントローラ */

    ##
    # @brief コンストラクタ パラメータと経路データで初期化
    # @param param: パラメータ構造体
    # @param path: 経路データ（Pose2Dのリスト）
    def __init__(self, param=None, path=None):
        self.__param = param
        self.__path = path
        self.__output = Pose2D()

    ##
    # @brief 経路データの設定
    # @param path: 経路データ
    def setPath(self, path):
        self.__path.clear()
        self.__path = path

    ##
    # @brief パラメータの設定
    # @param param: パラメータ構造体
    def setParam(self, param):
        self.__param = param

    ##
    # @brief モードの設定
    # @param mode: モードリスト
    def setMode(self, mode):
        self.__param.mode = mode

    ##
    # @brief 追従用フィードバックコントローラの設定
    # @param fbc_linear: 並進用のフィードバックコントローラ
    # @param fbc_angular: 回転用のフィードバックコントローラ
    def setController(self, fbc_linear, fbc_angular):
        self.__param.fbc_linear = fbc_linear
        self.__param.fbc_angular = fbc_angular

    ##
    # @brief 経路データを末尾に追加
    # @param path: 経路データ（Pose2Dのリスト）
    def push_back(self, path):
        for i in range(len(path)):
            self.__path.push_back(path[i])

    ##
    # @brief 値の更新
    # @param target: 経路データのインデックス
    # @param now_val: 現在値
    # @param dt: 前回この関数をコールしてからの経過時間
    def update(self, idx, now_pose, dt):
        distance = Pose2D.getDistance(now_pose, self.__path[idx])
        self.__param.fbc_linear.update(0, distance, dt)
        self.__output.x = -self.__param.fbc_linear.getControlVal()

        angle = Pose2D.getAngle(now_pose, self.__path[idx]) - now_pose.theta
        self.__param.fbc_angular.update(0, angle, dt)
        self.__output.theta = -self.__param.fbc_angular.getControlVal()

    ##
    # @brief 制御量（計算結果）の取得
    # @return 制御量（計算結果）
    # @attention update()を呼び出さないと値は更新されない
    def getControlVal(self):
        return self.__output



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



if __name__ == '__main__':
    main()
