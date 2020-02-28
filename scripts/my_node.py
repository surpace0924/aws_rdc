#!/usr/bin/env python
import roslib
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

def callback(vel):
    rospy.loginfo("cmd_vel [%.2f %.2f]", vel.linear.x, vel.angular.z)
    now = rospy.Time.now()
    rospy.loginfo("now: %f", now.to_sec())


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

if __name__ == '__main__':
    rospy.init_node('my_node')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
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
        twist.linear.x = 0.7
        twist.angular.z = -1 * angular[2]

        pub.publish(twist)

        points = Marker()
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = Marker.ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;

        points.type = Marker.POINTS;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.g = 1.0;
        points.color.a = 1.0;

        for (uint32_t i = 0; i < 100; ++i)
        {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }

        marker_pub.publish(points);

        rospy.loginfo("pose [%.2f %.2f %.2f]", linear[0], linear[1], linear[2])

        rate.sleep()
