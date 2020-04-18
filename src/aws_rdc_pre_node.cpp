#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "../../MyStdLib/MyStdLib.h"

// 機体性能最高速度
constexpr double MAX_SPD_LINEAR = 0.225; // [m/s]
constexpr double MAX_SPD_ANGULAR = 2.84; // [rad/s]

// 制御周期
constexpr double LOOP_HZ = 50.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aws_rdc_pre_node");
    ros::NodeHandle nh;
    ros::Rate rate(LOOP_HZ);
    geometry_msgs::Twist vel;

    ros::Publisher pub_cmd_vel;
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    vel.linear.x = 0.5;
    vel.angular.z = 1.0;

    while (ros::ok())
    {
        ROS_INFO("HELLO");

        pub_cmd_vel.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
