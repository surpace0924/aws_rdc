#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "../../MyStdLib/MyStdLib.h"
#include "path.h"

// 機体性能最高速度
constexpr double MAX_SPD_LINEAR = 0.225; // [m/s]
constexpr double MAX_SPD_ANGULAR = 2.84; // [rad/s]

// 制御周期
constexpr double LOOP_HZ = 30.0;

template <typename T>
void publishPose2D(ros::Publisher pub, std::string frame_id, myStd::Pose2D<T> pose2d)
{
    geometry_msgs::PoseStamped gm_ps;
    gm_ps.header.frame_id = frame_id;
    gm_ps.header.stamp = ros::Time::now();
    gm_ps.pose.position.x = pose2d.x;
    gm_ps.pose.position.y = pose2d.y;
    gm_ps.pose.orientation.z = pose2d.theta;
    pub.publish(gm_ps);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aws_rdc_pre_node");
    ros::NodeHandle nh;
    ros::Rate rate(LOOP_HZ);
    geometry_msgs::Twist twist;

    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    myStd::PID<double> pid_linear(2.0, 0.0, 0.0);
    pid_linear.setMode(myStd::PID<double>::Mode::pPID);
    pid_linear.setSaturationAbs(MAX_SPD_LINEAR);

    myStd::PID<double> pid_angular(0.029, 0.0, 0.0);
    pid_angular.setMode(myStd::PID<double>::Mode::pPID);
    pid_angular.setSaturationAbs(MAX_SPD_ANGULAR);

    myStd::PurePursuitControl<double, myStd::PID<double>> ppc(path);

    ppc.setController(pid_linear, pid_angular);

    ros::Publisher pub_terget_pos = nh.advertise<geometry_msgs::PoseStamped>("/terget_pos", 100);

    ros::Publisher pub_initialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100);
    ros::Duration(1.0).sleep();
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.header.stamp = ros::Time::now();
    init_pose.pose.pose.orientation.w = 1;
    pub_initialpose.publish(init_pose);

    ros::Duration(2.0).sleep();

    tf::TransformListener listener;

    long long idx = 0;
    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        myStd::Pose2D<double> now_pose;
        now_pose.x = transform.getOrigin().x();
        now_pose.y = transform.getOrigin().y();
        now_pose.theta = transform.getRotation().getAngle();

        ppc.update(idx, now_pose, 1 / LOOP_HZ);
        myStd::Pose2D<double> cmd_vel = ppc.getControlVal();

        twist.linear.x = cmd_vel.x;
        twist.angular.z = cmd_vel.theta;

        pub_cmd_vel.publish(twist);

        publishPose2D(pub_terget_pos, "map", path[idx]);

        std::cout << now_pose << " " << path[idx] << std::endl;

        ++idx;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
