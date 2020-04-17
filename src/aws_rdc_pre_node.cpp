#include "ros/ros.h"
#include "../../MyStdLib/MyStdLib.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aws_rdc_pre_node");
    ROS_INFO("HELLO");
    std::cout << Vector2::getAngle(Vector2(0, 0), Vector2(1, 1.73205)) * RAD_TO_DEG << std::endl;
    return 0;
}