#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_tests_node");
    ros::NodeHandle nh;

    ROS_INFO("Hexapod Tests Node started");

    // Na razie prosty node który tylko się uruchamia
    // Tutaj będziemy dodawać kolejne testy gaitów

    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}