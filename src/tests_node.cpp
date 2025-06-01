#include "hex_final_urdf_description/two_leg_gait.h"
#include "hex_final_urdf_description/one_leg_gait.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_test_node");
    ros::NodeHandle nh;

    hexapod::TwoLegGait gait(nh);
    gait.initialize();
    gait.execute();
    gait.stop();

    // Czekaj na wiadomości
    ros::spin();

    return 0;
}