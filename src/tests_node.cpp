#include "hex_final_urdf_description/one_leg_gait.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_test_node");
    ros::NodeHandle nh;

    hexapod::OneLegGait gait(nh);
    gait.initialize();

    // Czekaj na wiadomości
    ros::spin();

    return 0;
}