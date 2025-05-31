#include <ros/ros.h>
#include "hex_final_urdf_description/one_leg_gait.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_tests_node");
    ros::NodeHandle nh;

    ROS_INFO("Hexapod Tests Node started");

    try
    {
        hexapod::OneLegGait one_leg_gait(nh);

        ROS_INFO("Initializing OneLegGait...");
        one_leg_gait.initialize();

        // Ustawiamy bezpieczniejsze parametry
        one_leg_gait.setStepSize(3.0, 2.0); // Mniejsza długość i wysokość kroku
        one_leg_gait.setSpeed(2.0);         // Wolniejszy ruch

        ROS_INFO("Executing OneLegGait test...");
        if (one_leg_gait.execute())
        {
            ROS_INFO("Test completed successfully");
        }
        else
        {
            ROS_ERROR("Test failed");
        }

        one_leg_gait.stop();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Test failed with exception: %s", e.what());
        return 1;
    }

    return 0;
}