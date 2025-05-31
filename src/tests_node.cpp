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
        ros::Duration(1.0).sleep(); // Daj czas na ustabilizowanie połączeń

        // Ustawiamy bardzo bezpieczne parametry
        one_leg_gait.setStepSize(2.0, 1.5); // Minimalne ruchy
        one_leg_gait.setSpeed(2.5);         // Bardzo wolny ruch

        ROS_INFO("Starting execution...");
        if (one_leg_gait.execute())
        {
            ROS_INFO("Test completed successfully");
        }
        else
        {
            ROS_ERROR("Test failed");
            return 1;
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