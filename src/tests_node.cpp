#include "hexapod_project/one_leg_gait.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_tests_node");
    ros::NodeHandle nh;

    // Test One Leg Gait
    {
        ROS_INFO("Testing One Leg Gait");
        hexapod::OneLegGait gait(nh);
        gait.initialize();

        // Set custom parameters if needed
        gait.setStepSize(4.5, 2.5);
        gait.setSpeed(2.0);

        // Execute the gait
        gait.execute();
    }

    // Here you can add tests for other gaits
    // Just uncomment the section you want to test
    /*
    {
        ROS_INFO("Testing Two Legs Gait");
        hexapod::TwoLegsGait gait(nh);
        gait.initialize();
        gait.execute();
    }
    */

    /*
    {
        ROS_INFO("Testing Three Legs Gait");
        hexapod::ThreeLegsGait gait(nh);
        gait.initialize();
        gait.execute();
    }
    */

    return 0;
}