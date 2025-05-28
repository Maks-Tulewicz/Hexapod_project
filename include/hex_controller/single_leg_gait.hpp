#ifndef SINGLE_LEG_GAIT_HPP
#define SINGLE_LEG_GAIT_HPP

#include "hex_controller/gait_controller.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hex_controller
{

    class SingleLegGait : public GaitController
    {
    public:
        explicit SingleLegGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        // Sekwencja ruchu nóg
        const std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};
    };

} // namespace hex_controller

#endif // SINGLE_LEG_GAIT_HPP