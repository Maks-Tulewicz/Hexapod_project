// include/hex_controller/single_leg_gait.hpp
#ifndef SINGLE_LEG_GAIT_HPP
#define SINGLE_LEG_GAIT_HPP

#include "hex_controller/gait_controller.hpp"

namespace hex_controller
{

    class SingleLegGait : public GaitController
    {
    public:
        explicit SingleLegGait(ros::NodeHandle &nh) : GaitController(nh) {}
        void step(const geometry_msgs::Twist &cmd_vel) override;
    };

} // namespace hex_controller
#endif // SINGLE_LEG_GAIT_HPP