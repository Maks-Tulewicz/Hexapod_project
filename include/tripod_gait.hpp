#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include "gait_controller.hpp"

namespace hex_controller
{

    class TripodGait : public GaitController
    {
        std::vector<std::vector<int>> leg_groups_;

    public:
        explicit TripodGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveThreeLegs(const std::vector<int> &legs,
                           const geometry_msgs::Twist &cmd_vel);
    };

} // namespace hex_controller

#endif // TRIPOD_GAIT_HPP