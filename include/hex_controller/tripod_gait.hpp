#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include "hex_controller/gait_controller.hpp"
#include <vector>

namespace hex_controller
{

    class TripodGait : public GaitController
    {
    private:
        std::vector<std::vector<int>> leg_groups_;

    public:
        explicit TripodGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveLegGroup(int group, const geometry_msgs::Twist &cmd_vel, bool lift);
    };

} // namespace hex_controller

#endif // TRIPOD_GAIT_HPP