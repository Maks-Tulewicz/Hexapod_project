#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include "hex_controller/gait_controller.hpp"

namespace hex_controller
{

    class TripodGait : public GaitController
    {
    public:
        explicit TripodGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveLegGroup(int group, const geometry_msgs::Twist &cmd_vel, bool lift);
        std::vector<std::vector<int>> leg_groups_;
    };

} // namespace hex_controller

#endif // TRIPOD_GAIT_HPPs