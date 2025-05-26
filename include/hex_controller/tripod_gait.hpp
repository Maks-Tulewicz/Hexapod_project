#ifndef TRIPOD_GAIT_HPP
#define TRIPOD_GAIT_HPP

#include "gait_controller.hpp"
#include <array>
#include <vector>

namespace hex_controller
{

    class TripodGait : public GaitController
    {
        std::array<std::vector<int>, 2> leg_groups_;

    public:
        explicit TripodGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveLegGroup(int group_id, const geometry_msgs::Twist &cmd_vel,
                          bool lift);
    };

} // namespace hex_controller

#endif // TRIPOD_GAIT_HPP