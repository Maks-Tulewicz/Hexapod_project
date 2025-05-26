#ifndef BIPOD_GAIT_HPP
#define BIPOD_GAIT_HPP

#include "hex_controller/gait_controller.hpp"
#include <vector>
#include <utility>

namespace hex_controller
{

    class BipodGait : public GaitController
    {
    private:
        std::vector<std::pair<int, int>> leg_pairs_;

    public:
        explicit BipodGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveLegPair(const std::pair<int, int> &pair, const geometry_msgs::Twist &cmd_vel);
    };

} // namespace hex_controller

#endif // BIPOD_GAIT_HPP