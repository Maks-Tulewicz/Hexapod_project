// include/hex_controller/gait_manager.hpp
#ifndef GAIT_MANAGER_HPP
#define GAIT_MANAGER_HPP

#include "hex_controller/gait_controller.hpp"
#include "hex_controller/single_leg_gait.hpp"
#include "hex_controller/bipod_gait.hpp"
#include "hex_controller/tripod_gait.hpp"
#include <std_msgs/Int32.h>
#include <memory>

namespace hex_controller
{

    class GaitManager
    {
    public:
        explicit GaitManager(ros::NodeHandle &nh);

    private:
        std::unique_ptr<GaitController> createGait(GaitMode mode);
        void gaitModeCallback(const std_msgs::Int32::ConstPtr &msg);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void standCallback(const std_msgs::Empty::ConstPtr &msg);

        ros::NodeHandle &nh_;
        ros::Subscriber gait_mode_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber stand_sub_;

        std::unique_ptr<GaitController> current_gait_;
    };

} // namespace hex_controller

#endif // GAIT_MANAGER_HPP