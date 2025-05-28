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
        explicit GaitManager(ros::NodeHandle &nh) : nh_(nh), is_standing_(false)
        {
            // Inicjalizacja subskrypcji
            gait_mode_sub_ = nh_.subscribe("/hex/gait_mode", 1,
                                           &GaitManager::gaitModeCallback, this);
            cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1,
                                         &GaitManager::cmdVelCallback, this);
            stand_sub_ = nh_.subscribe("/hex/stand_command", 1,
                                       &GaitManager::standCallback, this);

            // Domyślny tryb chodu
            current_gait_ = createGait(GaitMode::SINGLE);
        }

    private:
        std::unique_ptr<GaitController> createGait(GaitMode mode)
        {
            switch (mode)
            {
            case GaitMode::SINGLE:
                return std::unique_ptr<GaitController>(new SingleLegGait(nh_));
            case GaitMode::TWO_LEG:
                return std::unique_ptr<GaitController>(new BipodGait(nh_));
            case GaitMode::THREE_LEG:
                return std::unique_ptr<GaitController>(new TripodGait(nh_));
            default:
                return nullptr;
            }
        }

        void gaitModeCallback(const std_msgs::Int32::ConstPtr &msg);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void standCallback(const std_msgs::Empty::ConstPtr &msg);

        ros::NodeHandle &nh_;
        ros::Subscriber gait_mode_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber stand_sub_;

        std::unique_ptr<GaitController> current_gait_;
        bool is_standing_;
    };

} // namespace hex_controller
#endif // GAIT_MANAGER_HPP