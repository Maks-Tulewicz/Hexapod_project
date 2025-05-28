// src/gait_manager.cpp
#include "hex_controller/gait_manager.hpp"

namespace hex_controller
{

    GaitManager::GaitManager(ros::NodeHandle &nh)
        : nh_(nh)
    {
        // Domyślnie zaczynamy od SingleLegGait
        current_gait_ = std::make_unique<SingleLegGait>(nh_);

        // Subskrybuj tematy
        gait_mode_sub_ = nh_.subscribe<std_msgs::Int32>("/hex/gait_mode", 1,
                                                        &GaitManager::gaitModeCallback, this);
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1,
                                                           &GaitManager::cmdVelCallback, this);
        stand_sub_ = nh_.subscribe<std_msgs::Empty>("/hex/stand_command", 1,
                                                    &GaitManager::standCallback, this);
    }

    std::unique_ptr<GaitController> GaitManager::createGait(GaitMode mode)
    {
        switch (mode)
        {
        case GaitMode::SINGLE:
            return std::make_unique<SingleLegGait>(nh_);
        case GaitMode::TWO_LEG:
            return std::make_unique<BipodGait>(nh_);
        case GaitMode::THREE_LEG:
            return std::make_unique<TripodGait>(nh_);
        default:
            ROS_ERROR("Unknown gait mode");
            return nullptr;
        }
    }

    void GaitManager::gaitModeCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        auto mode = static_cast<GaitMode>(msg->data);
        auto new_gait = createGait(mode);
        if (new_gait)
        {
            current_gait_ = std::move(new_gait);
            ROS_INFO("Switched to gait mode: %d", msg->data);
        }
    }

    void GaitManager::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (current_gait_)
        {
            current_gait_->step(*msg);
        }
    }

    void GaitManager::standCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (current_gait_)
        {
            current_gait_->standUp();
        }
    }

} // namespace hex_controller