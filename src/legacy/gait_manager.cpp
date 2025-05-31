#include "hex_controller/gait_manager.hpp"
#include <std_msgs/Int32.h>

namespace hex_controller
{
    GaitManager::GaitManager(ros::NodeHandle &nh)
        : nh_(nh), is_standing_(false)
    {
        gait_mode_sub_ = nh_.subscribe<std_msgs::Int32>("/hex/gait_mode", 1,
                                                        &GaitManager::gaitModeCallback, this);
        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1,
                                                           &GaitManager::cmdVelCallback, this);
        stand_sub_ = nh_.subscribe<std_msgs::Empty>("/hex/stand_command", 1,
                                                    &GaitManager::standCallback, this);

        current_gait_ = std::make_unique<SingleLegGait>(nh_);
    }

    void GaitManager::standCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (!is_standing_ && current_gait_)
        {
            ROS_INFO("Received stand up command");
            current_gait_->standUp();
            is_standing_ = true;
            current_gait_->setStanding(true); // Aktualizuj flagę w kontrolerze
        }
        else
        {
            ROS_INFO("Robot is already standing");
        }
    }

    void GaitManager::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (!is_standing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        if (current_gait_)
        {
            current_gait_->step(*msg);
        }
    }

    void GaitManager::gaitModeCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        auto mode = static_cast<GaitMode>(msg->data);
        auto new_gait = createGait(mode);
        if (new_gait)
        {
            bool was_standing = is_standing_; // Zapamiętaj poprzedni stan
            current_gait_ = std::move(new_gait);

            // Przekaż stan stania do nowego kontrolera
            current_gait_->setStanding(was_standing);

            // Jeśli robot już stał, nie wykonuj ponownie sekwencji wstawania
            if (!was_standing)
            {
                ROS_INFO("New gait controller initialized in sitting position");
            }

            ROS_INFO("Switched to gait mode: %d", msg->data);
        }
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
            return nullptr;
        }
    }
} // namespace hex_controller