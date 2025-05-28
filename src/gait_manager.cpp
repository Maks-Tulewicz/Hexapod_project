// src/gait_manager.cpp
#include "hex_controller/gait_manager.hpp"

namespace hex_controller
{

    void GaitManager::standCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (!is_standing_ && current_gait_)
        {
            ROS_INFO("Received stand up command");
            current_gait_->standUp();
            is_standing_ = true;
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
        if (auto new_gait = createGait(static_cast<GaitMode>(msg->data)))
        {
            current_gait_ = std::move(new_gait);
            if (is_standing_)
            {
                current_gait_->standUp();
            }
            ROS_INFO("Switched to gait mode: %d", msg->data);
        }
    }

} // namespace hex_controller