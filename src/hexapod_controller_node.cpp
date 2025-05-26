#include <ros/ros.h>
#include <std_msgs/String.h>
#include "hex_controller/tripod_gait.hpp"
#include "hex_controller/bipod_gait.hpp"
#include "hex_controller/tripod_gait.hpp"

class HexapodController
{
    ros::NodeHandle nh_;
    ros::Subscriber gait_cmd_sub_;
    ros::Subscriber vel_cmd_sub_;

    std::unique_ptr<hex_controller::GaitController> current_gait_;

    void gaitCommandCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "tripod")
        {
            current_gait_ = std::make_unique<hex_controller::TripodGait>(nh_);
        }
        else if (msg->data == "bipod")
        {
            current_gait_ = std::make_unique<hex_controller::BipodGait>(nh_);
        }
        else if (msg->data == "tripod")
        {
            current_gait_ = std::make_unique<hex_controller::TripodGait>(nh_);
        }

        if (current_gait_)
        {
            current_gait_->standUp();
        }
    }

    void velocityCommandCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (current_gait_)
        {
            current_gait_->step(*msg);
        }
    }

public:
    HexapodController()
    {
        gait_cmd_sub_ = nh_.subscribe("gait_command", 1,
                                      &HexapodController::gaitCommandCallback, this);
        vel_cmd_sub_ = nh_.subscribe("cmd_vel", 1,
                                     &HexapodController::velocityCommandCallback, this);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_controller");
    HexapodController controller;
    ros::spin();
    return 0;
}