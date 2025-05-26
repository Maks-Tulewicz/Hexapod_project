#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include "hex_controller/gait_controller.hpp"
#include "hex_controller/tripod_gait.hpp"
#include "hex_controller/bipod_gait.hpp"

class HexapodController
{
private:
    ros::NodeHandle nh_;
    std::unique_ptr<hex_controller::GaitController> gait_controller_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber gait_command_sub_;
    ros::Subscriber stand_command_sub_;

public:
    HexapodController() : nh_("~")
    {
        // Domyślnie zaczynamy z chodem trójnożnym
        gait_controller_ = std::make_unique<hex_controller::TripodGait>(nh_);

        // Subskrybuj tematy
        cmd_vel_sub_ = nh_.subscribe("/hex/cmd_vel", 1,
                                     &HexapodController::cmdVelCallback, this);
        gait_command_sub_ = nh_.subscribe("/hex/gait_command", 1,
                                          &HexapodController::gaitCommandCallback, this);
        stand_command_sub_ = nh_.subscribe("/hex/stand_command", 1,
                                           &HexapodController::standCommandCallback, this);

        ROS_INFO("Hexapod controller initialized");
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (gait_controller_)
        {
            gait_controller_->step(*msg);
        }
    }

    void gaitCommandCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "tripod")
        {
            gait_controller_ = std::make_unique<hex_controller::TripodGait>(nh_);
            ROS_INFO("Switched to tripod gait");
        }
        else if (msg->data == "bipod")
        {
            gait_controller_ = std::make_unique<hex_controller::BipodGait>(nh_);
            ROS_INFO("Switched to bipod gait");
        }
    }

    void standCommandCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (gait_controller_)
        {
            gait_controller_->standUp();
            ROS_INFO("Standing up");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_controller");
    HexapodController controller;
    ros::spin();
    return 0;
}