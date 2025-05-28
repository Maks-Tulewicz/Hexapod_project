#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include "hex_controller/gait_controller.hpp"
#include "hex_controller/tripod_gait.hpp"
#include "hex_controller/bipod_gait.hpp"
#include "hex_controller/gait_manager.hpp"

class HexapodController
{
private:
    ros::NodeHandle nh_;
    std::unique_ptr<hex_controller::GaitController> current_gait_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber gait_cmd_sub_;
    ros::Subscriber stand_cmd_sub_;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (current_gait_)
        {
            current_gait_->step(*msg);
        }
    }

    void gaitCommandCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "tripod")
        {
            current_gait_ = std::make_unique<hex_controller::TripodGait>(nh_);
            ROS_INFO("Switched to tripod gait");
        }
        else if (msg->data == "bipod")
        {
            current_gait_ = std::make_unique<hex_controller::BipodGait>(nh_);
            ROS_INFO("Switched to bipod gait");
        }
    }

    void standCommandCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (current_gait_)
        {
            current_gait_->standUp();
            ROS_INFO("Standing up");
        }
    }

public:
    HexapodController()
    {
        // Domyślnie zaczynamy z chodem trójnożnym
        current_gait_ = std::make_unique<hex_controller::TripodGait>(nh_);

        // Subskrybuj tematy
        cmd_vel_sub_ = nh_.subscribe("hex/cmd_vel", 1,
                                     &HexapodController::cmdVelCallback, this);
        gait_cmd_sub_ = nh_.subscribe("hex/gait_command", 1,
                                      &HexapodController::gaitCommandCallback, this);
        stand_cmd_sub_ = nh_.subscribe("hex/stand_command", 1,
                                       &HexapodController::standCommandCallback, this);

        ROS_INFO("Hexapod controller initialized");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_controller");
    ros::NodeHandle nh;

    hex_controller::GaitManager manager(nh);

    ros::spin();

    return 0;
}