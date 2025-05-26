#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <signal.h>

class HexapodTeleop
{
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher gait_pub_;

    geometry_msgs::Twist twist_;
    double linear_x_, linear_y_, angular_z_;
    double l_scale_, a_scale_;

public:
    HexapodTeleop() : linear_x_(0),
                      linear_y_(0),
                      angular_z_(0)
    {
        nh_.param("scale_angular", a_scale_, 1.0);
        nh_.param("scale_linear", l_scale_, 1.0);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        gait_pub_ = nh_.advertise<std_msgs::String>("gait_command", 1);

        ROS_INFO("Sterowanie hexapodem:");
        ROS_INFO("------------------");
        ROS_INFO("Strzałki: ruch przód/tył/boki");
        ROS_INFO("q/e : obrót w lewo/prawo");
        ROS_INFO("1,2,3: zmiana trybu chodu");
        ROS_INFO("CTRL-C aby zakończyć");
    }

    void keyLoop();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_teleop");
    HexapodTeleop teleop;
    teleop.keyLoop();
    return 0;
}