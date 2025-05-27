#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

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

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("hex/cmd_vel", 1);
        gait_pub_ = nh_.advertise<std_msgs::String>("hex/gait_command", 1);

        ROS_INFO("Sterowanie hexapodem:");
        ROS_INFO("------------------");
        ROS_INFO("Strzałki: ruch przód/tył/boki");
        ROS_INFO("q/e : obrót w lewo/prawo");
        ROS_INFO("1,2,3: zmiana trybu chodu");
        ROS_INFO("CTRL-C aby zakończyć");
    }

    void keyLoop()
    {
        char c;
        bool dirty = false;
        char seq[3];

        // get the console in raw mode
        struct termios cooked, raw;
        tcgetattr(STDIN_FILENO, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Sterowanie ruchem:");
        puts("Strzałki - ruch w przód/tył/boki");
        puts("q/e - obrót w miejscu");
        puts("Tryby chodu:");
        puts("1 - normalny");
        puts("2 - dwunożny");
        puts("3 - trójnożny");
        puts("CTRL-C aby zakończyć");

        std_msgs::String gait_msg;

        while (ros::ok())
        {
            // get the next event from the keyboard
            if (read(STDIN_FILENO, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            linear_x_ = linear_y_ = angular_z_ = 0;
            switch (c)
            {
            case 0x1B: // Escape sequence
                if (read(STDIN_FILENO, seq, 2) == 2)
                {
                    if (seq[0] == '[')
                    {
                        switch (seq[1])
                        {
                        case 'A': // Strzałka w górę
                            linear_x_ = l_scale_;
                            dirty = true;
                            break;
                        case 'B': // Strzałka w dół
                            linear_x_ = -l_scale_;
                            dirty = true;
                            break;
                        case 'C': // Strzałka w prawo
                            linear_y_ = -l_scale_;
                            dirty = true;
                            break;
                        case 'D': // Strzałka w lewo
                            linear_y_ = l_scale_;
                            dirty = true;
                            break;
                        }
                    }
                }
                break;
            case 'q':
                angular_z_ = a_scale_;
                dirty = true;
                break;
            case 'e':
                angular_z_ = -a_scale_;
                dirty = true;
                break;

            // Zmiana trybu chodu
            case '1':
                gait_msg.data = "normal";
                gait_pub_.publish(gait_msg);
                ROS_INFO("Zmieniono na chód normalny");
                break;
            case '2':
                gait_msg.data = "bipod";
                gait_pub_.publish(gait_msg);
                ROS_INFO("Zmieniono na chód dwunożny");
                break;
            case '3':
                gait_msg.data = "tripod";
                gait_pub_.publish(gait_msg);
                ROS_INFO("Zmieniono na chód trójnożny");
                break;
            }

            if (dirty)
            {
                twist_.linear.x = linear_x_;
                twist_.linear.y = linear_y_;
                twist_.linear.z = 0;
                twist_.angular.x = 0;
                twist_.angular.y = 0;
                twist_.angular.z = angular_z_;
                vel_pub_.publish(twist_);
                dirty = false;
            }
        }

        // restore normal terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_teleop");
    HexapodTeleop teleop;
    teleop.keyLoop();
    return 0;
}