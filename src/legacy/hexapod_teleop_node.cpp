#include "hex_controller/hexapod_teleop.hpp"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

namespace hex_controller
{
    static int kfd = 0;
    static struct termios cooked, raw;
    static bool quit_requested = false;

    void quit(int sig)
    {
        tcsetattr(kfd, TCSANOW, &cooked);
        quit_requested = true;
        ros::shutdown();
        exit(0);
    }

    HexapodTeleop::HexapodTeleop()
        : linear_x_(0.0), linear_y_(0.0), angular_(0.0), l_scale_(1.0), a_scale_(1.0), dirty_(false)
    {
        init();
    }

    void HexapodTeleop::init()
    {
        nh_.param("scale_angular", a_scale_, a_scale_);
        nh_.param("scale_linear", l_scale_, l_scale_);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        stand_pub_ = nh_.advertise<std_msgs::Empty>("/hex/stand_command", 1);
        gait_mode_pub_ = nh_.advertise<std_msgs::Int32>("/hex/gait_mode", 1);
    }

    void HexapodTeleop::keyLoop()
    {
        char c;
        static char escape_seq[3]; // Static buffer dla sekwencji escape

        // Przygotuj terminal
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys to move the hexapod.");
        puts("'u/o' - move left/right");
        puts("'r' - stand up");
        puts("'1' - single leg gait");
        puts("'2' - two leg gait");
        puts("'3' - three leg gait");
        puts("'q' - quit");

        while (!quit_requested && ros::ok())
        {
            if (read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            // Obsługa sekwencji escape dla strzałek
            if (c == '\x1b')
            {
                // Czytaj następne znaki sekwencji
                if (read(kfd, &escape_seq[0], 1) < 0)
                    break;
                if (read(kfd, &escape_seq[1], 1) < 0)
                    break;

                if (escape_seq[0] == '[')
                {
                    switch (escape_seq[1])
                    {
                    case 'A':
                        c = KEYCODE_UP;
                        break;
                    case 'B':
                        c = KEYCODE_DOWN;
                        break;
                    case 'C':
                        c = KEYCODE_RIGHT;
                        break;
                    case 'D':
                        c = KEYCODE_LEFT;
                        break;
                    default:
                        continue;
                    }
                }
            }

            updateCommandVelocity(c);

            if (dirty_)
            {
                publishVelocity();
                dirty_ = false;
            }
        }

        cleanup();
    }

    void HexapodTeleop::updateCommandVelocity(char c)
    {
        linear_x_ = linear_y_ = angular_ = 0;

        switch (c)
        {
        case KEYCODE_R:
        {
            std_msgs::Empty stand_msg;
            stand_pub_.publish(stand_msg);
            ROS_INFO("Standing up...");
            return;
        }
        case KEYCODE_1:
        case KEYCODE_2:
        case KEYCODE_3:
        {
            std_msgs::Int32 mode_msg;
            mode_msg.data = c - KEYCODE_1;
            gait_mode_pub_.publish(mode_msg);
            ROS_INFO("Switching to gait mode %d", mode_msg.data);
            return;
        }
        case KEYCODE_UP:
        case KEYCODE_I:
            linear_x_ = l_scale_;
            break;
        case KEYCODE_DOWN:
        case KEYCODE_K:
            linear_x_ = -l_scale_;
            break;
        case KEYCODE_LEFT:
        case KEYCODE_J:
            angular_ = a_scale_;
            break;
        case KEYCODE_RIGHT:
        case KEYCODE_L:
            angular_ = -a_scale_;
            break;
        case KEYCODE_U:
            linear_y_ = l_scale_;
            break;
        case KEYCODE_O:
            linear_y_ = -l_scale_;
            break;
        case KEYCODE_Q:
            quit_requested = true;
            return;
        }
        dirty_ = true;
    }

    void HexapodTeleop::publishVelocity()
    {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_x_;
        twist.linear.y = linear_y_;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angular_;

        vel_pub_.publish(twist);
    }

    void HexapodTeleop::cleanup()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

} // namespace hex_controller

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_teleop");
    hex_controller::HexapodTeleop teleop;

    signal(SIGINT, hex_controller::quit);

    teleop.keyLoop();

    return 0;
}