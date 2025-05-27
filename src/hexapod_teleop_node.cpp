#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h> // Dodaj ten nagłówek
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>

#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6F
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_M 0x6D
#define KEYCODE_COMMA 0x2C
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_Q 0x71
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_B 0x62
#define KEYCODE_N 0x6E
#define KEYCODE_R 0x72 // Dodaj klawisz 'r' do wstawania

class HexapodTeleop
{
public:
    HexapodTeleop();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Publisher stand_pub_; // Publisher dla komendy wstawania

    double linear_x_;
    double linear_y_;
    double angular_;
    double l_scale_;
    double a_scale_;
};

HexapodTeleop::HexapodTeleop() : linear_x_(0),
                                 linear_y_(0),
                                 angular_(0),
                                 l_scale_(1.0),
                                 a_scale_(1.0)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    stand_pub_ = nh_.advertise<std_msgs::Empty>("/hex/stand_command", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void HexapodTeleop::keyLoop()
{
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the hexapod.");
    puts("'r' - stand up");
    puts("'q' - quit");

    for (;;)
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_x_ = linear_y_ = angular_ = 0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch (c)
        {
        case KEYCODE_R: // Dodane - obsługa wstawania
            ROS_DEBUG("STAND UP");
            {
                std_msgs::Empty stand_msg;
                stand_pub_.publish(stand_msg);
            }
            break;
        case KEYCODE_I:
            linear_x_ = l_scale_;
            dirty = true;
            break;
        case KEYCODE_K:
            linear_x_ = -l_scale_;
            dirty = true;
            break;
        case KEYCODE_J:
            angular_ = a_scale_;
            dirty = true;
            break;
        case KEYCODE_L:
            angular_ = -a_scale_;
            dirty = true;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            quit(0);
            break;
            // ... (reszta case'ów bez zmian)
        }

        geometry_msgs::Twist twist;
        twist.linear.x = linear_x_;
        twist.linear.y = linear_y_;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = angular_;

        if (dirty == true)
        {
            vel_pub_.publish(twist);
            dirty = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_teleop");
    HexapodTeleop teleop;

    signal(SIGINT, quit);

    teleop.keyLoop();

    return 0;
}