#ifndef HEXAPOD_TELEOP_HPP
#define HEXAPOD_TELEOP_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

namespace hex_controller
{

    // Kody klawiszy
    constexpr char KEYCODE_R = 0x72;
    constexpr char KEYCODE_U = 0x75;
    constexpr char KEYCODE_I = 0x69;
    constexpr char KEYCODE_O = 0x6F;
    constexpr char KEYCODE_J = 0x6A;
    constexpr char KEYCODE_K = 0x6B;
    constexpr char KEYCODE_L = 0x6C;
    constexpr char KEYCODE_Q = 0x71;
    constexpr char KEYCODE_1 = 0x31;
    constexpr char KEYCODE_2 = 0x32;
    constexpr char KEYCODE_3 = 0x33;

    // Kody strzałek
    constexpr char KEYCODE_UP = 0x41;
    constexpr char KEYCODE_DOWN = 0x42;
    constexpr char KEYCODE_RIGHT = 0x43;
    constexpr char KEYCODE_LEFT = 0x44;

    class HexapodTeleop
    {
    public:
        HexapodTeleop();
        virtual ~HexapodTeleop() = default;
        void keyLoop();

    private:
        void init();
        void cleanup();
        void updateCommandVelocity(char key);
        void publishVelocity();

        ros::NodeHandle nh_;
        ros::Publisher vel_pub_;
        ros::Publisher stand_pub_;
        ros::Publisher gait_mode_pub_;

        double linear_x_;
        double linear_y_;
        double angular_;
        double l_scale_;
        double a_scale_;

        bool dirty_; // Flaga wskazująca, czy należy opublikować nowe wartości
    };

} // namespace hex_controller

#endif // HEXAPOD_TELEOP_HPP