#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Empty.h>

namespace hex_controller
{

    struct GaitParameters
    {
        double standing_height = 0.15; // domyślna wysokość stania
        double step_height = 0.05;     // wysokość kroku
        double cycle_time = 1.0;       // czas cyklu
        double leg_x_offset = 0.1;     // przesunięcie nogi w X
        double leg_y_offset = 0.1;     // przesunięcie nogi w Y
        double turning_radius = 0.3;   // promień skrętu
    };
    struct LegOrigin
    {
        double x;
        double y;
        bool invert_hip;  // Obrót biodra - niektóre nogi mają odwrócony kierunek
        bool invert_knee; // Obrót kolana - niektóre nogi mają odwrócony kierunek
    };

    // static const std::map<int, LegOrigin> leg_origins = {
    //     {1, {0.068956, -0.077136, false, false}}, // lewa przednia
    //     {2, {-0.086608, -0.077136, true, true}},  // prawa przednia
    //     {3, {0.101174, 0.000645, false, false}},  // lewa środkowa
    //     {4, {-0.118826, -0.000645, true, true}},  // prawa środkowa
    //     {5, {0.068956, 0.078427, false, false}},  // lewa tylna
    //     {6, {-0.086608, 0.078427, true, true}}    // prawa tylna
    // };

    static const std::map<int, LegOrigin> leg_origins = {
        {1, {0.0, -0.0, false, false}}, // lewa przednia
        {2, {-0.0, -0.0, true, true}},  // prawa przednia
        {3, {0.0, 0.0, false, false}},  // lewa środkowa
        {4, {-0.0, -0.0, true, true}},  // prawa środkowa
        {5, {0.0, 0.0, false, false}},  // lewa tylna
        {6, {-0.0, 0.0, true, true}}    // prawa tylna
    };
    // W gait_controller.hpp dodaj:
    static const std::map<int, std::vector<double>> base_positions = {
        {1, {0.18, -0.15, -0.24}},  // lewa przednia
        {2, {-0.18, -0.15, -0.24}}, // prawa przednia
        {3, {0.22, 0.0, -0.24}},    // lewa środkowa
        {4, {-0.22, 0.0, -0.24}},   // prawa środkowa
        {5, {0.18, 0.15, -0.24}},   // lewa tylna
        {6, {-0.18, 0.15, -0.24}}   // prawa tylna
    };

    class GaitController
    {
    protected:
        ros::NodeHandle &nh_;
        ros::Subscriber stand_up_sub_;
        std::map<std::string, ros::Publisher> joint_publishers_;
        GaitParameters params_;

    public:
        explicit GaitController(ros::NodeHandle &nh);
        virtual ~GaitController() = default;

        virtual void step(const geometry_msgs::Twist &cmd_vel) = 0;
        virtual void standUp();

    protected:
        void initializePublishers();
        void loadParameters();
        bool computeLegIK(int leg_id, double x, double y, double z,
                          double &hip, double &knee, double &ankle);
        void setLegJoints(int leg_id, double hip, double knee, double ankle);
        void adjustLegPosition(double &x, double &y,
                               const geometry_msgs::Twist &cmd_vel);
        void standUpCallback(const std_msgs::Empty &msg);
    };

} // namespace hex_controller

#endif // GAIT_CONTROLLER_HPP