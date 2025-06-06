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

    namespace robot_geometry
    {
        static constexpr double L1 = 6.5;  // hip → knee
        static constexpr double L2 = 10.5; // knee→ ankle
        static constexpr double L3 = 20.5; // ankle→ stopa

        // Współczynnik konwersji
        static constexpr double CM_TO_M = 0.01;
        static constexpr double M_TO_CM = 100.0;
    }

    enum class GaitMode
    {
        SINGLE = 0,
        TWO_LEG = 1,
        THREE_LEG = 2
    };

    struct GaitParameters
    {
        double standing_height = 0.15; // domyślna wysokość stania
        double step_height = 0.05;     // wysokość kroku
        double step_length = 4.0;      // długość kroku
        double cycle_time = 1.0;       // czas cyklu
        double leg_x_offset = 0.1;     // przesunięcie nogi w X
        double leg_y_offset = 0.1;     // przesunięcie nogi w Y
        double turning_radius = 0.3;   // promień skrętu
        double body_shift = 1.0;       // przesunięcie tułowia podczas obrotu
    };
    struct LegOrigin
    {
        double x;
        double y;
        bool invert_hip;  // Obrót biodra - niektóre nogi mają odwrócony kierunek
        bool invert_knee; // Obrót kolana - niektóre nogi mają odwrócony kierunek
    };

    static const std::map<int, LegOrigin> leg_origins = {
        {1, {0.0, -0.0, false, false}}, // lewa przednia
        {2, {-0.0, -0.0, true, true}},  // prawa przednia
        {3, {0.0, 0.0, false, false}},  // lewa środkowa
        {4, {-0.0, -0.0, true, true}},  // prawa środkowa
        {5, {0.0, 0.0, false, false}},  // lewa tylna
        {6, {-0.0, 0.0, true, true}}    // prawa tylna
    };

    const std::map<int, std::vector<double>> base_positions = {
        {1, {18.0, -15.0, -24.0}},  // lewa przednia - bardziej na zewnątrz
        {2, {-18.0, -15.0, -24.0}}, // prawa przednia - bardziej na zewnątrz
        {3, {22.0, 0.0, -24.0}},    // lewa środkowa - jeszcze bardziej na zewnątrz
        {4, {-22.0, 0.0, -24.0}},   // prawa środkowa - jeszcze bardziej na zewnątrz
        {5, {18.0, 15.0, -24.0}},   // lewa tylna - bardziej na zewnątrz
        {6, {-18.0, 15.0, -24.0}}   // prawa tylna - bardziej na zewnątrz
    };

    class GaitController
    {
    protected:
        ros::NodeHandle &nh_;
        ros::Subscriber stand_up_sub_;
        std::map<std::string, ros::Publisher> joint_publishers_;
        GaitParameters params_;
        GaitMode current_mode_;
        bool is_standing_; // Zostawiamy flagę tutaj

    public:
        explicit GaitController(ros::NodeHandle &nh);
        virtual ~GaitController() = default;

        void setGaitMode(GaitMode mode) { current_mode_ = mode; }
        GaitMode getGaitMode() const { return current_mode_; }
        bool isStanding() const { return is_standing_; }

        virtual void step(const geometry_msgs::Twist &cmd_vel) = 0;
        virtual void standUp();
        void setStanding(bool standing) { is_standing_ = standing; }

    protected:
        void initializePublishers();
        void loadParameters();
        bool computeLegIK(int leg_id, double x, double y, double z,
                          double &hip, double &knee, double &ankle);
        void setLegJoints(int leg_id, double hip, double knee, double ankle);
        void adjustLegPosition(double &x, double &y,
                               const geometry_msgs::Twist &cmd_vel);
        void standUpCallback(const std_msgs::Empty::ConstPtr &msg);
    };

} // namespace hex_controller
#endif // GAIT_CONTROLLER_HPP