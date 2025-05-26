#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <map>
#include <string>
#include <vector>

namespace hex_controller
{

    struct WalkingParameters
    {
        double step_length;     // długość kroku
        double step_height;     // wysokość podnoszenia nogi
        double cycle_time;      // czas jednego cyklu ruchu nogi
        double body_shift;      // przesunięcie ciała podczas przenoszenia nogi
        double standing_height; // wysokość podczas stania
        double turning_radius;  // promień skrętu
    };

    class GaitController
    {
    protected:
        ros::NodeHandle &nh_;
        std::map<std::string, ros::Publisher> joint_publishers_;
        WalkingParameters params_;

        // Stałe geometryczne
        static constexpr double L1 = 6.5;  // hip → knee
        static constexpr double L2 = 10.5; // knee → ankle
        static constexpr double L3 = 20.5; // ankle → stopa

        // Pozycje bazowe nóg
        std::map<int, std::vector<double>> base_positions_;

    public:
        explicit GaitController(ros::NodeHandle &nh);
        virtual ~GaitController() = default;

        // Inicjalizacja publisherów dla wszystkich stawów
        void initPublishers();

        // Funkcja do wykonania kroku w zadanym kierunku
        virtual void step(const geometry_msgs::Twist &cmd_vel) = 0;

        // Wspólne funkcje pomocnicze
        bool computeLegIK(int leg_number, double x, double y, double z,
                          double &q1, double &q2, double &q3);
        void setLegJoints(int leg_number, double q1, double q2, double q3);
        void standUp();

    protected:
        // Funkcja do modyfikacji pozycji nogi na podstawie komendy ruchu
        void adjustLegPosition(double &x, double &y, const geometry_msgs::Twist &cmd_vel);
    };

} // namespace hex_controller

#endif // GAIT_CONTROLLER_HPP