#include "hex_controller/gait_controller.hpp"
#include <sensor_msgs/JointState.h>
#include <cmath>

namespace hex_controller
{

    GaitController::GaitController(ros::NodeHandle &nh) : nh_(nh)
    {
        loadParameters();
        initializePublishers();
    }

    void GaitController::loadParameters()
    {
        // Wczytaj parametry z serwera parametrów ROS
        params_.standing_height = 0.15; // domyślna wysokość stania
        params_.step_height = 0.05;     // wysokość kroku
        params_.cycle_time = 1.0;       // czas cyklu chodu
        params_.leg_x_offset = 0.2;     // przesunięcie nogi w osi X
        params_.leg_y_offset = 0.15;    // przesunięcie nogi w osi Y

        nh_.param("standing_height", params_.standing_height, params_.standing_height);
        nh_.param("step_height", params_.step_height, params_.step_height);
        nh_.param("cycle_time", params_.cycle_time, params_.cycle_time);
        nh_.param("leg_x_offset", params_.leg_x_offset, params_.leg_x_offset);
        nh_.param("leg_y_offset", params_.leg_y_offset, params_.leg_y_offset);
    }

    void GaitController::initializePublishers()
    {
        // Inicjalizacja publisherów dla każdej nogi
        for (int i = 0; i < 6; ++i)
        {
            std::string topic = "/hex/leg" + std::to_string(i) + "/command";
            leg_publishers_.push_back(
                nh_.advertise<sensor_msgs::JointState>(topic, 1));
        }
    }

    void GaitController::standUp()
    {
        // Implementacja wstawania - wszystkie nogi na tej samej wysokości
        for (int i = 0; i < 6; ++i)
        {
            double q1 = 0.0, q2 = 0.0, q3 = 0.0;
            if (computeLegIK(i, 0.0, 0.0, params_.standing_height, q1, q2, q3))
            {
                setLegJoints(i, q1, q2, q3);
            }
        }
        ros::Duration(1.0).sleep(); // Poczekaj na wykonanie ruchu
    }

    bool GaitController::computeLegIK(int leg_id, double x, double y, double z,
                                      double &q1, double &q2, double &q3)
    {
        // Implementacja odwrotnej kinematyki dla pojedynczej nogi
        // To jest uproszczona wersja - w rzeczywistości potrzebna będzie
        // bardziej zaawansowana implementacja

        // Dodaj przesunięcia specyficzne dla każdej nogi
        double base_angle = M_PI / 3.0 * leg_id;
        double local_x = x * cos(base_angle) - y * sin(base_angle);
        double local_y = x * sin(base_angle) + y * cos(base_angle);

        // Proste obliczenia IK (przykładowe)
        q1 = atan2(local_y, local_x);
        double r = sqrt(local_x * local_x + local_y * local_y);
        double h = sqrt(r * r + z * z);

        if (h > 0.3)
        { // Maksymalny zasięg nogi
            return false;
        }

        q2 = -atan2(z, r) - acos((h * h + 0.1) / (2 * h * 0.15));
        q3 = M_PI - acos((0.1) / (2 * 0.15 * 0.15));

        return true;
    }

    void GaitController::setLegJoints(int leg_id, double q1, double q2, double q3)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();

        // Dodaj nazwy stawów
        joint_state.name = {
            "coxa_joint" + std::to_string(leg_id),
            "femur_joint" + std::to_string(leg_id),
            "tibia_joint" + std::to_string(leg_id)};

        // Dodaj pozycje stawów
        joint_state.position = {q1, q2, q3};

        // Publikuj stan stawów
        leg_publishers_[leg_id].publish(joint_state);
    }

    void GaitController::adjustLegPosition(double &x, double &y,
                                           const geometry_msgs::Twist &cmd_vel)
    {
        // Dostosuj pozycję nogi na podstawie zadanej prędkości
        x = params_.leg_x_offset * cmd_vel.linear.x;
        y = params_.leg_y_offset * cmd_vel.linear.y;

        // Ograniczenie maksymalnego przesunięcia
        double max_offset = 0.1;
        x = std::max(std::min(x, max_offset), -max_offset);
        y = std::max(std::min(y, max_offset), -max_offset);
    }

} // namespace hex_controller