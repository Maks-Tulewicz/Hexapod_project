#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "hex_controller/gait_controller.hpp"
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
        // Domyślne wartości
        params_.standing_height = 0.15; // 15cm
        params_.step_height = 0.05;     // 5cm
        params_.cycle_time = 1.0;       // 1 sekunda
        params_.leg_x_offset = 0.1;     // 10cm
        params_.leg_y_offset = 0.1;     // 10cm

        // Wczytaj parametry z ROS Parameter Server
        nh_.param<double>("standing_height", params_.standing_height, params_.standing_height);
        nh_.param<double>("step_height", params_.step_height, params_.step_height);
        nh_.param<double>("cycle_time", params_.cycle_time, params_.cycle_time);
        nh_.param<double>("leg_x_offset", params_.leg_x_offset, params_.leg_x_offset);
        nh_.param<double>("leg_y_offset", params_.leg_y_offset, params_.leg_y_offset);
    }

    void GaitController::initializePublishers()
    {
        // Inicjalizacja publisherów dla wszystkich stawów
        for (int leg_id = 1; leg_id <= 6; ++leg_id)
        {
            std::string hip_topic = "/hex_final_urdf/hip_joint_" + std::to_string(leg_id) + "_position_controller/command";
            std::string knee_topic = "/hex_final_urdf/knee_joint_" + std::to_string(leg_id) + "_position_controller/command";
            std::string ankle_topic = "/hex_final_urdf/ankle_joint_" + std::to_string(leg_id) + "_position_controller/command";

            joint_publishers_["hip_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(hip_topic, 1);
            joint_publishers_["knee_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(knee_topic, 1);
            joint_publishers_["ankle_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(ankle_topic, 1);
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

    void GaitController::setLegJoints(int leg_id, double hip, double knee, double ankle)
    {
        std_msgs::Float64 msg;

        // Publikuj pozycję biodra
        msg.data = hip;
        joint_publishers_["hip_" + std::to_string(leg_id + 1)].publish(msg);

        // Publikuj pozycję kolana
        msg.data = knee;
        joint_publishers_["knee_" + std::to_string(leg_id + 1)].publish(msg);

        // Publikuj pozycję kostki
        msg.data = ankle;
        joint_publishers_["ankle_" + std::to_string(leg_id + 1)].publish(msg);
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