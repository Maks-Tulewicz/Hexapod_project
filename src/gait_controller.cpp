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
        ROS_INFO("Starting stand up sequence");

        // Najpierw ustaw wszystkie stawy na 0
        for (int i = 0; i < 6; ++i)
        {
            setLegJoints(i, 0, 0, 0);
        }
        ros::Duration(0.5).sleep();

        // Teraz podnieś robota
        for (int i = 0; i < 6; ++i)
        {
            double q1 = 0.0, q2 = 0.0, q3 = 0.0;
            if (computeLegIK(i, 0.0, 0.0, params_.standing_height, q1, q2, q3))
            {
                ROS_INFO("Leg %d IK solution: q1=%.2f, q2=%.2f, q3=%.2f", i, q1, q2, q3);
                setLegJoints(i, q1, q2, q3);
            }
            else
            {
                ROS_ERROR("Failed to compute IK for leg %d", i);
            }
            ros::Duration(0.1).sleep();
        }

        ROS_INFO("Stand up sequence completed");
        ros::Duration(1.0).sleep();
    }

    bool GaitController::computeLegIK(int leg_id, double x, double y, double z, double &q1, double &q2, double &q3)
    {
        // Podstawowe sprawdzenie zakresu
        if (z < 0)
        {
            ROS_WARN("Requested z position is below ground");
            return false;
        }

        // Stałe dla nogi (z pliku hpp)
        const double L1 = 0.065; // długość hip → knee
        const double L2 = 0.105; // długość knee → ankle
        const double L3 = 0.205; // długość ankle → stopa

        // Oblicz kąt biodra (q1)
        q1 = atan2(y, x);

        // Oblicz długość od biodra do punktu końcowego w płaszczyźnie x-y
        double r = sqrt(x * x + y * y);

        // Oblicz długość od kolana do punktu końcowego
        double d = sqrt(r * r + z * z);

        // Sprawdź czy punkt jest osiągalny
        if (d > (L2 + L3))
        {
            ROS_WARN("Position out of reach");
            return false;
        }

        // Oblicz kąty używając prawa cosinusów
        double cos_q3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        if (cos_q3 > 1 || cos_q3 < -1)
        {
            ROS_WARN("Position unreachable - cosine out of range");
            return false;
        }

        // Oblicz q3 (kąt kostki)
        q3 = acos(cos_q3);

        // Oblicz q2 (kąt kolana)
        double beta = atan2(z, r);
        double psi = acos((L2 * L2 + d * d - L3 * L3) / (2 * L2 * d));
        q2 = beta + psi;

        // Konwersja na właściwe zakresy dla kontrolerów
        if (leg_id % 2 == 0)
        { // prawa strona
            q1 = -q1;
            q2 = -q2;
            q3 = -q3;
        }

        // Dodaj korekty kątów jeśli są potrzebne
        q2 = M_PI / 2 - q2; // Korekta względem pozycji początkowej

        ROS_DEBUG("IK for leg %d: q1=%.2f, q2=%.2f, q3=%.2f", leg_id, q1, q2, q3);
        return true;
    }

    void GaitController::setLegJoints(int leg_id, double q1, double q2, double q3)
    {
        std_msgs::Float64 msg;

        // Hip joint
        msg.data = q1;
        joint_publishers_["hip_" + std::to_string(leg_id + 1)].publish(msg);
        ros::Duration(0.01).sleep();

        // Knee joint
        msg.data = q2;
        joint_publishers_["knee_" + std::to_string(leg_id + 1)].publish(msg);
        ros::Duration(0.01).sleep();

        // Ankle joint
        msg.data = q3;
        joint_publishers_["ankle_" + std::to_string(leg_id + 1)].publish(msg);
        ros::Duration(0.01).sleep();
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