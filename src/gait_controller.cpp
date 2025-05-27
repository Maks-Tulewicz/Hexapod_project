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
        for (int leg_id = 1; leg_id <= 6; ++leg_id)
        {
            // use the *same* topics as your walk_two_leg_gait_node
            std::string hip_topic = "/hip_joint_" + std::to_string(leg_id) + "_position_controller/command";
            std::string knee_topic = "/knee_joint_" + std::to_string(leg_id) + "_position_controller/command";
            std::string ankle_topic = "/ankle_joint_" + std::to_string(leg_id) + "_position_controller/command";

            joint_publishers_["hip_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(hip_topic, 1);
            joint_publishers_["knee_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(knee_topic, 1);
            joint_publishers_["ankle_" + std::to_string(leg_id)] = nh_.advertise<std_msgs::Float64>(ankle_topic, 1);
        }
        ROS_INFO("Joint publishers initialized on /<joint>_position_controller/command");
    }

    void GaitController::standUp()
    {
        ROS_INFO("Starting stand up sequence");

        // --- two-phase IK stand-up, copied from walk_two_leg_gait_node ---
        const double final_height = -0.24; // 24cm above ground
        const double start_height = -0.15; // 15cm above ground
        const int STEPS = 200;             // smoothness
        const double dt = params_.cycle_time / STEPS;

        // target foot positions in robot frame (meters)
        const std::map<int, std::vector<double>> target_pos = {
            {1, {18.0, -15.0, start_height}},  // front-left
            {2, {-18.0, -15.0, start_height}}, // front-right
            {3, {22.0, 0.0, start_height}},    // middle-left
            {4, {-22.0, 0.0, start_height}},   // middle-right
            {5, {18.0, 15.0, start_height}},   // back-left
            {6, {-18.0, 15.0, start_height}}   // back-right
        };
        // Phase 1: spread legs outward (XY from 0→target, Z fixed at start_height)
        for (int step = 0; step <= STEPS / 2 && ros::ok(); ++step)
        {
            double phase = double(step) / (STEPS / 2);
            for (auto const &[leg, tgt] : target_pos)
            {
                double x = tgt[0] * phase, y = tgt[1] * phase, z = start_height;
                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                    setLegJoints(leg, q1, q2, q3);
            }
            ros::Duration(dt).sleep();
        }
        ros::Duration(1.0).sleep();

        // Phase 2: lower body (XY fixed at target, Z from start→final)
        for (int step = 0; step <= STEPS / 2 && ros::ok(); ++step)
        {
            double phase = double(step) / (STEPS / 2);
            double z = start_height + (final_height - start_height) * phase;
            for (auto const &[leg, tgt] : target_pos)
            {
                double x = tgt[0], y = tgt[1];
                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                    setLegJoints(leg, q1, q2, q3);
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(2.0).sleep();
        ROS_INFO("Stand up sequence completed");
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
        joint_publishers_.at("hip_" + std::to_string(leg_id)).publish(msg);
        ros::Duration(0.01).sleep();

        // Knee joint
        msg.data = q2;
        joint_publishers_.at("knee_" + std::to_string(leg_id)).publish(msg);
        ros::Duration(0.01).sleep();

        // Ankle joint
        msg.data = q3;
        joint_publishers_.at("ankle_" + std::to_string(leg_id)).publish(msg);
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