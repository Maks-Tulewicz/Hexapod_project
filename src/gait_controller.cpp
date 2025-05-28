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
        stand_up_sub_ = nh_.subscribe("/hex/stand_command", 1,
                                      &GaitController::standUpCallback, this);
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

        // Najpierw ustaw wszystkie stawy na 0
        for (int leg_id = 1; leg_id <= 6; ++leg_id)
        {
            setLegJoints(leg_id, 0, 0, 0);
        }
        ros::Duration(1.0).sleep();

        const double final_height = -0.24; // Konwersja z cm na m
        const double start_height = -0.2;  // Start from a higher position
        const double STEPS = 100;          // More steps for smoother motion
        const double dt = 0.05;            // Zwiększone opóźnienie dla stabilności

        // Szersze rozstawienie nóg dla lepszej stabilności
        const std::map<int, std::vector<double>> target_positions = {
            {1, {0.14, -0.10, final_height}},  // lewa przednia
            {2, {-0.14, -0.10, final_height}}, // prawa przednia
            {3, {0.17, 0.0, final_height}},    // lewa środkowa
            {4, {-0.17, 0.0, final_height}},   // prawa środkowa
            {5, {0.14, 0.10, final_height}},   // lewa tylna
            {6, {-0.14, 0.10, final_height}}   // prawa tylna
        };

        // Faza 1: Rozszerzanie nóg
        ROS_INFO("Phase 1: Spreading legs...");
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);

            for (const auto &[leg, target] : target_positions)
            {
                // Start from center and move outward
                double x = target[0] * phase;
                double y = target[1] * phase;
                double z = start_height;

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                    ROS_DEBUG("Leg %d: x=%.3f y=%.3f z=%.3f -> q1=%.3f q2=%.3f q3=%.3f",
                              leg, x, y, z, q1, q2, q3);
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(0.5).sleep();

        // Faza 2: Opuszczanie ciała
        ROS_INFO("Phase 2: Lowering body...");
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);
            double current_height = start_height + (final_height - start_height) * phase;

            for (const auto &[leg, target] : target_positions)
            {
                double q1, q2, q3;
                if (computeLegIK(leg, target[0], target[1], current_height, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                    ROS_DEBUG("Leg %d: x=%.3f y=%.3f z=%.3f -> q1=%.3f q2=%.3f q3=%.3f",
                              leg, target[0], target[1], current_height, q1, q2, q3);
                }
            }
            ros::Duration(dt).sleep();
        }
        is_standing_ = true;

        ROS_INFO("Stand up sequence completed");
        ros::Duration(1.0).sleep();
    }

    bool GaitController::computeLegIK(int leg_id, double x, double y, double z, double &q1, double &q2, double &q3)
    {
        // Pobierz konfigurację dla danej nogi
        const auto &leg = leg_origins.at(leg_id);

        // 1) Przesuń współrzędne względem biodra danej nogi
        double local_x = x - leg.x;
        double local_y = y - leg.y;

        // 2) Obrót biodra wokół Z
        q1 = std::atan2(local_y, local_x);

        if (leg.invert_hip)
        {
            // Dla prawej strony (2,4,6) - obrót o 180 stopni
            if (q1 > 0)
                q1 = q1 - M_PI;
            else
                q1 = q1 + M_PI;
        }

        // 3) Dystans promieniowy od osi biodra minus przesunięcie L1
        const double L1 = 0.065; // długość hip → knee
        const double L2 = 0.105; // długość knee → ankle
        const double L3 = 0.205; // długość ankle → stopa

        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z; // Oś Z w dół dodatnia

        // 4) Sprawdź zasięg
        double D2 = r * r + h * h;
        double D = std::sqrt(D2);
        if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        {
            ROS_WARN("Position out of reach: D=%f, L2+L3=%f, |L2-L3|=%f",
                     D, L2 + L3, std::fabs(L2 - L3));
            return false;
        }

        // 5) Kąt gamma między L2 i L3
        double cos_gamma = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
        double gamma = std::acos(cos_gamma);

        // 6) Kolano (joint 2)
        double alpha = std::atan2(h, r);
        double beta = std::acos((D2 + L2 * L2 - L3 * L3) / (2 * L2 * D));

        q2 = -(alpha - beta);

        // 7) Kostka (joint 3)
        if (leg.invert_knee)
        {
            // Dla prawej strony (2,4,6)
            q3 = gamma - M_PI;
        }
        else
        {
            // Dla lewej strony (1,3,5)
            q3 = -(M_PI - gamma);
        }

        ROS_DEBUG("Leg %d IK: x=%.3f y=%.3f z=%.3f -> q1=%.3f q2=%.3f q3=%.3f",
                  leg_id, x, y, z, q1, q2, q3);
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

    void GaitController::step(const geometry_msgs::Twist &cmd_vel)
    {
        if (!is_standing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        // switch (current_mode_)
        // {
        // case GaitMode::SINGLE:
        //     stepSingleLeg(cmd_vel);
        //     break;
        // case GaitMode::TWO_LEG:
        //     stepTwoLegs(cmd_vel);
        //     break;
        // case GaitMode::THREE_LEG:
        //     stepThreeLegs(cmd_vel);
        //     break;
        // }
    }
    void GaitController::standUpCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (!is_standing_)
        {
            ROS_INFO("Received stand up command");
            standUp();
            is_standing_ = true;
        }
        else
        {
            ROS_INFO("Robot is already standing");
        }
    }

} // namespace hex_controller