#include "hex_final_urdf_description/base_gait.h"
#include <cmath>

namespace hexapod
{

    // Initialize static member
    const std::map<int, LegOrigin> BaseGait::leg_origins = {
        {1, {0.068956, -0.077136, false, false}}, // lewa przednia
        {2, {-0.086608, -0.077136, true, true}},  // prawa przednia
        {3, {0.101174, 0.000645, false, false}},  // lewa środkowa
        {4, {-0.118826, -0.000645, true, true}},  // prawa środkowa
        {5, {0.068956, 0.078427, false, false}},  // lewa tylna
        {6, {-0.086608, 0.078427, true, true}}    // prawa tylna
    };

    const std::map<int, std::vector<double>> BaseGait::base_positions = {
        {1, {18.0, -15.0, -24.0}},  // lewa przednia
        {2, {-18.0, -15.0, -24.0}}, // prawa przednia
        {3, {22.0, 0.0, -24.0}},    // lewa środkowa
        {4, {-22.0, 0.0, -24.0}},   // prawa środkowa
        {5, {18.0, 15.0, -24.0}},   // lewa tylna
        {6, {-18.0, 15.0, -24.0}}   // prawa tylna
    };

    BaseGait::BaseGait(ros::NodeHandle &nh) : nh_(nh) {}

    void BaseGait::initializePublishers()
    {
        const std::vector<std::string> joint_types = {"hip", "knee", "ankle"};

        for (int leg = 1; leg <= 6; ++leg)
        {
            for (const auto &type : joint_types)
            {
                std::string joint_name = type + "_joint_" + std::to_string(leg);
                std::string topic = "/" + joint_name + "_position_controller/command";
                joint_publishers_[joint_name] = nh_.advertise<std_msgs::Float64>(topic, 1);

                // Dodajmy debug
                ROS_INFO("Initializing publisher for joint: %s on topic: %s",
                         joint_name.c_str(), topic.c_str());

                // Czekamy na subskrybentów
                ros::Time start = ros::Time::now();
                while (joint_publishers_[joint_name].getNumSubscribers() == 0 &&
                       ros::ok() && (ros::Time::now() - start).toSec() < 3.0)
                {
                    ros::Duration(0.1).sleep();
                    ROS_INFO_THROTTLE(1.0, "Waiting for subscribers on %s", topic.c_str());
                }

                if (joint_publishers_[joint_name].getNumSubscribers() == 0)
                {
                    ROS_WARN("No subscribers for %s after timeout", topic.c_str());
                }
                else
                {
                    ROS_INFO("Got subscribers for %s", topic.c_str());
                }
            }
        }
    }

    bool BaseGait::setJointPosition(const std::string &joint_name, double position)
    {
        auto it = joint_publishers_.find(joint_name);
        if (it == joint_publishers_.end())
        {
            ROS_ERROR("Joint publisher not found: %s", joint_name.c_str());
            return false;
        }

        std_msgs::Float64 msg;
        msg.data = position;
        it->second.publish(msg);
        return true;
    }

    bool BaseGait::computeLegIK(int leg_number, double x, double y, double z,
                                double &q1, double &q2, double &q3)
    {
        // Pobierz konfigurację dla danej nogi
        const auto &leg = leg_origins.at(leg_number);

        ROS_DEBUG("Leg %d IK input - x: %.2f, y: %.2f, z: %.2f", leg_number, x, y, z);

        // 1. Przekształcenie do lokalnego układu współrzędnych nogi
        double local_x = x - leg.x;
        double local_y = y - leg.y;

        // 2. Obliczenie kąta biodra (obrót wokół osi Z)
        q1 = std::atan2(local_y, local_x);

        // Inwersja kąta biodra dla prawych nóg
        if (leg.invert_hip)
        {
            if (q1 > 0)
                q1 = q1 - M_PI;
            else
                q1 = q1 + M_PI;
        }

        // 3. Obliczenie odległości radialnej od osi biodra
        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z; // Zmiana znaku, bo oś Z jest skierowana w dół

        // 4. Sprawdzenie czy punkt jest w zasięgu nogi
        double D2 = r * r + h * h;
        double D = std::sqrt(D2);

        if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        {
            ROS_DEBUG("Leg %d IK failed - Distance %.2f out of range [%.2f, %.2f]",
                      leg_number, D, std::fabs(L2 - L3), L2 + L3);
            return false;
        }

        // 5. Obliczenie kąta kolana (q2)
        double cos_q2 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
        cos_q2 = std::max(-1.0, std::min(1.0, cos_q2));

        q2 = -std::acos(cos_q2); // Podstawowe odwrócenie dla wszystkich nóg

        // 6. Obliczenie kąta kostki (q3)
        double beta = std::atan2(h, r);
        double alpha = std::atan2(L3 * std::sin(q2), L2 + L3 * std::cos(q2));

        q3 = -(beta - alpha);

        ROS_DEBUG("Leg %d (type: %s) IK result:", leg_number,
                  leg.invert_knee ? "right" : "left");
        ROS_DEBUG("  q1 (hip): %.2f°", q1 * 180.0 / M_PI);
        ROS_DEBUG("  q2 (knee): %.2f°", q2 * 180.0 / M_PI);
        ROS_DEBUG("  q3 (ankle): %.2f°", q3 * 180.0 / M_PI);

        return true;
    }
} // namespace hexapod