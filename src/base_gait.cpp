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

                // Wait for subscriber
                ros::Time start = ros::Time::now();
                while (joint_publishers_[joint_name].getNumSubscribers() == 0 &&
                       ros::ok() && (ros::Time::now() - start).toSec() < 3.0)
                {
                    ros::Duration(0.05).sleep();
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

        // Dodajemy debugowanie
        ROS_DEBUG("Leg %d IK input - x: %.2f, y: %.2f, z: %.2f", leg_number, x, y, z);

        // 1) Przesuń współrzędne względem biodra danej nogi
        double local_x = x - leg.x;
        double local_y = y - leg.y;

        // 2) Obrót biodra wokół Z
        q1 = std::atan2(local_y, local_x);

        if (leg.invert_hip)
        {
            if (q1 > 0)
                q1 = q1 - M_PI;
            else
                q1 = q1 + M_PI;
        }

        // 3) Dystans promieniowy od osi biodra minus przesunięcie L1
        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z;

        // Dodajemy debugowanie zasięgu
        double D2 = r * r + h * h;
        double D = std::sqrt(D2);

        if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        {
            ROS_DEBUG("Leg %d IK failed - Distance %.2f out of range [%.2f, %.2f]",
                      leg_number, D, std::fabs(L2 - L3), L2 + L3);
            return false;
        }

        // Reszta kodu bez zmian...
        return true;
    }

} // namespace hexapod