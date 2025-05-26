#include "hex_controller/bipod_gait.hpp"

namespace hex_controller
{

    BipodGait::BipodGait(ros::NodeHandle &nh) : GaitController(nh)
    {
        // Inicjalizacja par nóg dla chodu dwunożnego
        leg_pairs_.push_back(std::make_pair(0, 3)); // Przednia lewa i środkowa prawa
        leg_pairs_.push_back(std::make_pair(2, 5)); // Środkowa lewa i tylna prawa
        leg_pairs_.push_back(std::make_pair(4, 1)); // Tylna lewa i przednia prawa
    }

    void BipodGait::step(const geometry_msgs::Twist &cmd_vel)
    {
        for (const auto &pair : leg_pairs_)
        {
            moveLegPair(pair, cmd_vel);
            ros::Duration(params_.cycle_time / 6.0).sleep();
        }
    }

    void BipodGait::moveLegPair(const std::pair<int, int> &pair, const geometry_msgs::Twist &cmd_vel)
    {
        double x = 0, y = 0;
        adjustLegPosition(x, y, cmd_vel);

        // Zmienne dla kątów stawów
        double q1, q2, q3;

        // Pierwsza noga w parze do góry i do przodu
        if (computeLegIK(pair.first, x, y, params_.standing_height + params_.step_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.first, q1, q2, q3);
        }
        else
        {
            ROS_WARN("IK failed for leg %d (up)", pair.first);
        }

        // Druga noga w parze do tyłu przy podłożu
        if (computeLegIK(pair.second, -x, -y, params_.standing_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.second, q1, q2, q3);
        }
        else
        {
            ROS_WARN("IK failed for leg %d (down)", pair.second);
        }

        ros::Duration(params_.cycle_time / 4.0).sleep();

        // Zamiana ról nóg w parze
        if (computeLegIK(pair.first, -x, -y, params_.standing_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.first, q1, q2, q3);
        }
        else
        {
            ROS_WARN("IK failed for leg %d (down)", pair.first);
        }

        if (computeLegIK(pair.second, x, y, params_.standing_height + params_.step_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.second, q1, q2, q3);
        }
        else
        {
            ROS_WARN("IK failed for leg %d (up)", pair.second);
        }
    }

} // namespace hex_controller