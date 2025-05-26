#include "hex_controller/bipod_gait.hpp"

namespace hex_controller
{

    BipodGait::BipodGait(ros::NodeHandle &nh) : GaitController(nh)
    {
        // Inicjalizacja par nóg dla chodu dwunożnego
        leg_pairs_ = {
            {0, 3}, // Przednia lewa i tylna prawa
            {1, 4}, // Środkowa lewa i środkowa prawa
            {2, 5}  // Tylna lewa i przednia prawa
        };
    }

    void BipodGait::step(const geometry_msgs::Twist &cmd_vel)
    {
        for (const auto &pair : leg_pairs_)
        {
            moveLegPair(pair, cmd_vel);
            ros::Duration(params_.cycle_time / 6.0).sleep();
        }
    }

    void BipodGait::moveLegPair(const std::pair<int, int> &pair,
                                const geometry_msgs::Twist &cmd_vel)
    {
        double x = 0, y = 0;
        adjustLegPosition(x, y, cmd_vel);

        // Podniesienie nóg
        double q1, q2, q3;
        if (computeLegIK(pair.first, x, y, params_.standing_height + params_.step_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.first, q1, q2, q3);
        }
        if (computeLegIK(pair.second, -x, -y, params_.standing_height + params_.step_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.second, q1, q2, q3);
        }

        ros::Duration(params_.cycle_time / 4.0).sleep();

        // Opuszczenie nóg
        if (computeLegIK(pair.first, x, y, params_.standing_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.first, q1, q2, q3);
        }
        if (computeLegIK(pair.second, -x, -y, params_.standing_height,
                         q1, q2, q3))
        {
            setLegJoints(pair.second, q1, q2, q3);
        }

        ros::Duration(params_.cycle_time / 4.0).sleep();
    }

} // namespace hex_controller