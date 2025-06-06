#include "hex_controller/tripod_gait.hpp"

namespace hex_controller
{

    TripodGait::TripodGait(ros::NodeHandle &nh) : GaitController(nh)
    {
        // Inicjalizacja grup nóg dla chodu trójnożnego
        std::vector<int> group1 = {0, 2, 4}; // Pierwsza grupa (lewa przednia, prawa środkowa, lewa tylna)
        std::vector<int> group2 = {1, 3, 5}; // Druga grupa (prawa przednia, lewa środkowa, prawa tylna)
        leg_groups_.push_back(group1);
        leg_groups_.push_back(group2);
    }

    void TripodGait::step(const geometry_msgs::Twist &cmd_vel)
    {
        // Pierwsza grupa do góry i do przodu
        moveLegGroup(0, cmd_vel, true);
        // Druga grupa do tyłu przy podłożu
        moveLegGroup(1, cmd_vel, false);

        ros::Duration(params_.cycle_time / 2.0).sleep();

        // Zamiana grup
        moveLegGroup(1, cmd_vel, true);
        moveLegGroup(0, cmd_vel, false);

        ros::Duration(params_.cycle_time / 2.0).sleep();
    }

    void TripodGait::moveLegGroup(int group_id, const geometry_msgs::Twist &cmd_vel, bool lift)
    {
        if (group_id >= leg_groups_.size())
        {
            ROS_ERROR("Invalid group_id: %d", group_id);
            return;
        }

        double x = 0, y = 0;
        adjustLegPosition(x, y, cmd_vel);

        double height = params_.standing_height;
        if (lift)
        {
            height += params_.step_height;
            x *= 1.0; // Ruch do przodu
        }
        else
        {
            x *= -1.0; // Ruch do tyłu
        }

        for (int leg_id : leg_groups_[group_id])
        {
            double q1, q2, q3;
            if (computeLegIK(leg_id, x, y, height, q1, q2, q3))
            {
                setLegJoints(leg_id, q1, q2, q3);
            }
            else
            {
                ROS_WARN("IK failed for leg %d", leg_id);
            }
        }
    }

} // namespace hex_controller