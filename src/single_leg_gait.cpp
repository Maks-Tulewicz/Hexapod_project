#include "hex_controller/single_leg_gait.hpp"
#include <cmath>

namespace hex_controller
{
    SingleLegGait::SingleLegGait(ros::NodeHandle &nh)
        : GaitController(nh)
    {
        current_mode_ = GaitMode::SINGLE;
        ROS_INFO("Single leg gait controller initialized");
    }

    void SingleLegGait::step(const geometry_msgs::Twist &cmd_vel)
    {
        if (!isStanding())
        {
            ROS_WARN_ONCE("Robot must be standing before walking");
            return;
        }

        // Ignoruj małe wartości
        if (std::abs(cmd_vel.linear.x) < 0.01 &&
            std::abs(cmd_vel.linear.y) < 0.01 &&
            std::abs(cmd_vel.angular.z) < 0.01)
        {
            return;
        }

        // Parametry ruchu (w metrach!)
        const double step_length = 0.04; // 4cm = 0.04m
        const double step_height = 0.03; // 3cm = 0.03m
        const int STEPS = 60;
        const double dt = 0.005;

        std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};

        for (int leg : leg_sequence)
        {
            const auto &base = base_positions.at(leg);
            double start_x = base[0]; // Już w metrach
            double start_y = base[1];
            double start_z = base[2];

            // Faza unoszenia nogi
            for (int step = 0; step <= STEPS / 2; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                // Pozycja X - ruch do przodu/tyłu
                double x = start_x;
                if (cmd_vel.linear.x != 0)
                {
                    x += step_length * std::cos(M_PI * phase) * (cmd_vel.linear.x > 0 ? 1 : -1);
                }

                // Pozycja Y - ruch na boki
                double y = start_y;
                if (cmd_vel.linear.y != 0)
                {
                    y += step_length * std::cos(M_PI * phase) * cmd_vel.linear.y;
                }

                // Pozycja Z - ruch góra/dół
                double z = start_z - step_height * std::sin(M_PI * phase);

                // Używamy wspólnej implementacji kinematyki odwrotnej
                double hip, knee, ankle;
                if (computeLegIK(leg, x, y, z, hip, knee, ankle))
                {
                    setLegJoints(leg, hip, knee, ankle);
                }

                ros::Duration(dt).sleep();
            }

            // Faza opuszczania nogi
            for (int step = 0; step <= STEPS / 2; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                double x = start_x;
                if (cmd_vel.linear.x != 0)
                {
                    x += step_length * std::cos(M_PI * (1 - phase)) * (cmd_vel.linear.x > 0 ? 1 : -1);
                }

                double y = start_y;
                if (cmd_vel.linear.y != 0)
                {
                    y += step_length * std::cos(M_PI * (1 - phase)) * cmd_vel.linear.y;
                }

                double z = start_z - step_height * std::sin(M_PI * (1 - phase));

                double hip, knee, ankle;
                if (computeLegIK(leg, x, y, z, hip, knee, ankle))
                {
                    setLegJoints(leg, hip, knee, ankle);
                }

                ros::Duration(dt).sleep();
            }

            ros::Duration(0.1).sleep();
        }
    }
} // namespace hex_controller