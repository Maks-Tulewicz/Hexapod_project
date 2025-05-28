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
        if (!is_standing_)
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

        // Parametry kroku
        const double step_length = params_.leg_x_offset * std::abs(cmd_vel.linear.x);
        const double step_height = params_.step_height;
        const int STEPS = 30;
        const double dt = 0.01;

        for (int leg : leg_sequence)
        {
            const auto &base = base_positions.at(leg);

            // Faza unoszenia nogi
            for (int step = 0; step <= STEPS / 2; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                // Interpolacja pozycji
                double x = base[0];
                double y = base[1];
                double z = base[2] - step_height * std::sin(M_PI * phase);

                // Dodaj ruch do przodu/tyłu
                if (cmd_vel.linear.x > 0)
                {
                    x += step_length * phase;
                }
                else if (cmd_vel.linear.x < 0)
                {
                    x -= step_length * phase;
                }

                // Dodaj ruch w bok
                if (cmd_vel.linear.y != 0)
                {
                    y += cmd_vel.linear.y * params_.leg_y_offset * phase;
                }

                // Dodaj obrót
                if (cmd_vel.angular.z != 0)
                {
                    double angle = cmd_vel.angular.z * params_.turning_radius;
                    if (leg <= 2)
                    { // przednie nogi
                        y += angle * phase;
                    }
                    else if (leg >= 5)
                    { // tylne nogi
                        y -= angle * phase;
                    }
                }

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }

                ros::Duration(dt).sleep();
            }

            // Faza opuszczania nogi
            for (int step = 0; step <= STEPS / 2; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                double x = base[0];
                double y = base[1];
                double z = base[2] - step_height * std::sin(M_PI * (1 - phase));

                if (cmd_vel.linear.x > 0)
                {
                    x += step_length * (1 - phase);
                }
                else if (cmd_vel.linear.x < 0)
                {
                    x -= step_length * (1 - phase);
                }

                if (cmd_vel.linear.y != 0)
                {
                    y += cmd_vel.linear.y * params_.leg_y_offset * (1 - phase);
                }

                if (cmd_vel.angular.z != 0)
                {
                    double angle = cmd_vel.angular.z * params_.turning_radius;
                    if (leg <= 2)
                    {
                        y += angle * (1 - phase);
                    }
                    else if (leg >= 5)
                    {
                        y -= angle * (1 - phase);
                    }
                }

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }

                ros::Duration(dt).sleep();
            }

            // Mała pauza między nogami
            ros::Duration(0.1).sleep();
        }
    }

} // namespace hex_controller