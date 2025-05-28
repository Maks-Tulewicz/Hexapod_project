#include "hex_controller/single_leg_gait.hpp"
#include <cmath>

namespace hex_controller
{

    SingleLegGait::SingleLegGait(ros::NodeHandle &nh) : GaitController(nh)
    {
        ROS_INFO("Single leg gait controller initialized");
    }

    void SingleLegGait::moveSingleLeg(int leg_number,
                                      const LegPosition &start_pos,
                                      const geometry_msgs::Twist &cmd_vel)
    {
        const int STEPS = 30;
        const double dt = LEG_CYCLE_TIME / STEPS;

        // Oblicz długość kroku na podstawie cmd_vel
        double step_length = DEFAULT_STEP_LENGTH * std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
        double step_angle = std::atan2(cmd_vel.linear.y, cmd_vel.linear.x);

        // Uwzględnij obrót
        double turning_factor = cmd_vel.angular.z * params_.turning_radius;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;
            double x = start_pos.x;
            double y = start_pos.y;
            double z = start_pos.z;

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;

                // Ruch do przodu/boku z uwzględnieniem kierunku
                double dx = step_length * std::cos(step_angle) * (1.0 - std::cos(M_PI * swing_phase));
                double dy = step_length * std::sin(step_angle) * (1.0 - std::cos(M_PI * swing_phase));

                // Dodaj komponent obrotu
                if (leg_number <= 2)
                { // przednie nogi
                    dy += turning_factor * swing_phase;
                }
                else if (leg_number >= 5)
                { // tylne nogi
                    dy -= turning_factor * swing_phase;
                }

                x += dx;
                y += dy;

                // Ruch w górę i w dół
                z = start_pos.z - params_.step_height * std::sin(M_PI * swing_phase);
            }
            else
            {
                // Faza podporowa - powrót do pozycji początkowej
                double support_phase = (phase - 0.5) * 2.0;

                double dx = step_length * std::cos(step_angle) * (1.0 - support_phase);
                double dy = step_length * std::sin(step_angle) * (1.0 - support_phase);

                if (leg_number <= 2)
                {
                    dy += turning_factor * (1.0 - support_phase);
                }
                else if (leg_number >= 5)
                {
                    dy -= turning_factor * (1.0 - support_phase);
                }

                x += dx;
                y += dy;
            }

            double q1, q2, q3;
            if (computeLegIK(leg_number, x, y, z, q1, q2, q3))
            {
                setLegJoints(leg_number, q1, q2, q3);
            }

            ros::Duration(dt).sleep();
        }
    }

    void SingleLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        for (int leg : leg_sequence)
        {
            ROS_DEBUG("Moving leg %d", leg);

            // Pozycja początkowa dla bieżącej nogi
            const auto &base = base_positions.at(leg);
            LegPosition start_pos = {base[0], base[1], base[2]};

            // Wykonaj ruch nogą
            moveSingleLeg(leg, start_pos, cmd_vel);

            // Krótka pauza między ruchami nóg
            ros::Duration(0.05).sleep();
        }
    }

    void SingleLegGait::step(const geometry_msgs::Twist &cmd_vel)
    {
        // Sprawdź czy jest jakiś ruch
        if (std::abs(cmd_vel.linear.x) < 0.01 &&
            std::abs(cmd_vel.linear.y) < 0.01 &&
            std::abs(cmd_vel.angular.z) < 0.01)
        {
            return;
        }

        makeStep(cmd_vel);
    }

} // namespace hex_controller