#include "hex_controller/single_leg_gait.hpp"
#include <cmath>

namespace hex_controller
{
    SingleLegGait::SingleLegGait(ros::NodeHandle &nh) : GaitController(nh)
    {
        current_mode_ = GaitMode::SINGLE;
        initializeParameters();
        ROS_INFO("Single leg gait controller initialized");
    }

    void SingleLegGait::initializeParameters()
    {
        // Parametry chodu
        params_.step_length = 4.0;       // Długość kroku
        params_.step_height = 3.0;       // Wysokość podnoszenia
        params_.cycle_time = 0.6;        // Szybki cykl
        params_.body_shift = 1.0;        // Przesunięcie przy obrocie
        params_.standing_height = -24.0; // Wysokość stania
        params_.leg_x_offset = 0.1;      // Przesunięcie nogi w X
        params_.leg_y_offset = 0.1;      // Przesunięcie nogi w Y
        params_.turning_radius = 0.3;    // Promień skrętu

        // Parametry trajektorii
        traj_params_.lift_phase = 0.3;     // 30% cyklu na podnoszenie
        traj_params_.transfer_phase = 0.4; // 40% cyklu na przenoszenie
        traj_params_.lower_phase = 0.3;    // 30% cyklu na opuszczanie
        traj_params_.stance_height = params_.standing_height;
        traj_params_.clearance = params_.step_height;
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

        makeStep(cmd_vel);
    }

    void SingleLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        const std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};

        for (int leg : leg_sequence)
        {
            ROS_INFO("Ruch nogi %d", leg);

            // 1. Wykonaj ruch nogą
            const auto &base = base_positions.at(leg);
            moveSingleLeg(leg, base, cmd_vel);

            // 2. Krótka pauza na stabilizację
            ros::Duration(0.05).sleep();

            // 3. Przesunięcie tułowia
            if (cmd_vel.linear.x != 0)
            {
                geometry_msgs::Twist body_shift;
                body_shift.linear.x = -cmd_vel.linear.x * 0.5;

                for (int support_leg = 1; support_leg <= 6; ++support_leg)
                {
                    if (support_leg != leg)
                    {
                        const auto &support_base = base_positions.at(support_leg);
                        moveSingleLeg(support_leg, support_base, body_shift);
                    }
                }
            }
        }
    }

    void SingleLegGait::moveSingleLeg(int leg_number,
                                      const std::vector<double> &start_pos,
                                      const geometry_msgs::Twist &cmd_vel)
    {
        const int STEPS = 50;
        const double dt = params_.cycle_time / STEPS;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;
            double x = start_pos[0];
            double y = start_pos[1];
            double z = params_.standing_height;

            // Określenie fazy ruchu
            if (phase <= traj_params_.lift_phase)
            {
                // Faza 1: Podnoszenie nogi
                double lift_ratio = phase / traj_params_.lift_phase;
                z = params_.standing_height -
                    traj_params_.clearance * smoothStep(lift_ratio);

                if (std::abs(cmd_vel.linear.x) > 0.01)
                {
                    double forward_ratio = lift_ratio * 0.2;
                    y = start_pos[1] - params_.step_length *
                                           forward_ratio * (cmd_vel.linear.x > 0 ? 1 : -1);
                }
            }
            else if (phase <= traj_params_.lift_phase + traj_params_.transfer_phase)
            {
                // Faza 2: Przenoszenie nogi
                double transfer_phase = (phase - traj_params_.lift_phase) /
                                        traj_params_.transfer_phase;

                z = params_.standing_height - traj_params_.clearance;

                if (std::abs(cmd_vel.linear.x) > 0.01)
                {
                    double forward_ratio = 0.2 + 0.6 * smoothStep(transfer_phase);
                    y = start_pos[1] - params_.step_length *
                                           forward_ratio * (cmd_vel.linear.x > 0 ? 1 : -1);
                }

                if (std::abs(cmd_vel.linear.y) > 0.01)
                {
                    x = start_pos[0] + params_.step_length *
                                           smoothStep(transfer_phase) * cmd_vel.linear.y;
                }
            }
            else
            {
                // Faza 3: Opuszczanie nogi
                double lower_ratio = (phase - (traj_params_.lift_phase +
                                               traj_params_.transfer_phase)) /
                                     traj_params_.lower_phase;

                z = params_.standing_height -
                    traj_params_.clearance * (1.0 - smoothStep(lower_ratio));

                if (std::abs(cmd_vel.linear.x) > 0.01)
                {
                    double forward_ratio = 0.8 + 0.2 * smoothStep(lower_ratio);
                    y = start_pos[1] - params_.step_length *
                                           forward_ratio * (cmd_vel.linear.x > 0 ? 1 : -1);
                }

                if (std::abs(cmd_vel.linear.y) > 0.01)
                {
                    x = start_pos[0] + params_.step_length *
                                           cmd_vel.linear.y;
                }
            }

            // Obsługa obrotu
            if (std::abs(cmd_vel.angular.z) > 0.01)
            {
                const auto &leg = leg_origins.at(leg_number);
                double turn_angle = params_.body_shift * cmd_vel.angular.z;
                double turn_factor = smoothStep(phase);

                if (leg_number <= 2)
                    x += turn_angle * turn_factor;
                else if (leg_number >= 5)
                    x -= turn_angle * turn_factor;
            }

            // Kinematyka odwrotna
            double hip, knee, ankle;
            if (computeLegIK(leg_number, x, y, z, hip, knee, ankle))
            {
                setLegJoints(leg_number, hip, knee, ankle);
            }

            ros::Duration(dt).sleep();
        }
    }

    void SingleLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        for (int step = 0; step < num_steps && ros::ok(); ++step)
        {
            ROS_INFO("Wykonuję krok %d z %d", step + 1, num_steps);
            makeStep(cmd_vel);
            ros::Duration(0.2).sleep();
        }
    }

    double SingleLegGait::smoothStep(double x)
    {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3 - 2 * x);
    }

} // namespace hex_controller