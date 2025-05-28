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
        // Zmniejszamy parametry ruchu dla zachowania zasięgu
        params_.step_length = 4.0;       // Zmniejszamy długość kroku dla bezpieczeństwa
        params_.step_height = 5.0;       // Wysokość podnoszenia
        params_.cycle_time = 0.4;        // Znacznie szybszy cykl (było 1.8s)
        params_.body_shift = 1.0;        // Przesunięcie boczne
        params_.standing_height = -24.0; // Wysokość stania    }
    }

    void SingleLegGait::moveSingleLeg(int leg_number, const std::vector<double> &start_pos,
                                      const geometry_msgs::Twist &cmd_vel)
    {
        const int STEPS = 30; // Szybszy ruch
        const double dt = params_.cycle_time / STEPS;

        using namespace hex_controller::robot_geometry;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            // WAŻNE: Zachowujemy początkową pozycję X
            double x = start_pos[0]; // Pozycja X pozostaje stała przy ruchu do przodu
            double y = start_pos[1]; // Tu będzie ruch do przodu/tyłu
            double z = start_pos[2]; // Wysokość

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;

                // KLUCZOWA ZMIANA: Ruch do przodu w osi Y
                // Odwrócony znak dla właściwego kierunku (tak jak w walk_two_leg)
                y = start_pos[1] - params_.step_length *
                                       (1.0 - std::cos(M_PI * swing_phase)) *
                                       (cmd_vel.linear.x > 0 ? 1 : -1);

                // Ruch w górę i w dół
                z = start_pos[2] - params_.step_height * std::sin(M_PI * swing_phase);

                // Ruch boczny (jeśli jest) - w osi X
                if (std::abs(cmd_vel.linear.y) > 0.01)
                {
                    x = start_pos[0] + params_.step_length *
                                           (1.0 - std::cos(M_PI * swing_phase)) *
                                           cmd_vel.linear.y;
                }
            }
            else
            {
                // Faza podporowa
                double support_phase = (phase - 0.5) * 2.0;

                // WAŻNE: Powrót do pozycji początkowej w osi Y
                y = (start_pos[1] - params_.step_length *
                                        (cmd_vel.linear.x > 0 ? 1 : -1)) +
                    params_.step_length * support_phase *
                        (cmd_vel.linear.x > 0 ? 1 : -1);

                // Powrót z ruchu bocznego (jeśli był)
                if (std::abs(cmd_vel.linear.y) > 0.01)
                {
                    x = start_pos[0] + params_.step_length *
                                           cmd_vel.linear.y * (1.0 - support_phase);
                }
            }

            // Obsługa obrotu (jeśli jest)
            if (std::abs(cmd_vel.angular.z) > 0.01)
            {
                const auto &leg = leg_origins.at(leg_number);
                double turn_angle = params_.body_shift * cmd_vel.angular.z;

                // Przednie nogi (1,2)
                if (leg_number <= 2)
                {
                    x += turn_angle * std::sin(2 * M_PI * phase);
                }
                // Tylne nogi (5,6)
                else if (leg_number >= 5)
                {
                    x -= turn_angle * std::sin(2 * M_PI * phase);
                }
            }

            // Debug info
            ROS_DEBUG("Leg %d: phase=%.2f, x=%.2f, y=%.2f, z=%.2f",
                      leg_number, phase, x, y, z);

            // Kinematyka odwrotna
            double hip, knee, ankle;
            if (computeLegIK(leg_number, x, y, z, hip, knee, ankle))
            {
                setLegJoints(leg_number, hip, knee, ankle);
            }

            ros::Duration(dt).sleep();
        }
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
            ros::Duration(0.05).sleep(); // Zmniejszona z 0.1

            // 3. Przesunięcie tułowia (powrót do pozycji środkowej)
            if (cmd_vel.linear.x != 0)
            {
                // Przeciwny ruch tułowia - połowa długości kroku
                geometry_msgs::Twist body_shift;
                body_shift.linear.x = -cmd_vel.linear.x * 0.5;

                // Przesuń wszystkie nogi względem tułowia
                for (int support_leg = 1; support_leg <= 6; ++support_leg)
                {
                    if (support_leg != leg) // Nie ruszaj aktywnej nogi
                    {
                        const auto &support_base = base_positions.at(support_leg);
                        moveSingleLeg(support_leg, support_base, body_shift);
                    }
                }
            }
        }
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

        // Wykonaj pojedynczy krok
        makeStep(cmd_vel);
    }

    void SingleLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        // Ustaw parametry chodu
        params_.step_length = 6.0; // Zmniejszona długość kroku
        params_.step_height = 4.0; // Zmniejszona wysokość podnoszenia
        params_.cycle_time = 1.8;
        params_.body_shift = 2.0;
        params_.standing_height = -24.0;

        for (int step = 0; step < num_steps && ros::ok(); ++step)
        {
            ROS_INFO("Wykonuję krok %d z %d", step + 1, num_steps);
            makeStep(cmd_vel);
            ros::Duration(0.2).sleep(); // Pauza między krokami
        }
    }

} // namespace hex_controller