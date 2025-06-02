#include "hex_final_urdf_description/two_leg_gait.h"

namespace hexapod
{
    TwoLegGait::TwoLegGait(ros::NodeHandle &nh)
        : BaseGait(nh), is_initialized_(false), is_executing_(false)
    {
        // Parametry domyślne dla stabilnego chodu dwunożnego
        params_.step_length = 5.0;      // Średnie kroki dla balansu stabilność/prędkość
        params_.step_height = 5.0;      // Średnie podnoszenie
        params_.cycle_time = 2.5;       // Umiarkowana prędkość
        params_.standing_height = 24.0; // Wysokość stania (używana jako -24.0)
    }

    void TwoLegGait::initialize()
    {
        initializePublishers();
        updateParameters(nh_);
        is_initialized_ = true;
        ROS_INFO("TwoLegGait initialized - step_length=%.1f, step_height=%.1f, cycle_time=%.1f",
                 params_.step_length, params_.step_height, params_.cycle_time);
    }

    void TwoLegGait::updateParameters(const ros::NodeHandle &nh)
    {
        nh.param("step_length", params_.step_length, 5.0);
        nh.param("step_height", params_.step_height, 5.0);
        nh.param("cycle_time", params_.cycle_time, 2.5);
        nh.param("standing_height", params_.standing_height, 24.0);
    }

    bool TwoLegGait::execute()
    {
        if (!is_initialized_)
        {
            ROS_ERROR("TwoLegGait not initialized!");
            return false;
        }

        // Najpierw wstań używając metody z BaseGait
        if (!standUp())
        {
            ROS_ERROR("Failed to stand up");
            return false;
        }

        // Wykonaj kilka kroków do przodu
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.0; // Kierunek do przodu
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        walkForward(cmd_vel, 4); // 4 pełne cykle chodu dwunożnego
        return true;
    }

    void TwoLegGait::stop()
    {
        is_executing_ = false;
        ROS_INFO("TwoLegGait stopped - robot remains standing");
    }

    void TwoLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_cycles)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        is_executing_ = true;

        // Pozycje bazowe nóg - identyczne jak w standUp() dla spójności
        const std::map<int, LegPosition> base_positions = {
            {1, {18.0, -15.0, -params_.standing_height}},  // lewa przednia
            {2, {-18.0, -15.0, -params_.standing_height}}, // prawa przednia
            {3, {22.0, 0.0, -params_.standing_height}},    // lewa środkowa
            {4, {-22.0, 0.0, -params_.standing_height}},   // prawa środkowa
            {5, {18.0, 15.0, -params_.standing_height}},   // lewa tylna
            {6, {-18.0, 15.0, -params_.standing_height}}   // prawa tylna
        };

        // Pary nóg dla chodu dwunożnego (przeciwległe nogi dla stabilności)
        const std::vector<std::pair<int, int>> leg_pairs = {
            {1, 4}, // lewa przednia + prawa środkowa
            {2, 5}, // prawa przednia + lewa tylna
            {3, 6}  // lewa środkowa + prawa tylna
        };

        for (int cycle = 0; cycle < num_cycles && is_executing_ && ros::ok(); ++cycle)
        {
            ROS_INFO("=== Wykonuję cykl chodu dwunożnego %d/%d ===", cycle + 1, num_cycles);

            for (const auto &pair : leg_pairs)
            {
                int leg1 = pair.first;
                int leg2 = pair.second;

                ROS_INFO("Poruszam parę nóg: %d i %d", leg1, leg2);

                moveLegPair(leg1, leg2,
                            base_positions.at(leg1),
                            base_positions.at(leg2),
                            cmd_vel);

                // Pauza między parami dla stabilizacji
                ros::Duration(0.3).sleep();
            }

            // Dłuższa pauza między cyklami
            ros::Duration(0.5).sleep();
        }

        ROS_INFO("Zakończono chód dwunożny - robot pozostaje w pozycji stojącej");
    }

    void TwoLegGait::moveLegPair(int leg1, int leg2,
                                 const LegPosition &base_pos1,
                                 const LegPosition &base_pos2,
                                 const geometry_msgs::Twist &cmd_vel)
    {
        const int STEPS = 100;  // Płynność ruchu
        const double dt = 0.02; // Czas między krokami

        // Oblicz przesunięcie na podstawie prędkości
        double forward_step = params_.step_length * cmd_vel.linear.x;
        double side_step = params_.step_length * cmd_vel.linear.y;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            // === PARA PORUSZAJĄCYCH SIĘ NÓG ===
            for (auto leg_info : {std::make_pair(leg1, base_pos1), std::make_pair(leg2, base_pos2)})
            {
                int leg = leg_info.first;
                const LegPosition &base_pos = leg_info.second;

                double x = base_pos.x;
                double y, z;

                if (phase <= 0.5)
                {
                    // Pierwsza połowa: ruch do przodu + w górę
                    double swing_phase = phase * 2.0;

                    // Płynny ruch do przodu (używamy funkcji smoothstep)
                    double smooth_progress = smoothStep(swing_phase);
                    y = base_pos.y + forward_step * (smooth_progress - 0.5);

                    // Płynne podnoszenie (sinusoidalne)
                    z = base_pos.z - params_.step_height * std::sin(M_PI * swing_phase);
                }
                else
                {
                    // Druga połowa: kontynuacja ruchu + opuszczanie
                    double swing_phase = (phase - 0.5) * 2.0;

                    double smooth_progress = 0.5 + 0.5 * smoothStep(swing_phase);
                    y = base_pos.y + forward_step * (smooth_progress - 0.5);

                    // Płynne opuszczanie
                    z = base_pos.z - params_.step_height * std::sin(M_PI * (1.0 - swing_phase));
                }

                // Oblicz IK i ustaw stawy
                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
                else
                {
                    ROS_WARN("IK failed for leg %d at step %d", leg, step);
                }
            }

            // === POZOSTAŁE NOGI (PODPIERAJĄCE) ===
            for (int leg = 1; leg <= 6; ++leg)
            {
                if (leg == leg1 || leg == leg2)
                    continue; // Pomiń poruszające się nogi

                // Znajdź pozycję bazową dla tej nogi
                LegPosition support_pos;
                auto it = std::find_if(base_positions.begin(), base_positions.end(),
                                       [leg](const auto &pair)
                                       { return pair.first == leg; });

                if (it != base_positions.end())
                {
                    support_pos = it->second;

                    // Nogi podpierające wykonują subtelny ruch w przeciwnym kierunku
                    double x = support_pos.x;
                    double y = support_pos.y - forward_step * 0.3 * (phase - 0.5); // Mniejsze przesunięcie
                    double z = support_pos.z;

                    double q1, q2, q3;
                    if (computeLegIK(leg, x, y, z, q1, q2, q3))
                    {
                        setLegJoints(leg, q1, q2, q3);
                    }
                }
            }

            ros::Duration(dt).sleep();

            // Debug info co 25 kroków
            if (step % 25 == 0)
            {
                ROS_DEBUG("Pair (%d,%d), step %d/%d (%.1f%%)",
                          leg1, leg2, step, STEPS, phase * 100.0);
            }
        }
    }

    void TwoLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        // Pojedynczy krok = jeden cykl chodu dwunożnego
        walkForward(cmd_vel, 1);
    }

    double TwoLegGait::smoothStep(double x)
    {
        // Funkcja wygładzająca: 3x² - 2x³ (Hermite interpolation)
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

} // namespace hexapod