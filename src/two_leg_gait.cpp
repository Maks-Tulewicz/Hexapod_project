#include "hex_final_urdf_description/two_leg_gait.h"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace hexapod
{
    TwoLegGait::TwoLegGait(ros::NodeHandle &nh)
        : BaseGait(nh), is_initialized_(false), is_executing_(false)
    {
        updateParameters(nh);
    }

    void TwoLegGait::initialize()
    {
        initializePublishers();
        ROS_INFO("Two Leg Gait initialized");
        is_initialized_ = true;
    }

    bool TwoLegGait::execute()
    {
        standUp(); // Ustaw robota w pozycji stojącej

        if (!is_standing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return false;
        }

        is_executing_ = true;
        // Wykonaj ruch do przodu z domyślną prędkością
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.5;  // Prędkość do przodu
        cmd_vel.angular.z = 0.0; // Brak obrotu

        walkForward(cmd_vel, 10); // Wykonaj 10 kroków
        return true;
    }

    void TwoLegGait::stop()
    {
        is_executing_ = false;
        ROS_INFO("Stopping Two Leg Gait");
    }

    void TwoLegGait::updateParameters(const ros::NodeHandle &nh)
    {
        params_.step_length = 4.5;       // Mniejsza długość kroku dla stabilności
        params_.step_height = 2.5;       // Mniejsza wysokość podnoszenia
        params_.cycle_time = 2.0;        // Wolniejszy cykl
        params_.standing_height = -24.0; // Standardowa wysokość
    }

    void TwoLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!is_initialized_ || !is_executing_)
            return;

        for (int i = 0; i < num_steps && is_executing_; ++i)
        {
            makeStep(cmd_vel);
            ros::Duration(params_.cycle_time / 6.0).sleep();
        }
    }

    void TwoLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        if (!isStanding() || !is_initialized_ || !is_executing_)
            return;

        const int STEPS = 210;
        const double dt = 0.008;

        // Pozycje bazowe dla nóg
        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}},  // lewa przednia
            {2, {-18.0, -15.0, -24.0}}, // prawa przednia
            {3, {22.0, 0.0, -24.0}},    // lewa środkowa
            {4, {-22.0, 0.0, -24.0}},   // prawa środkowa
            {5, {18.0, 15.0, -24.0}},   // lewa tylna
            {6, {-18.0, 15.0, -24.0}}   // prawa tylna
        };

        // Pary nóg do równoczesnego ruchu
        const std::vector<std::pair<int, int>> leg_pairs = {
            {1, 4}, // Lewa przednia i prawa środkowa
            {2, 5}, // Prawa przednia i lewa tylna
            {3, 6}  // Lewa środkowa i prawa tylna
        };

        double movement_direction = cmd_vel.linear.x > 0 ? 1.0 : -1.0;

        for (const auto &pair : leg_pairs)
        {
            ROS_INFO("Moving legs %d and %d", pair.first, pair.second);
            const auto &pos1 = base_positions.at(pair.first);
            const auto &pos2 = base_positions.at(pair.second);

            // Faza uniesienia i przeniesienia nóg
            for (int step = 0; step < STEPS / 2 && is_executing_; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                // Ruch do przodu i w górę
                double x_offset = params_.step_length * phase * movement_direction;
                double z_offset = params_.step_height * std::sin(M_PI * phase);

                // Dla pierwszej nogi w parze
                double q1_1, q2_1, q3_1;
                if (computeLegIK(pair.first,
                                 pos1[0] + x_offset,
                                 pos1[1],
                                 pos1[2] + z_offset,
                                 q1_1, q2_1, q3_1))
                {
                    setLegJoints(pair.first, q1_1, q2_1, q3_1);
                }

                // Dla drugiej nogi w parze
                double q1_2, q2_2, q3_2;
                if (computeLegIK(pair.second,
                                 pos2[0] + x_offset,
                                 pos2[1],
                                 pos2[2] + z_offset,
                                 q1_2, q2_2, q3_2))
                {
                    setLegJoints(pair.second, q1_2, q2_2, q3_2);
                }

                ros::Duration(dt).sleep();
            }

            // Faza opuszczania nóg
            for (int step = 0; step < STEPS / 2 && is_executing_; ++step)
            {
                double phase = static_cast<double>(step) / (STEPS / 2);

                // Tylko ruch w dół
                double z_offset = params_.step_height * (1.0 - std::sin(M_PI * phase));

                // Dla pierwszej nogi w parze
                double q1_1, q2_1, q3_1;
                if (computeLegIK(pair.first,
                                 pos1[0] + params_.step_length * movement_direction,
                                 pos1[1],
                                 pos1[2] + z_offset,
                                 q1_1, q2_1, q3_1))
                {
                    setLegJoints(pair.first, q1_1, q2_1, q3_1);
                }

                // Dla drugiej nogi w parze
                double q1_2, q2_2, q3_2;
                if (computeLegIK(pair.second,
                                 pos2[0] + params_.step_length * movement_direction,
                                 pos2[1],
                                 pos2[2] + z_offset,
                                 q1_2, q2_2, q3_2))
                {
                    setLegJoints(pair.second, q1_2, q2_2, q3_2);
                }

                ros::Duration(dt).sleep();
            }
        }
    }

    void TwoLegGait::moveLegPair(int leg1, int leg2,
                                 const std::vector<double> &start_pos1,
                                 const std::vector<double> &start_pos2,
                                 double movement_direction)
    {
        const int STEPS = 210;
        const double dt = 0.008;

        for (int step = 0; step < STEPS && is_executing_; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;

                // Pierwsza noga
                double y1 = start_pos1[1] - params_.step_length * (1.0 - std::cos(M_PI * swing_phase)) * movement_direction;
                double z1 = start_pos1[2] - params_.step_height * std::sin(M_PI * swing_phase);

                // Druga noga - ten sam ruch
                double y2 = start_pos2[1] - params_.step_length * (1.0 - std::cos(M_PI * swing_phase)) * movement_direction;
                double z2 = start_pos2[2] - params_.step_height * std::sin(M_PI * swing_phase);

                // Obliczanie kątów dla nóg przy użyciu kinematyki odwrotnej
                double q1_1, q2_1, q3_1, q1_2, q2_2, q3_2;

                if (computeLegIK(leg1, start_pos1[0], y1, z1, q1_1, q2_1, q3_1))
                {
                    setLegJoints(leg1, q1_1, q2_1, q3_1);
                }

                if (computeLegIK(leg2, start_pos2[0], y2, z2, q1_2, q2_2, q3_2))
                {
                    setLegJoints(leg2, q1_2, q2_2, q3_2);
                }
            }
            else
            {
                // Faza podporowa - powrót do pozycji początkowej
                double support_phase = (phase - 0.5) * 2.0;

                // Pierwsza noga
                double y1 = (start_pos1[1] - params_.step_length * movement_direction) +
                            params_.step_length * support_phase * movement_direction;

                // Druga noga
                double y2 = (start_pos2[1] - params_.step_length * movement_direction) +
                            params_.step_length * support_phase * movement_direction;

                double q1_1, q2_1, q3_1, q1_2, q2_2, q3_2;

                if (computeLegIK(leg1, start_pos1[0], y1, start_pos1[2], q1_1, q2_1, q3_1))
                {
                    setLegJoints(leg1, q1_1, q2_1, q3_1);
                }

                if (computeLegIK(leg2, start_pos2[0], y2, start_pos2[2], q1_2, q2_2, q3_2))
                {
                    setLegJoints(leg2, q1_2, q2_2, q3_2);
                }
            }

            ros::Duration(dt).sleep();
        }
    }
} // namespace hexapod