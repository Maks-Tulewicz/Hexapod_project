#include "hex_final_urdf_description/two_leg_gait.h"
#include <cmath>
#include <algorithm>

namespace hexapod
{
    TwoLegGait::TwoLegGait(ros::NodeHandle &nh)
        : BaseGait(nh), is_initialized_(false), is_executing_(false)
    {
        updateParameters(nh);
    }

    void TwoLegGait::updateParameters(const ros::NodeHandle &nh)
    {
        params_.step_length = 6.0;       // Długość kroku
        params_.step_height = 3.0;       // Wysokość podnoszenia
        params_.cycle_time = 2.0;        // Czas cyklu
        params_.standing_height = -24.0; // Standardowa wysokość
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

    void TwoLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        if (!isStanding() || !is_executing_)
            return;

        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}},  // lewa przednia
            {2, {-18.0, -15.0, -24.0}}, // prawa przednia
            {3, {22.0, 0.0, -24.0}},    // lewa środkowa
            {4, {-22.0, 0.0, -24.0}},   // prawa środkowa
            {5, {18.0, 15.0, -24.0}},   // lewa tylna
            {6, {-18.0, 15.0, -24.0}}   // prawa tylna
        };

        const std::vector<std::pair<int, int>> leg_pairs = {
            {1, 4}, // Lewa przednia i prawa środkowa
            {2, 5}, // Prawa przednia i lewa tylna
            {3, 6}  // Lewa środkowa i prawa tylna
        };

        for (const auto &pair : leg_pairs)
        {
            ROS_INFO("Moving legs %d and %d", pair.first, pair.second);

            const auto &pos1 = base_positions.at(pair.first);
            const auto &pos2 = base_positions.at(pair.second);

            LegPosition start_pos1 = {pos1[0], pos1[1], pos1[2]};
            LegPosition start_pos2 = {pos2[0], pos2[1], pos2[2]};

            moveLegPair(pair.first, pair.second, start_pos1, start_pos2);
            ros::Duration(0.3).sleep();
        }
    }

    void TwoLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!isStanding() || !is_executing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        // Parametry stabilizacji
        const double step_pause = 0.2;  // Pauza między pojedynczymi krokami
        const double cycle_pause = 0.2; // Dłuższa pauza po pełnym cyklu

        for (int step = 0; step < num_steps && ros::ok() && is_executing_; ++step)
        {
            ROS_INFO("Executing step %d of %d", step + 1, num_steps);

            makeStep(cmd_vel);

            // Pauza na stabilizację po kroku
            ros::Duration(step_pause).sleep();

            // Co pełen cykl dajemy dłuższą pauzę na stabilizację
            if (step % 2 == 1)
            {
                ROS_INFO("Stabilizing after full step cycle");
                ros::Duration(cycle_pause).sleep();
            }
        }
    }

    void TwoLegGait::moveLegPair(int leg1, int leg2,
                                 const LegPosition &start_pos1,
                                 const LegPosition &start_pos2)
    {
        const int STEPS = 160;
        const double dt = 0.005;

        for (int step = 0; step <= STEPS && is_executing_; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;
            double x1 = start_pos1.x;
            double y1 = start_pos1.y;
            double z1 = start_pos1.z;
            double x2 = start_pos2.x;
            double y2 = start_pos2.y;
            double z2 = start_pos2.z;

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;

                // Ruch do przodu z funkcją cosinus dla płynności
                double forward_motion = (1.0 - std::cos(M_PI * swing_phase));
                y1 = start_pos1.y - params_.step_length * forward_motion;
                y2 = start_pos2.y - params_.step_length * forward_motion;

                // Sinusoidalny ruch w górę i w dół
                z1 = start_pos1.z - params_.step_height * std::sin(M_PI * swing_phase);
                z2 = start_pos2.z - params_.step_height * std::sin(M_PI * swing_phase);
            }
            else
            {
                // Faza podporowa - powrót do pozycji początkowej
                double support_phase = (phase - 0.5) * 2.0;

                y1 = (start_pos1.y - params_.step_length) + params_.step_length * support_phase;
                y2 = (start_pos2.y - params_.step_length) + params_.step_length * support_phase;

                z1 = start_pos1.z;
                z2 = start_pos2.z;
            }

            // Aplikuj kinematykę odwrotną i wysyłaj komendy dla obu nóg
            double q1_1, q2_1, q3_1, q1_2, q2_2, q3_2;

            if (computeLegIK(leg1, x1, y1, z1, q1_1, q2_1, q3_1))
            {
                setLegJoints(leg1, q1_1, q2_1, q3_1);
            }

            if (computeLegIK(leg2, x2, y2, z2, q1_2, q2_2, q3_2))
            {
                setLegJoints(leg2, q1_2, q2_2, q3_2);
            }

            ros::Duration(dt).sleep();
            ros::spinOnce();
        }
    }

} // namespace hexapod