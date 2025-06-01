#include "hex_final_urdf_description/two_leg_gait.h"
#include <cmath>
#include <algorithm>

namespace hexapod
{
    TwoLegGait::TwoLegGait(ros::NodeHandle &nh)
        : BaseGait(nh)
    {
        updateParameters(nh);
    }

    void TwoLegGait::updateParameters(const ros::NodeHandle &nh)
    {
        params_.step_length = 4.5;       // Mniejsza długość kroku dla stabilności
        params_.step_height = 2.5;       // Mniejsza wysokość podnoszenia
        params_.cycle_time = 2.0;        // Wolniejszy cykl
        params_.standing_height = -24.0; // Standardowa wysokość
    }

    void TwoLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        if (!isStanding())
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
            {1, 6}, // Lewa przednia i prawa tylna
            {2, 5}, // Prawa przednia i lewa tylna
            {3, 4}  // Lewa środkowa i prawa środkowa
        };

        double movement_direction = cmd_vel.linear.x > 0 ? -1.0 : 1.0;

        for (const auto &pair : leg_pairs)
        {
            ROS_INFO("Przenoszenie pary nóg %d i %d", pair.first, pair.second);

            moveLegPair(pair.first, pair.second,
                        base_positions.at(pair.first),
                        base_positions.at(pair.second),
                        movement_direction);

            ros::Duration(0.3).sleep();
        }
    }

    void TwoLegGait::moveLegPair(int leg1, int leg2,
                                 const std::vector<double> &start_pos1,
                                 const std::vector<double> &start_pos2,
                                 double movement_direction)
    {
        const int STEPS = 210;
        const double dt = 0.008;

        // Bufory dla wygładzania ruchu
        std::deque<double> y_buffer1(5, start_pos1[1]);
        std::deque<double> y_buffer2(5, start_pos2[1]);

        double last_y1 = start_pos1[1];
        double last_y2 = start_pos2[1];

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            // Pozycje dla obu nóg
            std::vector<double> pos1 = start_pos1;
            std::vector<double> pos2 = start_pos2;

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;
                double motion = std::sin(M_PI * swing_phase / 2.0);
                motion = motion * motion; // Kwadrat sinusa dla płynniejszego ruchu

                // Obliczenie pozycji Y dla obu nóg
                double y_offset = params_.step_length * movement_direction * motion;
                pos1[1] = start_pos1[1] + y_offset;
                pos2[1] = start_pos2[1] + y_offset;

                // Ruch w górę i w dół
                double height_factor = 4.0 * swing_phase * (1.0 - swing_phase);
                pos1[2] = start_pos1[2] - params_.step_height * height_factor;
                pos2[2] = start_pos2[2] - params_.step_height * height_factor;
            }
            else
            {
                // Faza podporowa
                double support_phase = (phase - 0.5) * 2.0;
                double motion = std::cos(M_PI * support_phase / 2.0);
                motion = motion * motion;

                double y_offset = params_.step_length * movement_direction * (1.0 - motion);
                pos1[1] = start_pos1[1] + y_offset;
                pos2[1] = start_pos2[1] + y_offset;

                pos1[2] = start_pos1[2];
                pos2[2] = start_pos2[2];
            }

            // Wygładzanie ruchu
            y_buffer1.pop_front();
            y_buffer1.push_back(pos1[1]);
            y_buffer2.pop_front();
            y_buffer2.push_back(pos2[1]);

            double smooth_y1 = 0.0, smooth_y2 = 0.0;
            for (double y : y_buffer1)
                smooth_y1 += y;
            for (double y : y_buffer2)
                smooth_y2 += y;
            pos1[1] = smooth_y1 / y_buffer1.size();
            pos2[1] = smooth_y2 / y_buffer2.size();

            // Ograniczenie prędkości zmian
            double max_step_change = 0.3;
            pos1[1] = last_y1 + std::clamp(pos1[1] - last_y1, -max_step_change, max_step_change);
            pos2[1] = last_y2 + std::clamp(pos2[1] - last_y2, -max_step_change, max_step_change);
            last_y1 = pos1[1];
            last_y2 = pos2[1];

            // Zastosowanie kinematyki odwrotnej i ustawienie stawów
            double q1, q2, q3;
            if (computeLegIK(leg1, pos1[0], pos1[1], pos1[2], q1, q2, q3))
            {
                setLegJoints(leg1, q1, q2, q3);
            }
            if (computeLegIK(leg2, pos2[0], pos2[1], pos2[2], q1, q2, q3))
            {
                setLegJoints(leg2, q1, q2, q3);
            }

            ros::Duration(dt).sleep();
            ros::spinOnce();
        }
    }
    void TwoLegGait::initialize()
    {

        initializePublishers();
        ROS_INFO("One leg gait initialized");
    }

    bool TwoLegGait::execute()
    {

        standUp(); // Ustaw robota w pozycji stojącej

        if (!is_standing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return false;
        }
        else
        {
            // Wykonaj ruch do przodu z domyślną prędkością
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.5;  // Prędkość do przodu
            cmd_vel.angular.z = 0.0; // Brak obrotu

            moveLegPair(1, 6, {18.0, -15.0, -24.0}, {-18.0, 15.0, -24.0}, cmd_vel.linear.x > 0 ? -1.0 : 1.0);
            moveLegPair(2, 5, {-18.0, -15.0, -24.0}, {18.0, 15.0, -24.0}, cmd_vel.linear.x > 0 ? -1.0 : 1.0);
            moveLegPair(3, 4, {22.0, 0.0, -24.0}, {-22.0, 0.0, -24.0}, cmd_vel.linear.x > 0 ? -1.0 : 1.0);
        }
        return true;
    }
    void TwoLegGait::stop() { /* Możesz dodać własną logikę zatrzymania lub zostawić pustą */ }

} // namespace hexapod