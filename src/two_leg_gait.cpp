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

    void TwoLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!isStanding() || !is_executing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        // Parametry stabilizacji
        const double step_pause = 0.2;  // Pauza między pojedynczymi krokami
        const double cycle_pause = 0.5; // Dłuższa pauza po pełnym cyklu

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

        // Modyfikujemy kolejność par nóg, żeby zacząć od przednich
        const std::vector<std::pair<int, int>> leg_pairs = {
            {2, 5}, // Prawa przednia i lewa tylna
            {1, 4}, // Lewa przednia i prawa środkowa
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

    void TwoLegGait::moveLegPair(int leg1, int leg2,
                                 const LegPosition &start_pos1,
                                 const LegPosition &start_pos2)
    {
        const int STEPS = 120; // Zwiększona ilość kroków dla płynności
        const double dt = 0.005;

        // Zwiększone bufory dla lepszego wygładzania
        const int BUFFER_SIZE = 7;
        std::deque<double> y_buffer1(BUFFER_SIZE, start_pos1.y);
        std::deque<double> y_buffer2(BUFFER_SIZE, start_pos2.y);
        std::deque<double> z_buffer1(BUFFER_SIZE, start_pos1.z);
        std::deque<double> z_buffer2(BUFFER_SIZE, start_pos2.z);
        std::deque<double> hip_buffer1(BUFFER_SIZE, 0.0);
        std::deque<double> hip_buffer2(BUFFER_SIZE, 0.0);

        // Poprzednie pozycje do ograniczenia prędkości zmian
        double prev_y1 = start_pos1.y;
        double prev_y2 = start_pos2.y;
        double prev_z1 = start_pos1.z;
        double prev_z2 = start_pos2.z;
        double prev_hip1 = 0.0;
        double prev_hip2 = 0.0;

        // Zwiększone limity zmian dla płynniejszego ruchu
        const double max_position_change = 0.5;
        const double max_hip_angle_change = 0.08;

        // Zmniejszony maksymalny kąt biodra
        const double max_hip_angle = 0.3;

        for (int step = 0; step <= STEPS && is_executing_; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;
            double x1 = start_pos1.x;
            double y1 = start_pos1.y;
            double z1 = start_pos1.z;
            double x2 = start_pos2.x;
            double y2 = start_pos2.y;
            double z2 = start_pos2.z;
            double current_hip1 = 0.0;
            double current_hip2 = 0.0;

            if (phase <= 0.5)
            {
                // Faza przenoszenia
                double swing_phase = phase * 2.0;

                // Zmodyfikowana funkcja smooth_swing dla łagodniejszego startu
                double smooth_swing = 0.5 * (1.0 - std::cos(M_PI * swing_phase));
                smooth_swing = smooth_swing * smooth_swing; // Dodatkowe wygładzenie

                y1 = start_pos1.y - params_.step_length * smooth_swing;
                y2 = start_pos2.y - params_.step_length * smooth_swing;

                // Wygładzony ruch w górę i w dół
                double height_phase = std::sin(M_PI * swing_phase);
                z1 = start_pos1.z - params_.step_height * height_phase;
                z2 = start_pos2.z - params_.step_height * height_phase;

                // Dodajemy ruch biodra podczas fazy przenoszenia
                if (leg1 <= 2) // Tylko dla przednich nóg
                    current_hip1 = max_hip_angle * std::sin(M_PI * swing_phase);
                if (leg2 <= 2)
                    current_hip2 = max_hip_angle * std::sin(M_PI * swing_phase);

                // Debug info dla fazy przenoszenia
                ROS_INFO_THROTTLE(0.1, "Leg %d SWING - Phase: %.2f, Y: %.2f, Z: %.2f, Hip: %.2f",
                                  leg1, phase, y1, z1, current_hip1);
            }
            else
            {
                // Faza podporowa
                double support_phase = (phase - 0.5) * 2.0;
                double smooth_support = 0.5 * (1.0 - std::cos(M_PI * support_phase));

                y1 = (start_pos1.y - params_.step_length) + params_.step_length * smooth_support;
                y2 = (start_pos2.y - params_.step_length) + params_.step_length * smooth_support;

                z1 = start_pos1.z;
                z2 = start_pos2.z;

                // Płynny powrót biodra do pozycji neutralnej
                if (leg1 <= 2)
                    current_hip1 = max_hip_angle * (1.0 - smooth_support);
                if (leg2 <= 2)
                    current_hip2 = max_hip_angle * (1.0 - smooth_support);

                // Debug info dla fazy podporowej
                ROS_INFO_THROTTLE(0.1, "Leg %d SUPPORT - Phase: %.2f, Y: %.2f, Z: %.2f, Hip: %.2f",
                                  leg1, phase, y1, z1, current_hip1);
            }

            // Wygładzanie przez średnią ruchomą
            y_buffer1.pop_front();
            y_buffer1.push_back(y1);
            y_buffer2.pop_front();
            y_buffer2.push_back(y2);
            z_buffer1.pop_front();
            z_buffer1.push_back(z1);
            z_buffer2.pop_front();
            z_buffer2.push_back(z2);
            hip_buffer1.pop_front();
            hip_buffer1.push_back(current_hip1);
            hip_buffer2.pop_front();
            hip_buffer2.push_back(current_hip2);

            // Obliczanie średnich
            double avg_y1 = 0.0, avg_y2 = 0.0, avg_z1 = 0.0, avg_z2 = 0.0;
            double avg_hip1 = 0.0, avg_hip2 = 0.0;

            for (double y : y_buffer1)
                avg_y1 += y;
            for (double y : y_buffer2)
                avg_y2 += y;
            for (double z : z_buffer1)
                avg_z1 += z;
            for (double z : z_buffer2)
                avg_z2 += z;
            for (double h : hip_buffer1)
                avg_hip1 += h;
            for (double h : hip_buffer2)
                avg_hip2 += h;

            y1 = avg_y1 / y_buffer1.size();
            y2 = avg_y2 / y_buffer2.size();
            z1 = avg_z1 / z_buffer1.size();
            z2 = avg_z2 / z_buffer2.size();
            double final_hip1 = avg_hip1 / hip_buffer1.size();
            double final_hip2 = avg_hip2 / hip_buffer2.size();

            // Debug info przed ograniczeniami
            ROS_DEBUG("Leg %d Before limits - Y: %.2f, Z: %.2f, Hip: %.2f",
                      leg1, y1, z1, final_hip1);

            // Ograniczenie maksymalnej zmiany pozycji
            y1 = prev_y1 + std::clamp(y1 - prev_y1, -max_position_change, max_position_change);
            y2 = prev_y2 + std::clamp(y2 - prev_y2, -max_position_change, max_position_change);
            z1 = prev_z1 + std::clamp(z1 - prev_z1, -max_position_change, max_position_change);
            z2 = prev_z2 + std::clamp(z2 - prev_z2, -max_position_change, max_position_change);
            final_hip1 = prev_hip1 + std::clamp(final_hip1 - prev_hip1, -max_hip_angle_change, max_hip_angle_change);
            final_hip2 = prev_hip2 + std::clamp(final_hip2 - prev_hip2, -max_hip_angle_change, max_hip_angle_change);

            // Debug info po ograniczeniach
            ROS_DEBUG("Leg %d After limits - Y: %.2f, Z: %.2f, Hip: %.2f",
                      leg1, y1, z1, final_hip1);

            // Zapamiętanie poprzednich pozycji
            prev_y1 = y1;
            prev_y2 = y2;
            prev_z1 = z1;
            prev_z2 = z2;
            prev_hip1 = final_hip1;
            prev_hip2 = final_hip2;

            // Aplikuj kinematykę odwrotną i wysyłaj komendy dla obu nóg
            double q1_1, q2_1, q3_1, q1_2, q2_2, q3_2;

            if (computeLegIK(leg1, x1, y1, z1, q1_1, q2_1, q3_1))
            {
                // Dla przednich nóg dodajemy obliczony kąt biodra
                if (leg1 <= 2)
                    q1_1 += final_hip1;

                // Debug info dla finalnych kątów
                ROS_INFO_THROTTLE(0.1, "Leg %d Final angles - q1: %.2f, q2: %.2f, q3: %.2f",
                                  leg1, q1_1, q2_1, q3_1);

                setLegJoints(leg1, q1_1, q2_1, q3_1);
            }

            if (computeLegIK(leg2, x2, y2, z2, q1_2, q2_2, q3_2))
            {
                // Dla przednich nóg dodajemy obliczony kąt biodra
                if (leg2 <= 2)
                    q1_2 += final_hip2;

                // Debug info dla finalnych kątów
                ROS_INFO_THROTTLE(0.1, "Leg %d Final angles - q1: %.2f, q2: %.2f, q3: %.2f",
                                  leg2, q1_2, q2_2, q3_2);

                setLegJoints(leg2, q1_2, q2_2, q3_2);
            }

            ros::Duration(dt).sleep();
            ros::spinOnce();
        }
    }

} // namespace hexapod