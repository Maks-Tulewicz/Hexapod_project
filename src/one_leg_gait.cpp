#include "hex_final_urdf_description/one_leg_gait.h"

namespace hexapod
{
    OneLegGait::OneLegGait(ros::NodeHandle &nh)
        : BaseGait(nh)
    {
        params_.step_height = 6.0;
        params_.step_length = 7.0;
        params_.cycle_time = 2.8;
        params_.standing_height = 24.0; // minus jest w obliczeniach
        params_.body_shift = 2.0;
    }

    void OneLegGait::initialize()
    {
        initializePublishers();
        ROS_INFO("One leg gait initialized");
    }

    bool OneLegGait::execute()
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

            walkForward(cmd_vel, 10); // Wykonaj 10 kroków
        }
        return true;
    }

    void OneLegGait::stop()
    {
        // Zatrzymaj ruch, ale zostaw robota w pozycji stojącej
        ROS_INFO("Stopping one leg gait");
    }

    void OneLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        if (!isStanding())
            return;

        // Stałe parametry chodu
        const int STEPS = 210;
        const double dt = 0.007;

        // Pozycje bazowe dla każdej nogi
        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}},  // Prawa przednia
            {2, {-18.0, -15.0, -24.0}}, // Prawa tylna
            {3, {22.0, 0.0, -24.0}},    // Prawa środkowa
            {4, {-22.0, 0.0, -24.0}},   // Lewa środkowa
            {5, {18.0, 15.0, -24.0}},   // Lewa przednia
            {6, {-18.0, 15.0, -24.0}}   // Lewa tylna
        };

        // Sekwencja nóg jak w oryginalnym kodzie
        const std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};

        for (int leg : leg_sequence)
        {
            ROS_INFO("Ruch nogi %d", leg);
            const auto &base_pos = base_positions.at(leg);

            // Parametry ruchu
            double step_length = params_.step_length;
            double step_height = params_.step_height;

            for (int step = 0; step <= STEPS; ++step)
            {
                double phase = static_cast<double>(step) / STEPS;

                double x = base_pos[0];
                double y = base_pos[1];
                double z = base_pos[2];

                if (phase <= 0.5)
                {
                    // Faza przenoszenia
                    double swing_phase = phase * 2.0;

                    // Ruch do przodu z funkcją cosinus dla płynności
                    double forward_motion = (1.0 - std::cos(M_PI * swing_phase));
                    y = base_pos[1] - step_length * forward_motion * (cmd_vel.linear.x > 0 ? 1.0 : -1.0);

                    // Sinusoidalny ruch w górę i w dół
                    z = base_pos[2] - step_height * std::sin(M_PI * swing_phase);
                }
                else
                {
                    // Faza podporowa - powrót do pozycji początkowej
                    double support_phase = (phase - 0.5) * 2.0;
                    double y_offset = step_length * (cmd_vel.linear.x > 0 ? 1.0 : -1.0);
                    y = (base_pos[1] - y_offset) + y_offset * support_phase;
                    z = base_pos[2];
                }

                // Obliczanie IK i ustawianie stawów
                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);

                    if (step % 20 == 0)
                    {
                        ROS_INFO("Noga %d, Krok %d/%d:", leg, step, STEPS);
                        ROS_INFO("  Pozycja: x=%.3f, y=%.3f, z=%.3f", x, y, z);
                        ROS_INFO("  Faza: %.2f", phase);
                        ROS_INFO("  Kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f",
                                 q1 * 180.0 / M_PI, q2 * 180.0 / M_PI, q3 * 180.0 / M_PI);
                    }
                }

                ros::Duration(dt).sleep();
            }

            // Krótka pauza między ruchami nóg
            ros::Duration(0.1).sleep();
        }
    }

    void OneLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        // Parametry stabilizacji
        const double step_pause = 0.2;  // Pauza między pojedynczymi krokami
        const double cycle_pause = 0.5; // Dłuższa pauza po pełnym cyklu (wszystkie nogi)

        // Sekwencja nóg zoptymalizowana dla stabilności
        const std::vector<std::vector<int>> leg_groups = {
            {1, 4, 5}, // pierwsza grupa (lewa przednia, prawa środkowa, lewa tylna)
            {2, 3, 6}  // druga grupa (prawa przednia, lewa środkowa, prawa tylna)
        };

        for (int step = 0; step < num_steps && ros::ok(); ++step)
        {
            ROS_INFO("Wykonuję krok %d z %d", step + 1, num_steps);

            // Wykonanie kroku przez pierwszą grupę nóg
            makeStep(cmd_vel);

            // Pauza na stabilizację po kroku
            ros::Duration(step_pause).sleep();

            // Co pełen cykl (co 2 kroki) dajemy dłuższą pauzę na stabilizację
            if (step % 2 == 1)
            {
                ROS_INFO("Stabilizacja po pełnym cyklu kroków");
                ros::Duration(cycle_pause).sleep();
            }
        }
    }

    double OneLegGait::smoothStep(double x)
    {
        // Funkcja wygładzająca ruch: 3x^2 - 2x^3
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

} // namespace hexapod