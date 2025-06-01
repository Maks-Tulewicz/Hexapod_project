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

        const int MAIN_STEPS = 150;
        const double dt = 0.02;

        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}},
            {2, {-18.0, -15.0, -24.0}},
            {3, {22.0, 0.0, -24.0}},
            {4, {-22.0, 0.0, -24.0}},
            {5, {18.0, 15.0, -24.0}},
            {6, {-18.0, 15.0, -24.0}}};

        const std::vector<int> leg_sequence = {1, 4, 5, 2, 3, 6};

        for (int leg : leg_sequence)
        {
            const auto &base_pos = base_positions.at(leg);

            // Określamy czy to noga zewnętrzna
            bool is_outer_leg = (leg == 1 || leg == 2 || leg == 5 || leg == 6);

            for (int step = 0; step <= MAIN_STEPS; ++step)
            {
                double phase = static_cast<double>(step) / MAIN_STEPS;

                auto smoothstep = [](double x) -> double
                {
                    if (x <= 0.0)
                        return 0.0;
                    if (x >= 1.0)
                        return 1.0;
                    return x * x * (3 - 2 * x);
                };

                double x = base_pos[0];
                double y = base_pos[1];
                double z = base_pos[2];

                double step_length = params_.step_length;
                if (cmd_vel.linear.x != 0.0)
                {
                    step_length *= cmd_vel.linear.x > 0 ? -1.0 : 1.0;
                }

                const double MAX_HEIGHT = params_.step_height * 0.75;

                if (phase < 0.5)
                { // Faza podnoszenia i ruchu do przodu
                    double lift_phase = smoothstep(phase * 2.0);

                    // Dodajemy komponent obrotu dla nóg zewnętrznych
                    if (is_outer_leg)
                    {
                        // Zwiększamy zakres ruchu w osi Y dla nóg zewnętrznych
                        double y_offset = step_length * 1.2; // Zwiększamy zakres ruchu
                        y = base_pos[1] - y_offset * lift_phase;

                        // Dodajemy ruch w osi X aby wymusić obrót w biodrze
                        double x_amplitude = 2.0; // Amplituda ruchu w osi X
                        x = base_pos[0] + x_amplitude * std::sin(lift_phase * M_PI);
                    }
                    else
                    {
                        y = base_pos[1] - step_length * lift_phase;
                    }

                    // Ruch w górę
                    double height_factor = std::sin(lift_phase * M_PI_2);
                    z = base_pos[2] - MAX_HEIGHT * height_factor;
                }
                else
                { // Faza opuszczania i powrotu
                    double return_phase = smoothstep((phase - 0.5) * 2.0);

                    if (is_outer_leg)
                    {
                        // Podobnie dla fazy powrotnej
                        double y_offset = step_length * 1.2;
                        y = base_pos[1] - y_offset * (1.0 - return_phase);

                        // Płynny powrót w osi X
                        double x_amplitude = 2.0;
                        x = base_pos[0] + x_amplitude * std::sin((1.0 - return_phase) * M_PI);
                    }
                    else
                    {
                        y = base_pos[1] - step_length * (1.0 - return_phase);
                    }

                    // Opuszczanie
                    double height_factor = std::cos(return_phase * M_PI_2);
                    z = base_pos[2] - MAX_HEIGHT * height_factor * 0.5;
                }

                // Wygładzenie końcowego ruchu
                if (phase > 0.85)
                {
                    double end_phase = smoothstep((phase - 0.85) / 0.15);
                    y = y * (1.0 - end_phase) + base_pos[1] * end_phase;
                    if (is_outer_leg)
                    {
                        x = x * (1.0 - end_phase) + base_pos[0] * end_phase;
                    }
                }

                // Ograniczenie maksymalnych zmian wysokości
                z = std::max(z, base_pos[2] - MAX_HEIGHT);
                z = std::min(z, base_pos[2] + MAX_HEIGHT * 0.1);

                // Obliczanie IK i ustawianie stawów
                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);

                    if (step % 20 == 0)
                    {
                        ROS_INFO("Noga %d, Krok %d/%d:", leg, step, MAIN_STEPS);
                        ROS_INFO("  Pozycja: x=%.3f, y=%.3f, z=%.3f", x, y, z);
                        ROS_INFO("  Faza: %.2f", phase);
                        ROS_INFO("  Kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f",
                                 q1 * 180.0 / M_PI, q2 * 180.0 / M_PI, q3 * 180.0 / M_PI);
                    }
                }

                ros::Duration(dt).sleep();
            }

            ros::Duration(0.02).sleep();
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