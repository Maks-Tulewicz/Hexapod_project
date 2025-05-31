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

        const int STEPS = 60;
        const double dt = 0.05;

        // Pozycje bazowe dla nóg
        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}}, // UWAGA: z jest ujemne!
            {2, {-18.0, -15.0, -24.0}},
            {3, {22.0, 0.0, -24.0}},
            {4, {-22.0, 0.0, -24.0}},
            {5, {18.0, 15.0, -24.0}},
            {6, {-18.0, 15.0, -24.0}}};

        for (int leg = 1; leg <= 6; ++leg)
        {
            ROS_INFO("=== Rozpoczynam ruch nogi %d ===", leg);
            const auto &base_pos = base_positions.at(leg);

            for (int step = 0; step <= STEPS; ++step)
            {
                double phase = static_cast<double>(step) / STEPS;

                // Startujemy od pozycji bazowej
                double x = base_pos[0];
                double y = base_pos[1];
                double z = base_pos[2]; // Już jest ujemne!

                if (phase <= 0.5)
                {
                    double swing_phase = phase * 2.0;
                    double step_length = params_.step_length;

                    if (cmd_vel.linear.x != 0.0)
                        step_length *= cmd_vel.linear.x > 0 ? 1.0 : -1.0;

                    // Ruch do przodu/tyłu
                    y += step_length * (1.0 - std::cos(M_PI * swing_phase));

                    // Podnoszenie nogi - dodajemy do z, bo z jest ujemne
                    z -= params_.step_height * std::sin(M_PI * swing_phase);
                }
                else
                {
                    double support_phase = (phase - 0.5) * 2.0;
                    double step_length = params_.step_length;

                    if (cmd_vel.linear.x != 0.0)
                        step_length *= cmd_vel.linear.x > 0 ? 1.0 : -1.0;

                    // Powrót do pozycji
                    y += step_length * (1.0 - support_phase);
                }

                if (step % 10 == 0)
                {
                    ROS_INFO("Noga %d, Krok %d/%d:", leg, step, STEPS);
                    ROS_INFO("  Pozycja: x=%.3f, y=%.3f, z=%.3f", x, y, z);
                    ROS_INFO("  Faza: %.2f", phase);
                }

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    double q1_deg = q1 * 180.0 / M_PI;
                    double q2_deg = q2 * 180.0 / M_PI;
                    double q3_deg = q3 * 180.0 / M_PI;

                    if (step % 10 == 0)
                    {
                        ROS_INFO("  Kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f",
                                 q1_deg, q2_deg, q3_deg);
                    }
                    setLegJoints(leg, q1, q2, q3);
                }

                ros::Duration(dt).sleep();
            }
            ROS_INFO("=== Zakończono ruch nogi %d ===\n", leg);
            ros::Duration(0.1).sleep();
        }
    }

    void OneLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps)
    {
        for (int i = 0; i < num_steps && ros::ok(); ++i)
        {
            makeStep(cmd_vel);
        }
    }

    double OneLegGait::smoothStep(double x)
    {
        // Funkcja wygładzająca ruch: 3x^2 - 2x^3
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

} // namespace hexapod