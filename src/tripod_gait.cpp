
// tripod_gait.cpp
#include "hex_final_urdf_description/tripod_gait.h"

namespace hexapod
{
    TripodalGait::TripodalGait(ros::NodeHandle &nh)
        : BaseGait(nh), is_initialized_(false), is_executing_(false)
    {
        // Parametry dla szybkiego chodu trypodalnego
        params_.step_length = 8.0;      // Długie kroki dla prędkości
        params_.step_height = 6.0;      // Wyższe podnoszenie dla przeszkód
        params_.cycle_time = 1.8;       // Szybki chód
        params_.standing_height = 24.0; // Standardowa wysokość
        params_.body_lean_angle = 0.05; // Lekkie pochylenie dla dynamiki
    }

    void TripodalGait::initialize()
    {
        initializePublishers();
        updateParameters(nh_);
        is_initialized_ = true;
        ROS_INFO("TripodalGait initialized - step_length=%.1f, step_height=%.1f, cycle_time=%.1f",
                 params_.step_length, params_.step_height, params_.cycle_time);
    }

    void TripodalGait::updateParameters(const ros::NodeHandle &nh)
    {
        nh.param("tripod_step_length", params_.step_length, 8.0);
        nh.param("tripod_step_height", params_.step_height, 6.0);
        nh.param("tripod_cycle_time", params_.cycle_time, 1.8);
        nh.param("tripod_standing_height", params_.standing_height, 24.0);
        nh.param("tripod_body_lean", params_.body_lean_angle, 0.05);
    }

    bool TripodalGait::execute()
    {
        if (!is_initialized_)
        {
            ROS_ERROR("TripodalGait not initialized!");
            return false;
        }

        // Wstań używając metody z BaseGait
        if (!standUp())
        {
            ROS_ERROR("Failed to stand up");
            return false;
        }

        // Wykonaj szybki ruch do przodu
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.2; // Wyższa prędkość niż inne chody
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        walkForward(cmd_vel, 5); // 5 cykli szybkiego chodu
        return true;
    }

    void TripodalGait::stop()
    {
        is_executing_ = false;
        ROS_INFO("TripodalGait stopped - robot remains standing");
    }

    void TripodalGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_cycles)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        is_executing_ = true;

        // Pozycje bazowe - szersze rozstawienie dla stabilności przy szybkim chodzie
        const std::map<int, LegPositionTripod> base_positions = {
            {1, {20.0, -18.0, -params_.standing_height}},  // lewa przednia
            {2, {-20.0, -18.0, -params_.standing_height}}, // prawa przednia
            {3, {25.0, 0.0, -params_.standing_height}},    // lewa środkowa
            {4, {-25.0, 0.0, -params_.standing_height}},   // prawa środkowa
            {5, {20.0, 18.0, -params_.standing_height}},   // lewa tylna
            {6, {-20.0, 18.0, -params_.standing_height}}   // prawa tylna
        };

        // Klasyczne grupy trypodalnego chodu
        // Trypod A: 1,4,5 (lewa przednia, prawa środkowa, lewa tylna)
        // Trypod B: 2,3,6 (prawa przednia, lewa środkowa, prawa tylna)
        const std::vector<int> tripod_A = {1, 4, 5};
        const std::vector<int> tripod_B = {2, 3, 6};

        for (int cycle = 0; cycle < num_cycles && is_executing_ && ros::ok(); ++cycle)
        {
            ROS_INFO("=== Wykonuję cykl chodu trypodalnego %d/%d ===", cycle + 1, num_cycles);

            // FAZA 1: Trypod A w ruchu, Trypod B podpiera
            executeTripodPhase(tripod_A, tripod_B, base_positions, cmd_vel, true);

            // Minimalna pauza - szybki chód
            ros::Duration(0.1).sleep();

            // FAZA 2: Trypod B w ruchu, Trypod A podpiera
            executeTripodPhase(tripod_B, tripod_A, base_positions, cmd_vel, false);

            // Krótka pauza między cyklami
            ros::Duration(0.15).sleep();
        }

        ROS_INFO("Zakończono chód trypodalny - robot pozostaje w pozycji stojącej");
    }

    void TripodalGait::executeTripodPhase(const std::vector<int> &moving_legs,
                                          const std::vector<int> &supporting_legs,
                                          const std::map<int, LegPositionTripod> &base_positions,
                                          const geometry_msgs::Twist &cmd_vel,
                                          bool is_first_phase)
    {
        const int STEPS = 80;    // Mniej kroków = szybszy ruch
        const double dt = 0.012; // Krótszy czas = szybszy chód

        // Oblicz przesunięcie - większe dla szybszego chodu
        double forward_step = params_.step_length * cmd_vel.linear.x;
        double side_step = params_.step_length * cmd_vel.linear.y;

        // Dynamiczne pochylenie ciała w kierunku ruchu
        double body_lean = params_.body_lean_angle * cmd_vel.linear.x;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            // === TRYPOD W RUCHU (swing phase) ===
            for (int leg : moving_legs)
            {
                const auto &base_pos = base_positions.at(leg);
                double x = base_pos.x;
                double y, z;

                if (phase <= 0.4)
                {
                    // Szybkie podnoszenie (40% czasu)
                    double swing_phase = phase / 0.4;

                    // Agresywny ruch do przodu
                    double progress = smoothStep(swing_phase);
                    y = base_pos.y + forward_step * (progress - 0.5);
                    x = base_pos.x + side_step * (progress - 0.5);

                    // Wysokie, szybkie podnoszenie
                    z = base_pos.z - params_.step_height * std::sin(M_PI * swing_phase);
                }
                else
                {
                    // Szybkie opuszczanie (60% czasu)
                    double swing_phase = (phase - 0.4) / 0.6;

                    // Kontynuacja ruchu
                    double progress = 0.5 + 0.5 * smoothStep(swing_phase);
                    y = base_pos.y + forward_step * (progress - 0.5);
                    x = base_pos.x + side_step * (progress - 0.5);

                    // Szybkie, kontrolowane lądowanie
                    z = base_pos.z - params_.step_height * std::sin(M_PI * (1.0 - swing_phase));
                }

                // Dodaj pochylenie ciała
                z += body_lean * 5.0; // Wzmocnienie efektu

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
            }

            // === TRYPOD PODPIERAJĄCY (stance phase) ===
            for (int leg : supporting_legs)
            {
                const auto &base_pos = base_positions.at(leg);

                // Nogi podpierające aktywnie przesuwają ciało
                double x = base_pos.x;
                double y = base_pos.y - forward_step * (phase - 0.5);
                double z = base_pos.z + body_lean * 3.0; // Kompensacja pochylenia

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
            }

            ros::Duration(dt).sleep();

            // Debug co 20 kroków
            if (step % 20 == 0)
            {
                ROS_DEBUG("Trypod %s, krok %d/%d (%.1f%%)",
                          is_first_phase ? "A" : "B", step, STEPS, phase * 100.0);
            }
        }
    }

    void TripodalGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        // Pojedynczy krok = jeden cykl trypodalny
        walkForward(cmd_vel, 1);
    }

    double TripodalGait::smoothStep(double x)
    {
        // Funkcja wygładzająca ruch: 3x² - 2x³
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

} // namespace hexapod
