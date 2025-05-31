#include "hex_final_urdf_description/one_leg_gait.h"
#include <cmath>

namespace hexapod
{

    OneLegGait::OneLegGait(ros::NodeHandle &nh) : BaseGait(nh)
    {
        // Bezpieczniejsze parametry ruchu
        params_.step_length = 2.5;       // Bardzo krótkie kroki
        params_.step_height = 1.5;       // Niewielkie podnoszenie
        params_.cycle_time = 2.5;        // Wolny ruch
        params_.body_shift = 0.5;        // Minimalne przesunięcie ciała
        params_.standing_height = -24.0; // Standardowa wysokość stania

        // Sekwencja zapewniająca lepszą stabilność
        leg_sequence_ = {1, 6, 3, 4, 5, 2};

        ROS_INFO("OneLegGait zainicjalizowany z parametrami:");
        ROS_INFO("Step length: %.2f, Step height: %.2f",
                 params_.step_length, params_.step_height);
        ROS_INFO("Cycle time: %.2f, Body shift: %.2f, Standing height: %.2f",
                 params_.cycle_time, params_.body_shift, params_.standing_height);
    }
    void OneLegGait::initialize()
    {
        initializePublishers();
    }

    void OneLegGait::setStepSize(double length, double height)
    {
        params_.step_length = length;
        params_.step_height = height;
    }

    void OneLegGait::setSpeed(double cycle_time)
    {
        params_.cycle_time = cycle_time;
    }
    bool OneLegGait::standUp()
    {
        const double start_height = -15.0;
        const int STEPS = 200;
        const double dt = 0.05;

        // Początkowe pozycje (bliżej środka)
        const std::map<int, std::vector<double>> start_positions = {
            {1, {10.0, -8.0, start_height}},
            {2, {-10.0, -8.0, start_height}},
            {3, {12.0, 0.0, start_height}},
            {4, {-12.0, 0.0, start_height}},
            {5, {10.0, 8.0, start_height}},
            {6, {-10.0, 8.0, start_height}}};

        // Końcowe pozycje (szerzej rozstawione)
        const std::map<int, std::vector<double>> target_positions = {
            {1, {18.0, -15.0, params_.standing_height}},
            {2, {-18.0, -15.0, params_.standing_height}},
            {3, {22.0, 0.0, params_.standing_height}},
            {4, {-22.0, 0.0, params_.standing_height}},
            {5, {18.0, 15.0, params_.standing_height}},
            {6, {-18.0, 15.0, params_.standing_height}}};

        // Najpierw ustaw nogi w pozycji początkowej
        ROS_INFO("Ustawianie pozycji początkowej...");
        for (const auto &[leg, start] : start_positions)
        {
            if (!moveLeg(leg, start[0], start[1], start[2]))
            {
                ROS_ERROR("Nie udało się ustawić nogi %d w pozycji początkowej", leg);
                return false;
            }
        }
        ros::Duration(1.0).sleep();

        // Główna sekwencja wstawania
        ROS_INFO("Rozpoczynam sekwencję wstawania...");
        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            // Używamy funkcji sinusoidalnej dla płynniejszego ruchu
            double smooth_phase = (1.0 - std::cos(M_PI * phase)) / 2.0;

            for (const auto &[leg, target] : target_positions)
            {
                const auto &start = start_positions.at(leg);

                // Interpolacja między pozycją startową a końcową
                double x = start[0] + (target[0] - start[0]) * smooth_phase;
                double y = start[1] + (target[1] - start[1]) * smooth_phase;
                double z = start[2] + (target[2] - start[2]) * smooth_phase;

                if (!moveLeg(leg, x, y, z))
                {
                    ROS_ERROR("Błąd podczas wstawania - noga %d, faza %.2f", leg, phase);
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        // Końcowa stabilizacja
        ros::Duration(1.0).sleep();

        // Weryfikacja końcowej pozycji
        bool all_legs_ok = true;
        for (const auto &[leg, target] : target_positions)
        {
            if (!moveLeg(leg, target[0], target[1], target[2]))
            {
                ROS_ERROR("Noga %d nie osiągnęła pozycji końcowej", leg);
                all_legs_ok = false;
            }
        }

        if (all_legs_ok)
        {
            ROS_INFO("Robot wstał stabilnie.");
            return true;
        }
        else
        {
            ROS_ERROR("Nie wszystkie nogi osiągnęły pozycję końcową");
            return false;
        }
    }
    void OneLegGait::performStepCycle()
    {
        const int STEPS = 60;
        const double dt = 0.01; // Zwiększamy dt dla wolniejszego ruchu

        // Pozycje bazowe dla każdej nogi - zmniejszamy zasięg ruchu
        const std::map<int, std::vector<double>> base_positions = {
            {1, {16.0, -12.0, params_.standing_height}},  // Zmniejszone z 18.0, -15.0
            {2, {-16.0, -12.0, params_.standing_height}}, // Zmniejszone z -18.0, -15.0
            {3, {20.0, 0.0, params_.standing_height}},    // Zmniejszone z 22.0
            {4, {-20.0, 0.0, params_.standing_height}},   // Zmniejszone z -22.0
            {5, {16.0, 12.0, params_.standing_height}},   // Zmniejszone z 18.0, 15.0
            {6, {-16.0, 12.0, params_.standing_height}}   // Zmniejszone z -18.0, 15.0
        };

        for (int leg : leg_sequence_)
        {
            const auto &base = base_positions.at(leg);

            for (int step = 0; step <= STEPS; ++step)
            {
                double phase = static_cast<double>(step) / STEPS;
                double x = base[0];
                double y = base[1];
                double z = base[2];

                if (phase <= 0.5)
                {
                    // Faza przenoszenia - łagodniejszy ruch
                    double swing_phase = phase * 2.0;
                    double swing_sine = std::sin(M_PI * swing_phase);

                    // Zmniejszamy amplitudę ruchu i dodajemy płynniejszą trajektorię
                    y = base[1] - params_.step_length * (1.0 - swing_sine) * 0.5;
                    z = base[2] - params_.step_height * swing_sine;
                }
                else
                {
                    // Faza podporowa - wolniejszy powrót
                    double support_phase = (phase - 0.5) * 2.0;
                    y = (base[1] - params_.step_length * 0.5) +
                        params_.step_length * 0.5 * support_phase;
                }

                if (!moveLeg(leg, x, y, z))
                {
                    ROS_WARN("Leg %d movement failed at phase %.2f", leg, phase);
                    // Próba powrotu do pozycji bezpiecznej
                    moveLeg(leg, base[0], base[1], base[2]);
                    continue;
                }

                ros::Duration(dt).sleep();
            }

            // Zwiększamy pauzę między ruchami nóg
            ros::Duration(0.2).sleep();
        }
    }

    bool OneLegGait::moveLeg(int leg_number, double x, double y, double z)
    {
        ROS_INFO("Moving leg %d to position: [%.2f, %.2f, %.2f]", leg_number, x, y, z);

        double q1, q2, q3;
        if (!computeLegIK(leg_number, x, y, z, q1, q2, q3))
        {
            ROS_ERROR("IK failed for leg %d at position [%.2f, %.2f, %.2f]",
                      leg_number, x, y, z);
            return false;
        }

        ROS_INFO("IK solution for leg %d: [%.2f°, %.2f°, %.2f°]",
                 leg_number,
                 q1 * 180.0 / M_PI,
                 q2 * 180.0 / M_PI,
                 q3 * 180.0 / M_PI);

        std::string hip = "hip_joint_" + std::to_string(leg_number);
        std::string knee = "knee_joint_" + std::to_string(leg_number);
        std::string ankle = "ankle_joint_" + std::to_string(leg_number);

        if (!setJointPosition(hip, q1))
        {
            ROS_ERROR("Failed to set hip position for leg %d", leg_number);
            return false;
        }
        if (!setJointPosition(knee, q2))
        {
            ROS_ERROR("Failed to set knee position for leg %d", leg_number);
            return false;
        }
        if (!setJointPosition(ankle, q3))
        {
            ROS_ERROR("Failed to set ankle position for leg %d", leg_number);
            return false;
        }

        return true;
    }
    bool OneLegGait::execute()
    {
        ROS_INFO("Rozpoczynam wykonywanie chodu");

        // Najpierw wstań
        if (!standUp())
        {
            ROS_ERROR("Nie udało się wstać");
            return false;
        }

        // Wykonaj 3 cykle chodu
        for (int i = 0; i < 3 && ros::ok(); ++i)
        {
            ROS_INFO("Cykl chodu %d/3", i + 1);
            performStepCycle();
            ros::Duration(0.5).sleep();
        }

        return true;
    }

    void OneLegGait::stop()
    {
        ROS_INFO("Zatrzymywanie chodu");
    }

} // namespace hexapod