#include "hex_final_urdf_description/one_leg_gait.h"
#include <cmath>

namespace hexapod
{

    OneLegGait::OneLegGait(ros::NodeHandle &nh) : BaseGait(nh)
    {
        // Zmniejszamy parametry ruchu dla większej stabilności
        params_.step_length = 3.0;       // Zmniejszona z 4.5 na 3.0
        params_.step_height = 2.0;       // Zmniejszona z 2.5 na 2.0
        params_.cycle_time = 2.0;        // Zostawiamy bez zmian
        params_.body_shift = 0.5;        // Zmniejszona z 1.0 na 0.5
        params_.standing_height = -24.0; // Zostawiamy bez zmian

        // Zmieniamy kolejność nóg dla lepszej stabilności
        leg_sequence_ = {1, 6, 3, 4, 5, 2}; // Nowa sekwencja
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

        // Wider stance positions for better stability
        const std::map<int, std::vector<double>> target_positions = {
            {1, {18.0, -15.0, params_.standing_height}},
            {2, {-18.0, -15.0, params_.standing_height}},
            {3, {22.0, 0.0, params_.standing_height}},
            {4, {-22.0, 0.0, params_.standing_height}},
            {5, {18.0, 15.0, params_.standing_height}},
            {6, {-18.0, 15.0, params_.standing_height}}};

        // First phase: spread legs wider
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);

            for (const auto &[leg, target] : target_positions)
            {
                double x = target[0] * phase;
                double y = target[1] * phase;
                double z = start_height;

                if (!moveLeg(leg, x, y, z))
                {
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(1.0).sleep();

        // Second phase: lower the body
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);
            double current_height = start_height +
                                    (params_.standing_height - start_height) * phase;

            for (const auto &[leg, target] : target_positions)
            {
                if (!moveLeg(leg, target[0], target[1], current_height))
                {
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(2.0).sleep();
        ROS_INFO("Robot wstał stabilnie.");
        return true;
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
        double q1, q2, q3;
        if (!computeLegIK(leg_number, x, y, z, q1, q2, q3))
        {
            ROS_ERROR("IK failed for leg %d", leg_number);
            return false;
        }

        std::string hip = "hip_joint_" + std::to_string(leg_number);
        std::string knee = "knee_joint_" + std::to_string(leg_number);
        std::string ankle = "ankle_joint_" + std::to_string(leg_number);

        setJointPosition(hip, q1);
        setJointPosition(knee, q2);
        setJointPosition(ankle, q3);

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