#include "hex_final_urdf_description/one_leg_gait.h"

namespace hexapod
{
    OneLegGait::OneLegGait(ros::NodeHandle &nh)
        : BaseGait(nh)
    {
        // Konserwatywne parametry które na pewno działają
        params_.step_height = 3.0;
        params_.step_length = 3.0;
        params_.cycle_time = 3.5;
        params_.standing_height = 24.0;
        params_.body_shift = 1.0;
    }

    void OneLegGait::initialize()
    {
        initializePublishers();
        ROS_INFO("OneLegGait initialized - step_length=%.1f, step_height=%.1f, cycle_time=%.1f",
                 params_.step_length, params_.step_height, params_.cycle_time);
    }

    bool OneLegGait::execute()
    {
        if (!standUp())
        {
            ROS_ERROR("Failed to stand up");
            return false;
        }

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        walkForward(cmd_vel, 2);
        return true;
    }

    void OneLegGait::stop()
    {
        ROS_INFO("Stopping OneLegGait - robot remains standing");
    }

    void OneLegGait::walkForward(const geometry_msgs::Twist &cmd_vel, int num_sequences)
    {
        if (!isStanding())
        {
            ROS_WARN("Robot must be standing before walking");
            return;
        }

        const std::vector<int> safe_leg_sequence = {1, 4, 2, 5, 3, 6};

        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -params_.standing_height}},
            {2, {-18.0, -15.0, -params_.standing_height}},
            {3, {22.0, 0.0, -params_.standing_height}},
            {4, {-22.0, 0.0, -params_.standing_height}},
            {5, {18.0, 15.0, -params_.standing_height}},
            {6, {-18.0, 15.0, -params_.standing_height}}};

        for (int sequence = 0; sequence < num_sequences && ros::ok(); ++sequence)
        {
            ROS_INFO("=== Wykonuję sekwencję jednonożną %d/%d ===", sequence + 1, num_sequences);

            for (int leg : safe_leg_sequence)
            {
                if (!ros::ok())
                    break;

                ROS_INFO("Poruszam nogą %d", leg);

                // USUNIEMY WALIDACJĘ - po prostu spróbuj ruch
                moveSingleLegSimple(leg, base_positions.at(leg), cmd_vel);

                ros::Duration(0.4).sleep();
                stabilizeRemainingLegs(leg, base_positions);
                ros::Duration(0.2).sleep();
            }

            ros::Duration(0.6).sleep();
        }

        ROS_INFO("Zakończono chód jednonożny");
    }

    void OneLegGait::moveSingleLegSimple(int leg_number,
                                         const std::vector<double> &base_pos,
                                         const geometry_msgs::Twist &cmd_vel)
    {
        const int STEPS = 60;
        const double dt = 0.03;

        double step_length = params_.step_length * cmd_vel.linear.x;
        double step_height = params_.step_height;

        for (int step = 0; step <= STEPS; ++step)
        {
            double phase = static_cast<double>(step) / STEPS;

            double x = base_pos[0];
            double y = base_pos[1];
            double z = base_pos[2];

            if (phase <= 0.5)
            {
                // Faza swing - bardzo konserwatywna
                double swing_phase = phase / 0.5;

                // Prosta liniowa interpolacja
                y = base_pos[1] + step_length * (swing_phase - 0.5);

                // Niska, bezpieczna trajektoria
                z = base_pos[2] - step_height * std::sin(M_PI * swing_phase);
            }
            else
            {
                // Faza stance - powrót do pozycji bazowej
                double stance_phase = (phase - 0.5) / 0.5;

                y = base_pos[1] + step_length * 0.5 * (1.0 - stance_phase);
                z = base_pos[2];
            }

            // Prosta próba IK - jeśli nie wyjdzie, po prostu pomiń
            double q1, q2, q3;
            if (computeLegIK(leg_number, x, y, z, q1, q2, q3))
            {
                setLegJoints(leg_number, q1, q2, q3);
            }
            // Jeśli IK zawodzi, po prostu nie ruszamy nogą w tym kroku

            ros::Duration(dt).sleep();
        }
    }

    void OneLegGait::stabilizeRemainingLegs(int moving_leg,
                                            const std::map<int, std::vector<double>> &base_positions)
    {
        for (int leg = 1; leg <= 6; ++leg)
        {
            if (leg == moving_leg)
                continue;

            const auto &pos = base_positions.at(leg);
            double q1, q2, q3;

            if (computeLegIK(leg, pos[0], pos[1], pos[2], q1, q2, q3))
            {
                setLegJoints(leg, q1, q2, q3);
            }
        }
    }

    void OneLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        // Nie używamy tej funkcji w nowej implementacji
        ROS_INFO("Use walkForward() instead");
    }

    double OneLegGait::smoothStep(double x)
    {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

} // namespace hexapod