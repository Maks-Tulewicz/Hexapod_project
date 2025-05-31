#include "hex_final_urdf_description/one_leg_gait.h"

namespace hexapod
{
    OneLegGait::OneLegGait(ros::NodeHandle &nh)
        : BaseGait(nh), is_standing_(false)
    {
        params_.step_height = 5.0;
        params_.step_length = 10.0;
        params_.cycle_time = 1.0;
        params_.standing_height = 24.0;
        params_.body_shift = 5.0;

        stand_up_sub_ = nh_.subscribe<std_msgs::Empty>("stand_up", 1,
                                                       &OneLegGait::standUpCallback, this);
    }

    void OneLegGait::initialize()
    {
        initializePublishers();
        ROS_INFO("One leg gait initialized");
        standUp(); // Ustaw robota w pozycji stojącej
    }

    bool OneLegGait::execute()
    {
        if (!is_standing_)
        {
            ROS_WARN("Robot must be standing before walking");
            return false;
        }
        return true;
    }

    void OneLegGait::stop()
    {
        // Zatrzymaj ruch, ale zostaw robota w pozycji stojącej
        ROS_INFO("Stopping one leg gait");
    }

    void OneLegGait::standUpCallback(const std_msgs::EmptyConstPtr &msg)
    {
        if (!is_standing_)
        {
            if (BaseGait::standUp())
            {
                is_standing_ = true;
                ROS_INFO("Robot stood up successfully");
            }
            else
            {
                ROS_ERROR("Failed to stand up");
            }
        }
        else
        {
            ROS_INFO("Robot is already standing");
        }
    }
    void OneLegGait::makeStep(const geometry_msgs::Twist &cmd_vel)
    {
        if (!is_standing_)
            return;

        const int STEPS = 50;
        const double dt = params_.cycle_time / STEPS;

        // Implementacja pojedynczego kroku dla każdej nogi po kolei
        for (int leg = 1; leg <= 6; ++leg)
        {
            for (int step = 0; step <= STEPS; ++step)
            {
                double phase = static_cast<double>(step) / STEPS;
                double smooth_phase = smoothStep(phase);

                // Oblicz pozycję nogi
                double x = 0.0;
                double y = params_.step_length * std::sin(phase * M_PI);
                double z = -params_.standing_height +
                           params_.step_height * std::sin(phase * M_PI);

                // Uwzględnij kierunek ruchu z cmd_vel
                if (cmd_vel.linear.x != 0.0)
                    y *= cmd_vel.linear.x > 0 ? 1.0 : -1.0;

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }

                ros::Duration(dt).sleep();
            }
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