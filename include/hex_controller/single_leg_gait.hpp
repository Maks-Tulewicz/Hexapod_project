#pragma once

#include "hex_controller/gait_controller.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hex_controller
{

    class SingleLegGait : public GaitController
    {
    private:
        // Struktura parametrów chodu
        struct WalkingParameters
        {
            double step_length;     // długość kroku
            double step_height;     // wysokość podnoszenia nogi
            double cycle_time;      // czas jednego cyklu ruchu nogi
            double body_shift;      // przesunięcie ciała podczas przenoszenia nogi
            double standing_height; // wysokość podczas stania
        };

        // Parametry dla interpolacji trajektorii
        struct TrajectoryParams
        {
            double lift_phase;     // Faza podnoszenia (0.0-0.3)
            double transfer_phase; // Faza przenoszenia (0.3-0.7)
            double lower_phase;    // Faza opuszczania (0.7-1.0)
            double stance_height;  // Wysokość podporu
            double clearance;      // Prześwit podczas przenoszenia
        };

        TrajectoryParams traj_params_;

    public:
        explicit SingleLegGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps);

    private:
        void initializeParameters();
        void makeStep(const geometry_msgs::Twist &cmd_vel);
        void moveSingleLeg(int leg_number,
                           const std::vector<double> &start_pos,
                           const geometry_msgs::Twist &cmd_vel);
        double smoothStep(double x);
    };
} // namespace hex_controller