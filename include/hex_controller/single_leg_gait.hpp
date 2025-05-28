#pragma once

#include "hex_controller/gait_controller.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hex_controller
{
    // Struktura parametrów chodu
    struct WalkingParameters
    {
        double step_length;     // długość kroku
        double step_height;     // wysokość podnoszenia nogi
        double cycle_time;      // czas jednego cyklu ruchu nogi
        double body_shift;      // przesunięcie ciała podczas przenoszenia nogi
        double standing_height; // wysokość podczas stania
    };

    // W single_leg_gait.hpp dodajemy:
    class SingleLegGait : public GaitController
    {
    private:
        WalkingParameters params_;
        void initializeParameters();
        void moveSingleLeg(int leg_number, const std::vector<double> &start_pos,
                           const geometry_msgs::Twist &cmd_vel);
        void makeStep(const geometry_msgs::Twist &cmd_vel);
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps);

    public:
        SingleLegGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;
    };
} // namespace hex_controller