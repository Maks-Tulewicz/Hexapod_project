#ifndef SINGLE_LEG_GAIT_HPP
#define SINGLE_LEG_GAIT_HPP

#include "hex_controller/gait_controller.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hex_controller
{

    class SingleLegGait : public GaitController
    {
    private:
        // Struktura do przechowywania pozycji nogi
        struct LegPosition
        {
            double x;
            double y;
            double z;
        };

    public:
        explicit SingleLegGait(ros::NodeHandle &nh);
        void step(const geometry_msgs::Twist &cmd_vel) override;

    private:
        void moveSingleLeg(int leg_number,
                           const LegPosition &start_pos,
                           const geometry_msgs::Twist &cmd_vel);

        void makeStep(const geometry_msgs::Twist &cmd_vel);

        // Stałe dla chodu
        static constexpr double DEFAULT_STEP_LENGTH = 0.06; // 6cm
        static constexpr double DEFAULT_STEP_HEIGHT = 0.04; // 4cm
        static constexpr double LEG_CYCLE_TIME = 0.3;       // 300ms na cykl nogi

        // Pozycje bazowe dla nóg (x, y, z)
        const std::map<int, std::vector<double>> base_positions = {
            {1, {0.14, -0.10, -0.24}},  // lewa przednia
            {2, {-0.14, -0.10, -0.24}}, // prawa przednia
            {3, {0.17, 0.0, -0.24}},    // lewa środkowa
            {4, {-0.17, 0.0, -0.24}},   // prawa środkowa
            {5, {0.14, 0.10, -0.24}},   // lewa tylna
            {6, {-0.14, 0.10, -0.24}}   // prawa tylna
        };

        // Sekwencja ruchu nóg
        const std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};
    };

} // namespace hex_controller

#endif // SINGLE_LEG_GAIT_HPP