#ifndef TWO_LEG_GAIT_H
#define TWO_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <map>
#include <vector>
#include <deque>

namespace hexapod
{
    class TwoLegGait : public BaseGait
    {
    public:
        explicit TwoLegGait(ros::NodeHandle &nh);
        virtual ~TwoLegGait() = default;

        void initialize() override;
        bool execute() override;
        void stop() override;
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps = 1);

    private:
        struct Parameters
        {
            double step_length;
            double step_height;
            double cycle_time;
            double standing_height;
        } params_;

        void makeStep(const geometry_msgs::Twist &cmd_vel);
        void moveLegPair(int leg1, int leg2,
                         const std::vector<double> &start_pos1,
                         const std::vector<double> &start_pos2,
                         double movement_direction);

        void updateParameters(const ros::NodeHandle &nh);
        bool is_initialized_;
        bool is_executing_;
    };

} // namespace hexapod

#endif // TWO_LEG_GAIT_H