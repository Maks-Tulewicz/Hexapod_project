#ifndef TWO_LEG_GAIT_H
#define TWO_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"
#include <geometry_msgs/Twist.h>
#include <deque>
#include <map>
#include <vector>

namespace hexapod
{
    struct Parameters
    {
        double step_length;
        double step_height;
        double cycle_time;
        double standing_height;
    };

    struct LegPosition
    {
        double x;
        double y;
        double z;
    };

    class TwoLegGait : public BaseGait
    {
    public:
        explicit TwoLegGait(ros::NodeHandle &nh);
        ~TwoLegGait() override = default;

        void initialize() override;
        bool execute() override;
        void stop() override;

    private:
        Parameters params_;
        bool is_initialized_;
        bool is_executing_;

        void updateParameters(const ros::NodeHandle &nh);
        void makeStep(const geometry_msgs::Twist &cmd_vel);
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps);
        void moveLegPair(int leg1, int leg2,
                         const LegPosition &start_pos1,
                         const LegPosition &start_pos2);
    };

} // namespace hexapod

#endif // TWO_LEG_GAIT_H