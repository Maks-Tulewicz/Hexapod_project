#ifndef ONE_LEG_GAIT_H
#define ONE_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

namespace hexapod
{
    struct WalkingParameters
    {
        double step_height;
        double step_length;
        double cycle_time;
        double standing_height;
        double body_shift;
    };

    class OneLegGait : public BaseGait
    {
    private:
        WalkingParameters params_;
        bool is_standing_;
        ros::Subscriber stand_up_sub_;

    public:
        explicit OneLegGait(ros::NodeHandle &nh);
        ~OneLegGait() override = default;

        void initialize() override;
        bool execute() override;
        void stop() override;

        void setParameters(const WalkingParameters &params) { params_ = params; }
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_steps = 1);
        bool isStanding() const { return is_standing_; }

    private:
        void standUpCallback(const std_msgs::EmptyConstPtr &msg);
        void makeStep(const geometry_msgs::Twist &cmd_vel);
        double smoothStep(double x);
    };

} // namespace hexapod

#endif // ONE_LEG_GAIT_H