#ifndef ONE_LEG_GAIT_H
#define ONE_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"
#include <geometry_msgs/Twist.h>

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

    public:
        explicit OneLegGait(ros::NodeHandle &nh);
        ~OneLegGait() override = default;

        void initialize() override;
        bool execute() override;
        void stop() override;

        void setParameters(const WalkingParameters &params) { params_ = params; }
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_sequences = 1);

    private:
        void makeStep(const geometry_msgs::Twist &cmd_vel);

        // Uproszczone funkcje bez walidacji
        void moveSingleLegSimple(int leg_number,
                                 const std::vector<double> &base_pos,
                                 const geometry_msgs::Twist &cmd_vel);
        void stabilizeRemainingLegs(int moving_leg,
                                    const std::map<int, std::vector<double>> &base_positions);

        double smoothStep(double x);
    };

} // namespace hexapod

#endif // ONE_LEG_GAIT_H