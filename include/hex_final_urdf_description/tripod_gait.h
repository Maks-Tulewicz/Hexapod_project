
// tripod_gait.h
#ifndef TRIPOD_GAIT_H
#define TRIPOD_GAIT_H

#include "hex_final_urdf_description/base_gait.h"
#include <geometry_msgs/Twist.h>
#include <vector>

namespace hexapod
{
    struct TripodParameters
    {
        double step_length;
        double step_height;
        double cycle_time;
        double standing_height;
        double body_lean_angle; // Kąt pochylenia ciała dla dynamiki
    };

    struct LegPositionTripod
    {
        double x;
        double y;
        double z;
    };

    class TripodalGait : public BaseGait
    {
    public:
        explicit TripodalGait(ros::NodeHandle &nh);
        ~TripodalGait() override = default;

        void initialize() override;
        bool execute() override;
        void stop() override;

        void setParameters(const TripodParameters &params) { params_ = params; }
        void walkForward(const geometry_msgs::Twist &cmd_vel, int num_cycles = 1);

    private:
        TripodParameters params_;
        bool is_initialized_;
        bool is_executing_;

        void updateParameters(const ros::NodeHandle &nh);
        void makeStep(const geometry_msgs::Twist &cmd_vel);

        // Metoda wykonująca fazę trypodalną
        void executeTripodPhase(const std::vector<int> &moving_legs,
                                const std::vector<int> &supporting_legs,
                                const std::map<int, LegPositionTripod> &base_positions,
                                const geometry_msgs::Twist &cmd_vel,
                                bool is_first_phase);

        double smoothStep(double x);
    };

} // namespace hexapod

#endif // TRIPOD_GAIT_H
