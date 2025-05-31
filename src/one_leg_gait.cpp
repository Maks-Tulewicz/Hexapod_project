#include "hex_final_urdf_description/one_leg_gait.h"

namespace hexapod
{

    OneLegGait::OneLegGait(ros::NodeHandle &nh) : BaseGait(nh)
    {
        // Initialize parameters for single leg gait
        params_.step_length = 4.5; // Shorter steps for stability
        params_.step_height = 2.5; // Lower leg lifting
        params_.cycle_time = 2.0;  // Slower cycle
        params_.body_shift = 1.0;  // Less body shift
    }

    void OneLegGait::setStepSize(double length, double height)
    {
        params_.step_length = length;
        params_.step_height = height;
    }

    void OneLegGait::setSpeed(double cycle_time)
    {
        params_.cycle_time = cycle_time;
    }

    void OneLegGait::performStepCycle()
    {
        // Base positions for legs
        const std::map<int, LegPosition> base_positions = {
            {1, {18.0, -15.0, params_.standing_height}},  // left front
            {2, {-18.0, -15.0, params_.standing_height}}, // right front
            {3, {22.0, 0.0, params_.standing_height}},    // left middle
            {4, {-22.0, 0.0, params_.standing_height}},   // right middle
            {5, {18.0, 15.0, params_.standing_height}},   // left back
            {6, {-18.0, 15.0, params_.standing_height}}   // right back
        };

        // Move each leg in sequence
        for (int leg : leg_sequence_)
        {
            ROS_INFO("Moving leg %d", leg);

            const auto &base = base_positions.at(leg);

            // Define movement trajectory points
            LegPosition start_pos = base;
            LegPosition lift_pos = {
                base.x,
                base.y,
                base.z - params_.step_height};
            LegPosition forward_pos = {
                base.x,
                base.y + params_.step_length,
                base.z - params_.step_height};
            LegPosition end_pos = {
                base.x,
                base.y + params_.step_length,
                base.z};

            // Execute movement trajectory
            double phase_time = params_.cycle_time / 4.0;
            moveSingleLeg(leg, start_pos, lift_pos, phase_time);   // Lift leg
            moveSingleLeg(leg, lift_pos, forward_pos, phase_time); // Move forward
            moveSingleLeg(leg, forward_pos, end_pos, phase_time);  // Lower leg

            ros::Duration(0.1).sleep(); // Short pause between legs
        }
    }

    bool OneLegGait::execute()
    {
        if (!standUp())
        {
            ROS_ERROR("Failed to stand up");
            return false;
        }

        ros::Duration(1.0).sleep(); // Stabilization pause

        while (ros::ok())
        {
            performStepCycle();

            // Check for stop conditions or other commands
            if (!ros::ok())
                break;
        }

        return true;
    }

} // namespace hexapod