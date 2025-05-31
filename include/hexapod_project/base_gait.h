#ifndef BASE_GAIT_H
#define BASE_GAIT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace hexapod
{

    // Geometry parameters from the URDF/XACRO model
    struct GeometryParams
    {
        static constexpr double L1 = 6.5;  // hip → knee  (offset in XY plane)
        static constexpr double L2 = 10.5; // knee → ankle (thigh)
        static constexpr double L3 = 20.5; // ankle → foot (shin)
    };

    // Leg origin structure (from URDF)
    struct LegOrigin
    {
        double x;
        double y;
        bool invert_hip;  // Hip rotation - some legs have inverted direction
        bool invert_knee; // Knee rotation - some legs have inverted direction
    };

    // Walking parameters structure
    struct WalkingParameters
    {
        double step_length;     // Length of a single step
        double step_height;     // Height of leg lifting
        double cycle_time;      // Time of one leg movement cycle
        double body_shift;      // Body shift during leg transfer
        double standing_height; // Height when standing
    };

    // Position structure for leg movement
    struct LegPosition
    {
        double x;
        double y;
        double z;
    };

    class BaseGait
    {
    protected:
        ros::NodeHandle &nh_;
        std::map<std::string, ros::Publisher> joint_publishers_;
        static const std::map<int, LegOrigin> leg_origins_;
        WalkingParameters params_;

    public:
        explicit BaseGait(ros::NodeHandle &nh);
        virtual ~BaseGait() = default;

        // Core functionality
        virtual void initialize();  // Initializes ROS publishers and parameters
        virtual bool execute() = 0; // Pure virtual - must be implemented by derived classes
        virtual void stop();        // Stops the current gait execution

    protected:
        // Inverse kinematics - calculates joint angles for given leg position
        bool computeLegIK(int leg_number, double x, double y, double z,
                          double &q1, double &q2, double &q3);

        // Moves a single leg from current position to target position
        void moveSingleLeg(int leg_number, const LegPosition &start_pos,
                           const LegPosition &end_pos, double duration);

        // Standing up sequence - brings robot to initial standing position
        bool standUp();

        // Linear interpolation between two positions
        LegPosition interpolatePosition(const LegPosition &start, const LegPosition &end,
                                        double phase);

        // Utility functions
        void setLegJoints(int leg_number, double q1, double q2, double q3);
        void initializePublishers();
        bool waitForControllers(double timeout = 3.0);

    private:
        static const std::vector<std::string> joint_names_;
        const double DEFAULT_STANDING_HEIGHT = -24.0;
        const int INTERPOLATION_STEPS = 60;
    };

} // namespace hexapod

#endif // BASE_GAIT_H