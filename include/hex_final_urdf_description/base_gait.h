#ifndef BASE_GAIT_H
#define BASE_GAIT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>

namespace hexapod
{

    struct LegOrigin
    {
        double x;
        double y;
        bool invert_hip;
        bool invert_knee;
    };

    class BaseGait
    {
    protected:
        ros::NodeHandle &nh_;
        std::map<std::string, ros::Publisher> joint_publishers_;

        // Parametry geometryczne nóg
        static constexpr double L1 = 6.5;  // hip → knee
        static constexpr double L2 = 10.5; // knee → ankle
        static constexpr double L3 = 20.5; // ankle → stopa

        // Pozycje bioder względem centrum robota (na podstawie URDF)
        static const std::map<int, LegOrigin> leg_origins;

    public:
        explicit BaseGait(ros::NodeHandle &nh);
        virtual ~BaseGait() = default;

        virtual void initialize() = 0;
        virtual bool execute() = 0;
        virtual void stop() = 0;

    protected:
        void initializePublishers();
        bool setJointPosition(const std::string &joint_name, double position);
        bool computeLegIK(int leg_number, double x, double y, double z,
                          double &hip_angle, double &knee_angle, double &ankle_angle);
        static const std::map<int, std::vector<double>> base_positions;
    };

} // namespace hexapod

#endif // BASE_GAIT_H