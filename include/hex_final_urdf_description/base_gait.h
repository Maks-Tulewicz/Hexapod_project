#ifndef BASE_GAIT_H
#define BASE_GAIT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <map>
#include <string>
#include <vector>

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
        ros::Subscriber stand_up_sub_;
        bool is_standing_;

        // Parametry geometryczne nóg (w cm)
        static constexpr double L1 = 6.5;  // hip → knee
        static constexpr double L2 = 10.5; // knee → ankle
        static constexpr double L3 = 20.5; // ankle → stopa

        // Pozycje bioder względem centrum robota
        static const std::map<int, LegOrigin> leg_origins;

    public:
        explicit BaseGait(ros::NodeHandle &nh);
        virtual ~BaseGait() = default;

        virtual void initialize() = 0;
        virtual bool execute() = 0;
        virtual void stop() = 0;

        // Metody związane ze stanem wstawania
        bool isStanding() const { return is_standing_; }
        bool standUp();

        // Nowe funkcje diagnostyczne
        bool debugLegIK(int leg_number, double x, double y, double z);
        void testAllBasePositions();

        // Public wrappers dla testowania
        bool computeLegIK(int leg_number, double x, double y, double z,
                          double &q1, double &q2, double &q3);
        void setLegJoints(int leg_number, double q1, double q2, double q3);

    protected:
        void initializePublishers();
        bool setJointPosition(const std::string &joint_name, double position);

    private:
        void standUpCallback(const std_msgs::Empty::ConstPtr &msg);
    };

} // namespace hexapod

#endif // BASE_GAIT_H