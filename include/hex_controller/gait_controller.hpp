#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace hex_controller
{

    struct GaitParameters
    {
        double standing_height;
        double step_height;
        double cycle_time;
        double leg_x_offset;
        double leg_y_offset;
    };

    class GaitController
    {
    protected:
        ros::NodeHandle &nh_;
        GaitParameters params_;
        std::vector<ros::Publisher> leg_publishers_;

    public:
        explicit GaitController(ros::NodeHandle &nh);
        virtual ~GaitController() = default;

        virtual void step(const geometry_msgs::Twist &cmd_vel) = 0;
        virtual void standUp();

    protected:
        bool computeLegIK(int leg_id, double x, double y, double z,
                          double &q1, double &q2, double &q3);
        void setLegJoints(int leg_id, double q1, double q2, double q3);
        void adjustLegPosition(double &x, double &y,
                               const geometry_msgs::Twist &cmd_vel);
        void initializePublishers();
        void loadParameters();
    };

} // namespace hex_controller

#endif // GAIT_CONTROLLER_HPP