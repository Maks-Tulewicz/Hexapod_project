#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>
#include <vector>

namespace hex_controller
{

    struct GaitParameters
    {
        double standing_height; // wysokość stania
        double step_height;     // wysokość kroku
        double cycle_time;      // czas cyklu
        double leg_x_offset;    // przesunięcie nogi w osi X
        double leg_y_offset;    // przesunięcie nogi w osi Y
    };

    class GaitController
    {
    protected:
        ros::NodeHandle &nh_;
        std::map<std::string, ros::Publisher> joint_publishers_;
        GaitParameters params_;

    public:
        explicit GaitController(ros::NodeHandle &nh);
        virtual ~GaitController() = default;

        virtual void step(const geometry_msgs::Twist &cmd_vel) = 0;
        virtual void standUp();

    protected:
        void initializePublishers();
        void loadParameters();
        bool computeLegIK(int leg_id, double x, double y, double z,
                          double &hip, double &knee, double &ankle);
        void setLegJoints(int leg_id, double hip, double knee, double ankle);
        void adjustLegPosition(double &x, double &y,
                               const geometry_msgs::Twist &cmd_vel);
    };

} // namespace hex_controller

#endif // GAIT_CONTROLLER_HPP