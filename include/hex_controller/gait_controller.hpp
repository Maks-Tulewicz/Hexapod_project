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
        double standing_height = 0.15; // domyślna wysokość stania
        double step_height = 0.05;     // wysokość kroku
        double cycle_time = 1.0;       // czas cyklu
        double leg_x_offset = 0.1;     // przesunięcie nogi w X
        double leg_y_offset = 0.1;     // przesunięcie nogi w Y
        double turning_radius = 0.3;   // promień skrętu
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