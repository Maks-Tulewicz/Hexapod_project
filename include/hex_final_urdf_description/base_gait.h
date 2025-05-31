#ifndef BASE_GAIT_H
#define BASE_GAIT_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>

namespace hexapod
{

    class BaseGait
    {
    protected:
        ros::NodeHandle &nh_;
        std::map<std::string, ros::Publisher> joint_publishers_;

        // Parametry geometryczne nóg
        const double L1 = 6.5;  // Długość pierwszego segmentu
        const double L2 = 10.5; // Długość drugiego segmentu
        const double L3 = 20.5; // Długość trzeciego segmentu

    public:
        explicit BaseGait(ros::NodeHandle &nh) : nh_(nh) {}
        virtual ~BaseGait() = default;

        // Metody wirtualne do implementacji w klasach pochodnych
        virtual void initialize() = 0; // Inicjalizacja gaitu
        virtual bool execute() = 0;    // Wykonanie kroku
        virtual void stop() = 0;       // Zatrzymanie ruchu

    protected:
        // Metody pomocnicze
        void initializePublishers();
        bool setJointPosition(const std::string &joint_name, double position);
        bool calculateInverseKinematics(double x, double y, double z,
                                        double &hip_angle, double &knee_angle, double &ankle_angle);
    };

} // namespace hexapod

#endif // BASE_GAIT_H