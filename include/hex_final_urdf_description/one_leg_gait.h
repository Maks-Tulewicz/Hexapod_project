#ifndef ONE_LEG_GAIT_H
#define ONE_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"

namespace hexapod
{

    struct GaitParams
    {
        double step_length;     // długość kroku
        double step_height;     // wysokość podnoszenia nogi
        double cycle_time;      // czas jednego cyklu ruchu nogi
        double body_shift;      // przesunięcie ciała podczas przenoszenia nogi
        double standing_height; // wysokość podczas stania
    };

    class OneLegGait : public BaseGait
    {
    private:
        GaitParams params_;
        std::vector<int> leg_sequence_;

    public:
        explicit OneLegGait(ros::NodeHandle &nh);

        void initialize() override;
        bool execute() override;
        void stop() override;

        void setStepSize(double length, double height);
        void setSpeed(double cycle_time);

    private:
        bool standUp();
        void performStepCycle();
        bool moveLeg(int leg_number, double x, double y, double z);
    };

} // namespace hexapod

#endif // ONE_LEG_GAIT_H