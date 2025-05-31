#ifndef ONE_LEG_GAIT_H
#define ONE_LEG_GAIT_H

#include "hexapod_project/base_gait.h"

namespace hexapod
{

    class OneLegGait : public BaseGait
    {
    public:
        explicit OneLegGait(ros::NodeHandle &nh);

        bool execute() override;
        void setStepSize(double length, double height);
        void setSpeed(double cycle_time);

    private:
        void performStepCycle();

        // Sequence of legs for walking
        // Each leg moves one at a time in this sequence
        const std::vector<int> leg_sequence_ = {1, 4, 5, 2, 3, 6};
    };

} // namespace hexapod

#endif // ONE_LEG_GAIT_H