#ifndef ONE_LEG_GAIT_H
#define ONE_LEG_GAIT_H

#include "hex_final_urdf_description/base_gait.h"

namespace hexapod
{

    class OneLegGait : public BaseGait
    {
    private:
        // Parametry chodu
        double step_height_ = 2.0; // Wysokość podnoszenia nogi
        double step_length_ = 4.0; // Długość kroku
        double cycle_time_ = 1.0;  // Czas cyklu dla jednej nogi

        // Sekwencja nóg (numeracja od 1 do 6)
        std::vector<int> leg_sequence_ = {1, 4, 5, 2, 3, 6};

    public:
        explicit OneLegGait(ros::NodeHandle &nh);

        void initialize() override;
        bool execute() override;
        void stop() override;

        // Metody konfiguracyjne
        void setStepParameters(double height, double length, double time);

    private:
        bool moveLeg(int leg_number, double x, double y, double z);
        void performSingleStep(int leg_number);
    };

} // namespace hexapod

#endif // ONE_LEG_GAIT_H