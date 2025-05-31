#include "hex_final_urdf_description/base_gait.h"
#include <cmath>

namespace hexapod
{
    // Initialize static member
    const std::map<int, LegOrigin> BaseGait::leg_origins = {
        {1, {0.068956, -0.077136, false, false}}, // lewa przednia
        {2, {-0.086608, -0.077136, true, true}},  // prawa przednia
        {3, {0.101174, 0.000645, false, false}},  // lewa środkowa
        {4, {-0.118826, -0.000645, true, true}},  // prawa środkowa
        {5, {0.068956, 0.078427, false, false}},  // lewa tylna
        {6, {-0.086608, 0.078427, true, true}}    // prawa tylna
    };

    BaseGait::BaseGait(ros::NodeHandle &nh)
        : nh_(nh), is_standing_(false)
    {
        // Subscriber do komendy stand_up
        stand_up_sub_ = nh_.subscribe<std_msgs::Empty>(
            "stand_up", 1, &BaseGait::standUpCallback, this);
    }

    void BaseGait::initializePublishers()
    {
        const std::vector<std::string> joint_types = {"hip", "knee", "ankle"};

        for (int leg = 1; leg <= 6; ++leg)
        {
            for (const auto &type : joint_types)
            {
                std::string joint_name = type + "_joint_" + std::to_string(leg);
                std::string topic = "/" + joint_name + "_position_controller/command";
                joint_publishers_[joint_name] = nh_.advertise<std_msgs::Float64>(topic, 1);
            }
        }
    }

    bool BaseGait::setJointPosition(const std::string &joint_name, double position)
    {
        auto it = joint_publishers_.find(joint_name);
        if (it == joint_publishers_.end())
        {
            ROS_ERROR("Joint publisher not found: %s", joint_name.c_str());
            return false;
        }

        std_msgs::Float64 msg;
        msg.data = position;
        it->second.publish(msg);
        return true;
    }

    void BaseGait::setLegJoints(int leg_number, double q1, double q2, double q3)
    {
        setJointPosition("hip_joint_" + std::to_string(leg_number), q1);
        setJointPosition("knee_joint_" + std::to_string(leg_number), q2);
        setJointPosition("ankle_joint_" + std::to_string(leg_number), q3);
    }

    bool BaseGait::computeLegIK(int leg_number, double x, double y, double z,
                                double &q1, double &q2, double &q3)
    {
        // Pobierz konfigurację dla danej nogi
        const auto &leg = leg_origins.at(leg_number);

        ROS_DEBUG("Leg %d IK input - x: %.2f, y: %.2f, z: %.2f", leg_number, x, y, z);

        // 1. Przekształcenie do lokalnego układu współrzędnych nogi
        double local_x = x - leg.x;
        double local_y = y - leg.y;

        // 2. Obliczenie kąta biodra (obrót wokół osi Z)
        q1 = std::atan2(local_y, local_x);

        // Inwersja kąta biodra dla prawych nóg
        if (leg.invert_hip)
        {
            if (q1 > 0)
                q1 = q1 - M_PI;
            else
                q1 = q1 + M_PI;
        }

        // 3. Obliczenie odległości radialnej od osi biodra
        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z; // Zmiana znaku, bo oś Z jest skierowana w dół

        // 4. Sprawdzenie czy punkt jest w zasięgu nogi
        double D2 = r * r + h * h;
        double D = std::sqrt(D2);

        if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        {
            ROS_DEBUG("Leg %d IK failed - Distance %.2f out of range [%.2f, %.2f]",
                      leg_number, D, std::fabs(L2 - L3), L2 + L3);
            return false;
        }

        // 5. Obliczenie gamma (kąt między L2 i L3)
        double cos_gamma = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
        cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
        double gamma = std::acos(cos_gamma);

        // 6. Obliczenie kąta kolana (q2)
        double alpha = std::atan2(h, r);
        double beta = std::acos((D2 + L2 * L2 - L3 * L3) / (2.0 * L2 * D));
        q2 = -(alpha - beta);

        // 7. Obliczenie kąta kostki (q3) - używamy sprawdzonej metody z walk_two_leg_gait
        if (leg.invert_knee)
        {
            // Dla prawej strony (2,4,6)
            q3 = gamma - M_PI;
        }
        else
        {
            // Dla lewej strony (1,3,5)
            q3 = -(M_PI - gamma);
        }

        return true;
    }

    bool BaseGait::standUp()
    {
        const double final_height = -24.0; // Końcowa wysokość
        const double start_height = -20.0; // Wyższa pozycja startowa
        const int STEPS = 200;             // Więcej kroków dla płynniejszego ruchu
        const double dt = 0.01;

        // Szersze rozstawienie nóg dla lepszej stabilności
        const std::map<int, std::vector<double>> target_positions = {
            {1, {18.0, -15.0, final_height}},  // lewa przednia - bardziej na zewnątrz
            {2, {-18.0, -15.0, final_height}}, // prawa przednia - bardziej na zewnątrz
            {3, {22.0, 0.0, final_height}},    // lewa środkowa - jeszcze bardziej na zewnątrz
            {4, {-22.0, 0.0, final_height}},   // prawa środkowa - jeszcze bardziej na zewnątrz
            {5, {18.0, 15.0, final_height}},   // lewa tylna - bardziej na zewnątrz
            {6, {-18.0, 15.0, final_height}}   // prawa tylna - bardziej na zewnątrz
        };

        // Faza 1: rozstaw nogi szerzej
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);

            for (const auto &[leg, target] : target_positions)
            {
                // Zacznij od aktualnej pozycji i przesuwaj na zewnątrz
                double x = target[0] * phase;
                double y = target[1] * phase;
                double z = start_height; // Utrzymuj wyżej na początku

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
                else
                {
                    ROS_ERROR("IK failed for leg %d during stand up", leg);
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(1.0).sleep(); // Pauza dla stabilizacji

        // Faza 2: opuść ciało
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);
            double current_height = start_height + (final_height - start_height) * phase;

            for (const auto &[leg, target] : target_positions)
            {
                double q1, q2, q3;
                if (computeLegIK(leg, target[0], target[1], current_height, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
                else
                {
                    ROS_ERROR("IK failed for leg %d during stand up", leg);
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(2.0).sleep();
        ROS_INFO("Robot wstał stabilnie.");
        return true;
    }
    void BaseGait::standUpCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (!is_standing_)
        {
            ROS_INFO("Rozpoczynam sekwencję wstawania...");
            if (standUp())
            {
                is_standing_ = true;
                ROS_INFO("Robot wstał pomyślnie");
            }
            else
            {
                ROS_ERROR("Nie udało się wstać");
            }
        }
        else
        {
            ROS_INFO("Robot już stoi");
        }
    }
} // namespace hexapod