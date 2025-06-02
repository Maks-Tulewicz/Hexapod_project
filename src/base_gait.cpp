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

        ROS_DEBUG("Leg %d - local coords: x=%.3f, y=%.3f", leg_number, local_x, local_y);

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

        ROS_DEBUG("Leg %d - hip angle before constraints: %.2f deg", leg_number, q1 * 180.0 / M_PI);

        // 3. Obliczenie odległości radialnej od osi biodra
        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z; // Zmiana znaku, bo oś Z jest skierowana w dół

        ROS_DEBUG("Leg %d - r=%.2f, h=%.2f", leg_number, r, h);

        // 4. Sprawdzenie czy punkt jest w zasięgu nogi
        double D2 = r * r + h * h;
        double D = std::sqrt(D2);

        ROS_DEBUG("Leg %d - distance D=%.2f, max_reach=%.2f, min_reach=%.2f",
                  leg_number, D, L2 + L3, std::fabs(L2 - L3));

        if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        {
            ROS_WARN("Leg %d IK failed - Distance %.2f out of range [%.2f, %.2f]",
                     leg_number, D, std::fabs(L2 - L3), L2 + L3);
            ROS_WARN("  Target: x=%.2f, y=%.2f, z=%.2f", x, y, z);
            ROS_WARN("  Local: x=%.2f, y=%.2f", local_x, local_y);
            ROS_WARN("  r=%.2f, h=%.2f", r, h);
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

        // 7. Obliczenie kąta kostki (q3)
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

        ROS_DEBUG("Leg %d final angles [deg]: hip=%.1f, knee=%.1f, ankle=%.1f",
                  leg_number, q1 * 180.0 / M_PI, q2 * 180.0 / M_PI, q3 * 180.0 / M_PI);

        return true;
    }

    bool BaseGait::debugLegIK(int leg_number, double x, double y, double z)
    {
        const auto &leg = leg_origins.at(leg_number);

        ROS_INFO("=== DEBUG IK dla nogi %d ===", leg_number);
        ROS_INFO("Cel: x=%.2f, y=%.2f, z=%.2f", x, y, z);
        ROS_INFO("Origin nogi: x=%.3f, y=%.3f", leg.x, leg.y);
        ROS_INFO("Flags: invert_hip=%s, invert_knee=%s",
                 leg.invert_hip ? "true" : "false",
                 leg.invert_knee ? "true" : "false");

        // Lokalne współrzędne
        double local_x = x - leg.x;
        double local_y = y - leg.y;
        ROS_INFO("Lokalne: x=%.2f, y=%.2f", local_x, local_y);

        // Odległość radialna
        double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
        double h = -z;
        double D = std::sqrt(r * r + h * h);

        ROS_INFO("r=%.2f, h=%.2f, D=%.2f", r, h, D);
        ROS_INFO("Zasięg: min=%.2f, max=%.2f", std::fabs(L2 - L3), L2 + L3);
        ROS_INFO("Długości segmentów: L1=%.1f, L2=%.1f, L3=%.1f", L1, L2, L3);

        if (D > (L2 + L3))
        {
            ROS_ERROR("Cel za daleko! D=%.2f > max=%.2f (różnica: %.2f)",
                      D, L2 + L3, D - (L2 + L3));
            return false;
        }

        if (D < std::fabs(L2 - L3))
        {
            ROS_ERROR("Cel za blisko! D=%.2f < min=%.2f", D, std::fabs(L2 - L3));
            return false;
        }

        ROS_INFO("IK wykonalne - cel w zasięgu");

        // Przetestuj rzeczywiste IK
        double q1, q2, q3;
        bool ik_result = computeLegIK(leg_number, x, y, z, q1, q2, q3);
        ROS_INFO("Rezultat IK: %s", ik_result ? "SUCCESS" : "FAILED");

        if (ik_result)
        {
            ROS_INFO("Kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f",
                     q1 * 180.0 / M_PI, q2 * 180.0 / M_PI, q3 * 180.0 / M_PI);
        }

        return ik_result;
    }

    void BaseGait::testAllBasePositions()
    {
        ROS_INFO("=== TESTOWANIE WSZYSTKICH POZYCJI BAZOWYCH ===");

        // Obecne pozycje bazowe z standUp()
        const std::map<int, std::vector<double>> base_positions = {
            {1, {18.0, -15.0, -24.0}}, // Oryginalne wartości
            {2, {-18.0, -15.0, -24.0}},
            {3, {22.0, 0.0, -24.0}},
            {4, {-22.0, 0.0, -24.0}},
            {5, {18.0, 15.0, -24.0}},
            {6, {-18.0, 15.0, -24.0}}};

        bool all_ok = true;

        for (const auto &[leg, pos] : base_positions)
        {
            ROS_INFO("\n--- NOGA %d ---", leg);
            bool result = debugLegIK(leg, pos[0], pos[1], pos[2]);
            if (!result)
                all_ok = false;

            // Test z krokiem do przodu
            double step_forward = 4.0;
            ROS_INFO("Test z krokiem do przodu (+%.1f):", step_forward);
            bool with_step = debugLegIK(leg, pos[0], pos[1] + step_forward, pos[2]);
            if (!with_step)
                all_ok = false;

            // Test z krokiem do tyłu
            ROS_INFO("Test z krokiem do tyłu (-%.1f):", step_forward);
            bool with_back_step = debugLegIK(leg, pos[0], pos[1] - step_forward, pos[2]);
            if (!with_back_step)
                all_ok = false;
        }

        ROS_INFO("\n=== PODSUMOWANIE TESTÓW ===");
        ROS_INFO("Wszystkie testy: %s", all_ok ? "PASSED" : "FAILED");

        if (!all_ok)
        {
            ROS_WARN("Niektóre pozycje są poza zasięgiem!");
            ROS_WARN("Rozważ zmniejszenie step_length lub pozycji bazowych");
        }
    }

    bool BaseGait::standUp()
    {
        const double final_height = -24.0; // Pozostawiamy oryginalną wysokość
        const double start_height = -20.0;
        const int STEPS = 200;
        const double dt = 0.03;

        // Oryginalne pozycje - sprawdzimy czy działają
        const std::map<int, std::vector<double>> target_positions = {
            {1, {18.0, -15.0, final_height}},
            {2, {-18.0, -15.0, final_height}},
            {3, {22.0, 0.0, final_height}},
            {4, {-22.0, 0.0, final_height}},
            {5, {18.0, 15.0, final_height}},
            {6, {-18.0, 15.0, final_height}}};

        ROS_INFO("Rozpoczynam stand up - testowanie pozycji docelowych...");

        // Test pozycji przed rozpoczęciem
        for (const auto &[leg, target] : target_positions)
        {
            double q1, q2, q3;
            if (!computeLegIK(leg, target[0], target[1], target[2], q1, q2, q3))
            {
                ROS_ERROR("Stand up impossible - leg %d target position unreachable!", leg);
                ROS_ERROR("Target: x=%.1f, y=%.1f, z=%.1f", target[0], target[1], target[2]);
                return false;
            }
        }

        ROS_INFO("Wszystkie pozycje docelowe osiągalne - kontynuuję stand up");

        // Faza 1: rozstaw nogi szerzej
        for (int step = 0; step <= STEPS / 2; ++step)
        {
            double phase = static_cast<double>(step) / (STEPS / 2);

            for (const auto &[leg, target] : target_positions)
            {
                double x = target[0] * phase;
                double y = target[1] * phase;
                double z = start_height;

                double q1, q2, q3;
                if (computeLegIK(leg, x, y, z, q1, q2, q3))
                {
                    setLegJoints(leg, q1, q2, q3);
                }
                else
                {
                    ROS_ERROR("IK failed for leg %d during stand up phase 1", leg);
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(1.0).sleep();

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
                    ROS_ERROR("IK failed for leg %d during stand up phase 2", leg);
                    return false;
                }
            }
            ros::Duration(dt).sleep();
        }

        ros::Duration(2.0).sleep();
        is_standing_ = true;
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