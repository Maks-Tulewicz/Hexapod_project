#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

//---------------------------------------------------------------------
//  PARAMETRY GEOMETRII – cm (zgodne z modelem w URDF/XACRO)
//---------------------------------------------------------------------
static constexpr double L1 = 6.5;  // hip → knee  (offset w płaszczyźnie XY)
static constexpr double L2 = 10.5; // knee→ ankle (udziec)
static constexpr double L3 = 20.5; // ankle→ stopa (podudzie)

// Pozycje bioder względem centrum robota (na podstawie URDF)
struct LegOrigin
{
    double x;
    double y;
    bool invert_hip;  // Obrót biodra - niektóre nogi mają odwrócony kierunek
    bool invert_knee; // Obrót kolana - niektóre nogi mają odwrócony kierunek
};

// Dodajemy nową strukturę dla parametrów chodu
struct WalkingParameters
{
    double step_length;     // długość kroku
    double step_height;     // wysokość podnoszenia nogi
    double cycle_time;      // czas jednego cyklu ruchu nogi
    double body_shift;      // przesunięcie ciała podczas przenoszenia nogi
    double standing_height; // wysokość podczas stania
};

// Inicjalizacja pozycji początkowych bioder wszystkich nóg (z URDF)
static const std::map<int, LegOrigin> leg_origins = {
    {1, {0.068956, -0.077136, false, false}}, // lewa przednia
    {2, {-0.086608, -0.077136, true, true}},  // prawa przednia
    {3, {0.101174, 0.000645, false, false}},  // lewa środkowa
    {4, {-0.118826, -0.000645, true, true}},  // prawa środkowa
    {5, {0.068956, 0.078427, false, false}},  // lewa tylna
    {6, {-0.086608, 0.078427, true, true}}    // prawa tylna
};
//---------------------------------------------------------------------
//  Klasa usprawniająca publikowanie na /<joint>_position_controller/command
//---------------------------------------------------------------------
class JointPublisher
{
    ros::Publisher pub_;

public:
    JointPublisher() = default; //  wymagane przez std::map
    JointPublisher(ros::NodeHandle &nh, const std::string &topic)
    {
        pub_ = nh.advertise<std_msgs::Float64>(topic, 1);
        // poczekaj maks 3 s na subskrybenta (controller)
        ros::Time start = ros::Time::now();
        while (pub_.getNumSubscribers() == 0 && ros::ok() &&
               (ros::Time::now() - start).toSec() < 3.0)
            ros::Duration(0.05).sleep();
    }
    void publish(double val) const
    {
        std_msgs::Float64 m;
        m.data = val;
        pub_.publish(m);
    }
};

//---------------------------------------------------------------------
//  Lista wszystkich 18 przegubów
//---------------------------------------------------------------------
static const std::vector<std::string> joint_names = {
    "hip_joint_1", "hip_joint_2", "hip_joint_3",
    "hip_joint_4", "hip_joint_5", "hip_joint_6",
    "knee_joint_1", "knee_joint_2", "knee_joint_3",
    "knee_joint_4", "knee_joint_5", "knee_joint_6",
    "ankle_joint_1", "ankle_joint_2", "ankle_joint_3",
    "ankle_joint_4", "ankle_joint_5", "ankle_joint_6"};

std::map<std::string, JointPublisher> init_publishers(ros::NodeHandle &nh)
{
    std::map<std::string, JointPublisher> pubs;
    for (const auto &j : joint_names)
    {
        std::string topic = "/" + j + "_position_controller/command";
        pubs.emplace(j, JointPublisher(nh, topic));
    }
    ROS_INFO("Połączono z kontrolerami – startujemy.");
    return pubs;
}

//---------------------------------------------------------------------
//  KINEMATYKA ODWROTNA – dla wszystkich nóg
//---------------------------------------------------------------------
bool computeLegIK(int leg_number, double x, double y, double z,
                  double &q1, double &q2, double &q3)
{
    // Pobierz konfigurację dla danej nogi
    const auto &leg = leg_origins.at(leg_number);

    // 1) Przesuń współrzędne względem biodra danej nogi
    double local_x = x - leg.x;
    double local_y = y - leg.y;

    // 2) Obrót biodra wokół Z - nowa logika
    q1 = std::atan2(local_y, local_x);

    if (leg.invert_hip)
    {
        // Dla prawej strony (2,4,6) - obrót o 180 stopni
        if (q1 > 0)
            q1 = q1 - M_PI;
        else
            q1 = q1 + M_PI;
    }

    // 3) Dystans promieniowy od osi biodra minus przesunięcie L1
    double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
    double h = -z; // Oś Z w dół dodatnia

    // 4) Sprawdź zasięg
    double D2 = r * r + h * h;
    double D = std::sqrt(D2);
    if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        return false;

    // 5) Kąt gamma między L2 i L3
    double cos_gamma = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
    double gamma = std::acos(cos_gamma);

    // 6) Kolano (joint 2)
    double alpha = std::atan2(h, r);
    double beta = std::acos((D2 + L2 * L2 - L3 * L3) / (2 * L2 * D));

    // Podstawowe obliczenie dla kolana
    q2 = -(alpha - beta);

    // 7) Kostka (joint 3)
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
//---------------------------------------------------------------------
//  Dostęp do przegubów danej nogi
//---------------------------------------------------------------------
void setLegJoints(std::map<std::string, JointPublisher> &pubs,
                  int leg_number, double q1, double q2, double q3)
{
    std::string hip_joint = "hip_joint_" + std::to_string(leg_number);
    std::string knee_joint = "knee_joint_" + std::to_string(leg_number);
    std::string ankle_joint = "ankle_joint_" + std::to_string(leg_number);

    pubs[hip_joint].publish(q1);
    pubs[knee_joint].publish(q2);
    pubs[ankle_joint].publish(q3);
}

//---------------------------------------------------------------------
//  Sekwencja wstawania oparta na kinematyce odwrotnej
//---------------------------------------------------------------------
void stand_up_ik(std::map<std::string, JointPublisher> &pubs)
{
    const double height = -24.0;
    const double STEPS = 150; // Zwiększona liczba kroków dla płynności
    const double dt = 0.05;   // Wolniejszy ruch

    // Pozycje docelowe dla każdej nogi - rozszerzone pozycje dla lepszej stabilności
    const std::map<int, std::vector<double>> target_positions = {
        {1, {14.0, -12.0, height}},  // lewa przednia - bardziej na zewnątrz
        {2, {-14.0, -12.0, height}}, // prawa przednia - bardziej na zewnątrz
        {3, {17.0, 0.0, height}},    // lewa środkowa
        {4, {-17.0, 0.0, height}},   // prawa środkowa
        {5, {14.0, 12.0, height}},   // lewa tylna - bardziej na zewnątrz
        {6, {-14.0, 12.0, height}}   // prawa tylna - bardziej na zewnątrz
    };

    // Zacznij od wyższej pozycji początkowej
    const double start_height = -12.0; // Zwiększona wysokość początkowa

    // Interpolacja z funkcją wygładzającą
    for (int step = 1; step <= STEPS && ros::ok(); ++step)
    {
        // Wygładzona interpolacja zamiast liniowej
        double ratio = (1 - std::cos(M_PI * step / STEPS)) / 2.0;

        for (int leg = 1; leg <= 6; ++leg)
        {
            const auto &target = target_positions.at(leg);
            double z_current = start_height + ratio * (target[2] - start_height);

            // Dodaj stopniowe rozszerzanie nóg
            double x = target[0];
            double y = target[1] * ratio; // Stopniowo rozszerzaj nogi na boki

            double q1, q2, q3;
            if (computeLegIK(leg, x, y, z_current, q1, q2, q3))
            {
                setLegJoints(pubs, leg, q1, q2, q3);
            }
        }
        ros::Duration(dt).sleep();
    }

    ros::Duration(2.0).sleep();
    ROS_INFO("Robot wstał.");
}
//---------------------------------------------------------------------
//  Cykl ruchu dla wszystkich nóg z kinematyką odwrotną
//---------------------------------------------------------------------
void legs_cycle(std::map<std::string, JointPublisher> &pubs)
{
    const double z0 = -24.0; // Pozycja środkowa (wysokość nad ziemią)
    const double A = 5.0;    // Amplituda ruchu

    const int STEPS = 40; // ~0.8s na pełny cykl
    const int CYCLES = 4; // ~3.2s łącznego ruchu

    // Pozycje bazowe dla każdej nogi (x, y, z)
    const std::map<int, std::vector<double>> base_positions = {
        {1, {14.0, -10.0, z0}},  // przednia prawa
        {2, {-14.0, -10.0, z0}}, // przednia lewa
        {3, {17.0, 0.0, z0}},    // środkowa prawa
        {4, {-17.0, 0.0, z0}},   // środkowa lewa
        {5, {14.0, 10.0, z0}},   // tylna prawa
        {6, {-14.0, 10.0, z0}}   // tylna lewa
    };

    // Pętla dla cykli ruchu
    for (int c = 0; c < CYCLES && ros::ok(); ++c)
    {
        for (int s = 0; s <= STEPS && ros::ok(); ++s)
        {
            double phase = static_cast<double>(s) / STEPS; // 0...1

            // Dla każdej nogi
            for (int leg = 1; leg <= 6; ++leg)
            {
                const auto &base = base_positions.at(leg);

                // Oblicz nową wysokość z sinusoidalnym ruchem
                // Przesunięcie fazowe dla poszczególnych nóg (nogi diagonalne w przeciwfazie)
                double leg_phase = phase;
                if (leg == 1 || leg == 4 || leg == 5)
                {
                    leg_phase = (phase + 0.5) > 1.0 ? (phase + 0.5 - 1.0) : (phase + 0.5);
                }

                double z = base[2] + A * std::sin(2 * M_PI * leg_phase);

                double q1, q2, q3;
                if (computeLegIK(leg, base[0], base[1], z, q1, q2, q3))
                {
                    setLegJoints(pubs, leg, q1, q2, q3);
                }
            }

            ros::Duration(0.01).sleep(); // 50Hz
        }
    }

    // Po zakończeniu cykli, wróć do stabilnej pozycji
    for (int leg = 1; leg <= 6; ++leg)
    {
        const auto &base = base_positions.at(leg);
        double q1, q2, q3;
        if (computeLegIK(leg, base[0], base[1], base[2], q1, q2, q3))
        {
            setLegJoints(pubs, leg, q1, q2, q3);
        }
    }

    ros::Duration(1.0).sleep(); // Czas na stabilizację
}

// Struktura do przechowywania pozycji nogi
struct LegPosition
{
    double x;
    double y;
    double z;
};
void moveSingleLeg(std::map<std::string, JointPublisher> &pubs,
                   int leg_number,
                   const LegPosition &start_pos,
                   const WalkingParameters &params)
{
    const int STEPS = 60;
    const double dt = 0.005;

    for (int step = 0; step <= STEPS; ++step)
    {
        double phase = static_cast<double>(step) / STEPS;
        double x = start_pos.x;
        double y = start_pos.y;
        double z = start_pos.z;

        if (phase <= 0.5)
        {
            // Faza przenoszenia
            double swing_phase = phase * 2.0;

            // Ruch do przodu (odwrócony znak dla właściwego kierunku)
            y = start_pos.y - params.step_length * (1.0 - std::cos(M_PI * swing_phase));

            // Płynniejszy ruch w górę i w dół
            z = start_pos.z - params.step_height * std::sin(M_PI * swing_phase);
        }
        else
        {
            // Faza podporowa - powrót do pozycji początkowej
            double support_phase = (phase - 0.5) * 2.0;
            y = (start_pos.y - params.step_length) +
                params.step_length * support_phase;
        }

        double q1, q2, q3;
        if (computeLegIK(leg_number, x, y, z, q1, q2, q3))
        {
            setLegJoints(pubs, leg_number, q1, q2, q3);
        }

        ros::Duration(dt).sleep();
    }
}

//---------------------------------------------------------------------

// Nowa funkcja do wykonania jednego kroku
void makeStep(std::map<std::string, JointPublisher> &pubs,
              const std::map<int, std::vector<double>> &base_positions,
              const WalkingParameters &params)
{
    // Kolejność nóg w chodzie: 3, 1, 5, 4, 2, 6
    const std::vector<int> leg_sequence = {1, 6, 2, 5, 3, 4};

    for (int leg : leg_sequence)
    {
        ROS_INFO("Ruch nogi %d", leg);

        // Pozycja początkowa dla bieżącej nogi
        const auto &base = base_positions.at(leg);
        LegPosition start_pos = {base[0], base[1], base[2]};

        // Wykonaj ruch nogą
        moveSingleLeg(pubs, leg, start_pos, params);

        // Krótka pauza między ruchami nóg
        ros::Duration(0.1).sleep();
    }
}

// Nowa funkcja implementująca chód
void walkForward(std::map<std::string, JointPublisher> &pubs,
                 const std::map<int, std::vector<double>> &base_positions,
                 int num_steps)
{
    WalkingParameters params{
        .step_length = 5.0, // Zmniejszona długość kroku
        .step_height = 4.0, // Zmniejszona wysokość podnoszenia
        .cycle_time = 1.8,
        .body_shift = 2.0,
        .standing_height = -24.0};

    for (int step = 0; step < num_steps && ros::ok(); ++step)
    {
        ROS_INFO("Wykonuję krok %d z %d", step + 1, num_steps);
        makeStep(pubs, base_positions, params);
        ros::Duration(0.2).sleep(); // Pauza między krokami
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_kinematics_node");
    ros::NodeHandle nh;

    std::map<std::string, JointPublisher> pubs = init_publishers(nh);

    // Pozycje bazowe dla nóg - lekko zmodyfikowane dla lepszej stabilności
    const std::map<int, std::vector<double>> base_positions = {
        {1, {14.0, -12.0, -24.0}},  // lewa przednia
        {2, {-14.0, -12.0, -24.0}}, // prawa przednia
        {3, {17.0, 0.0, -24.0}},    // lewa środkowa
        {4, {-17.0, 0.0, -24.0}},   // prawa środkowa
        {5, {14.0, 12.0, -24.0}},   // lewa tylna
        {6, {-14.0, 12.0, -24.0}}   // prawa tylna
    };

    stand_up_ik(pubs);
    ros::Duration(2.0).sleep(); // Dłuższa pauza na stabilizację

    walkForward(pubs, base_positions, 5);

    return 0;
}