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
    const double final_height = -24.0;
    const double start_height = -15.0; // Start from a higher position
    const double STEPS = 200;          // More steps for smoother motion
    const double dt = 0.05;

    // Wider stance positions for better stability
    const std::map<int, std::vector<double>> target_positions = {
        {1, {18.0, -15.0, final_height}},  // lewa przednia - bardziej na zewnątrz
        {2, {-18.0, -15.0, final_height}}, // prawa przednia - bardziej na zewnątrz
        {3, {22.0, 0.0, final_height}},    // lewa środkowa - jeszcze bardziej na zewnątrz
        {4, {-22.0, 0.0, final_height}},   // prawa środkowa - jeszcze bardziej na zewnątrz
        {5, {18.0, 15.0, final_height}},   // lewa tylna - bardziej na zewnątrz
        {6, {-18.0, 15.0, final_height}}   // prawa tylna - bardziej na zewnątrz
    };

    // First phase: spread legs wider
    for (int step = 0; step <= STEPS / 2; ++step)
    {
        double phase = static_cast<double>(step) / (STEPS / 2);

        for (const auto &[leg, target] : target_positions)
        {
            // Start from current position and move outward
            double x = target[0] * phase;
            double y = target[1] * phase;
            double z = start_height; // Keep higher initially

            double q1, q2, q3;
            if (computeLegIK(leg, x, y, z, q1, q2, q3))
            {
                setLegJoints(pubs, leg, q1, q2, q3);
            }
        }
        ros::Duration(dt).sleep();
    }

    ros::Duration(1.0).sleep(); // Pause for stability

    // Second phase: lower the body
    for (int step = 0; step <= STEPS / 2; ++step)
    {
        double phase = static_cast<double>(step) / (STEPS / 2);
        double current_height = start_height + (final_height - start_height) * phase;

        for (const auto &[leg, target] : target_positions)
        {
            double q1, q2, q3;
            if (computeLegIK(leg, target[0], target[1], current_height, q1, q2, q3))
            {
                setLegJoints(pubs, leg, q1, q2, q3);
            }
        }
        ros::Duration(dt).sleep();
    }

    ros::Duration(2.0).sleep();
    ROS_INFO("Robot wstał stabilnie.");
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
        .step_length = 6.0, // Zmniejszona długość kroku
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
// Dodaj nową funkcję do równoczesnego ruchu pary nóg
void moveLegPair(std::map<std::string, JointPublisher> &pubs,
                 int leg1, int leg2,
                 const LegPosition &start_pos1,
                 const LegPosition &start_pos2,
                 const WalkingParameters &params)
{
    const int STEPS = 60;
    const double dt = 0.005;

    for (int step = 0; step <= STEPS; ++step)
    {
        double phase = static_cast<double>(step) / STEPS;

        // Obliczenia dla pierwszej nogi
        double x1 = start_pos1.x;
        double y1 = start_pos1.y;
        double z1 = start_pos1.z;

        // Obliczenia dla drugiej nogi
        double x2 = start_pos2.x;
        double y2 = start_pos2.y;
        double z2 = start_pos2.z;

        if (phase <= 0.5)
        {
            // Faza przenoszenia - obie nogi w powietrzu
            double swing_phase = phase * 2.0;

            // Pierwsza noga
            y1 = start_pos1.y - params.step_length * (1.0 - std::cos(M_PI * swing_phase));
            z1 = start_pos1.z - params.step_height * std::sin(M_PI * swing_phase);

            // Druga noga - ten sam ruch
            y2 = start_pos2.y - params.step_length * (1.0 - std::cos(M_PI * swing_phase));
            z2 = start_pos2.z - params.step_height * std::sin(M_PI * swing_phase);
        }
        else
        {
            // Faza podporowa - obie nogi wracają
            double support_phase = (phase - 0.5) * 2.0;

            // Pierwsza noga
            y1 = (start_pos1.y - params.step_length) + params.step_length * support_phase;

            // Druga noga
            y2 = (start_pos2.y - params.step_length) + params.step_length * support_phase;
        }

        // Zastosuj kinematykę odwrotną i wyślij komendy dla obu nóg jednocześnie
        double q1_1, q2_1, q3_1, q1_2, q2_2, q3_2;

        if (computeLegIK(leg1, x1, y1, z1, q1_1, q2_1, q3_1))
        {
            setLegJoints(pubs, leg1, q1_1, q2_1, q3_1);
        }

        if (computeLegIK(leg2, x2, y2, z2, q1_2, q2_2, q3_2))
        {
            setLegJoints(pubs, leg2, q1_2, q2_2, q3_2);
        }

        ros::Duration(dt).sleep();
    }
}

// Zmodyfikowana funkcja chodu dwunożnego
void walkForwardTwoLegs(std::map<std::string, JointPublisher> &pubs,
                        const std::map<int, std::vector<double>> &base_positions,
                        int num_steps)
{
    WalkingParameters params{
        .step_length = 6.0,      // Mniejsza długość kroku dla stabilności
        .step_height = 3.0,      // Mniejsza wysokość podnoszenia
        .cycle_time = 2.0,       // Wolniejszy cykl
        .body_shift = 1.5,       // Mniejsze przesunięcie ciała
        .standing_height = -24.0 // Standardowa wysokość
    };

    // Pary nóg do równoczesnego ruchu
    const std::vector<std::pair<int, int>> leg_pairs = {
        {1, 6}, // Lewa przednia i prawa tylna
        {2, 5}, // Prawa przednia i lewa tylna
        {3, 4}  // Lewa środkowa i prawa środkowa
    };

    for (int step = 0; step < num_steps && ros::ok(); ++step)
    {
        ROS_INFO("Wykonuję krok dwunożny %d z %d", step + 1, num_steps);

        // Wykonaj ruch dla każdej pary nóg
        for (const auto &pair : leg_pairs)
        {
            ROS_INFO("Przenoszenie pary nóg %d i %d", pair.first, pair.second);

            // Przygotuj pozycje początkowe dla obu nóg
            const auto &base1 = base_positions.at(pair.first);
            const auto &base2 = base_positions.at(pair.second);

            LegPosition start_pos1 = {base1[0], base1[1], base1[2]};
            LegPosition start_pos2 = {base2[0], base2[1], base2[2]};

            // Przenieś parę nóg jednocześnie
            moveLegPair(pubs, pair.first, pair.second, start_pos1, start_pos2, params);

            // Krótka pauza między parami dla stabilności
            ros::Duration(0.3).sleep();
        }

        // Pauza między pełnymi cyklami
        ros::Duration(0.5).sleep();
    }
}

// Funkcja do równoczesnego ruchu trzech nóg
void moveThreeLegs(std::map<std::string, JointPublisher> &pubs,
                   int leg1, int leg2, int leg3,
                   const LegPosition &start_pos1,
                   const LegPosition &start_pos2,
                   const LegPosition &start_pos3,
                   const WalkingParameters &params)
{
    const int STEPS = 60;
    const double dt = 0.005;

    for (int step = 0; step <= STEPS; ++step)
    {
        double phase = static_cast<double>(step) / STEPS;

        // Pozycje dla wszystkich trzech nóg
        std::vector<double> x = {start_pos1.x, start_pos2.x, start_pos3.x};
        std::vector<double> y = {start_pos1.y, start_pos2.y, start_pos3.y};
        std::vector<double> z = {start_pos1.z, start_pos2.z, start_pos3.z};
        std::vector<int> legs = {leg1, leg2, leg3};

        if (phase <= 0.5)
        {
            // Faza przenoszenia - wszystkie trzy nogi w powietrzu
            double swing_phase = phase * 2.0;

            for (int i = 0; i < 3; ++i)
            {
                y[i] = y[i] - params.step_length * (1.0 - std::cos(M_PI * swing_phase));
                z[i] = z[i] - params.step_height * std::sin(M_PI * swing_phase);
            }
        }
        else
        {
            // Faza podporowa - wszystkie nogi wracają
            double support_phase = (phase - 0.5) * 2.0;

            for (int i = 0; i < 3; ++i)
            {
                y[i] = (y[i] - params.step_length) + params.step_length * support_phase;
            }
        }

        // Aplikuj kinematykę odwrotną i wysyłaj komendy dla wszystkich nóg
        for (int i = 0; i < 3; ++i)
        {
            double q1, q2, q3;
            if (computeLegIK(legs[i], x[i], y[i], z[i], q1, q2, q3))
            {
                setLegJoints(pubs, legs[i], q1, q2, q3);
            }
        }

        ros::Duration(dt).sleep();
    }
}

// Funkcja implementująca chód trójnożny
void walkForwardThreeLegs(std::map<std::string, JointPublisher> &pubs,
                          const std::map<int, std::vector<double>> &base_positions,
                          int num_steps)
{
    WalkingParameters params{
        .step_length = 4.5,      // Mniejsza długość kroku dla większej stabilności
        .step_height = 2.5,      // Mniejsza wysokość podnoszenia
        .cycle_time = 2.0,       // Wolniejszy cykl
        .body_shift = 1.0,       // Mniejsze przesunięcie ciała
        .standing_height = -24.0 // Standardowa wysokość
    };

    // Dwie grupy po trzy nogi
    const std::vector<std::vector<int>> leg_groups = {
        {1, 4, 5}, // Grupa 1: lewa przednia, prawa środkowa, lewa tylna
        {2, 3, 6}  // Grupa 2: prawa przednia, lewa środkowa, prawa tylna
    };

    for (int step = 0; step < num_steps && ros::ok(); ++step)
    {
        ROS_INFO("Wykonuję krok trójnożny %d z %d", step + 1, num_steps);

        // Wykonaj ruch dla każdej grupy nóg
        for (const auto &group : leg_groups)
        {
            ROS_INFO("Przenoszenie grupy nóg %d, %d i %d", group[0], group[1], group[2]);

            // Przygotuj pozycje początkowe dla wszystkich nóg w grupie
            std::vector<LegPosition> start_positions;
            for (int leg : group)
            {
                const auto &base = base_positions.at(leg);
                start_positions.push_back({base[0], base[1], base[2]});
            }

            // Przenieś trzy nogi jednocześnie
            moveThreeLegs(pubs, group[0], group[1], group[2],
                          start_positions[0], start_positions[1], start_positions[2],
                          params);

            // Krótka pauza między grupami dla stabilności
            ros::Duration(0.4).sleep();
        }

        // Pauza między pełnymi cyklami
        ros::Duration(0.6).sleep();
    }
}

// Zmodyfikowany main do demonstracji wszystkich trybów chodu
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_gait_demo_node");
    ros::NodeHandle nh;

    std::map<std::string, JointPublisher> pubs = init_publishers(nh);

    // Pozycje bazowe dla nóg
    const std::map<int, std::vector<double>> base_positions = {
        {1, {18.0, -15.0, -24.0}},  // lewa przednia
        {2, {-18.0, -15.0, -24.0}}, // prawa przednia
        {3, {22.0, 0.0, -24.0}},    // lewa środkowa
        {4, {-22.0, 0.0, -24.0}},   // prawa środkowa
        {5, {18.0, 15.0, -24.0}},   // lewa tylna
        {6, {-18.0, 15.0, -24.0}}   // prawa tylna
    };

    // 1. Najpierw wstań
    ROS_INFO("Rozpoczynam sekwencję wstawania...");
    stand_up_ik(pubs);
    ros::Duration(2.0).sleep();

    // 2. Normalny chód
    ROS_INFO("\n=== Demonstracja normalnego chodu ===");
    walkForward(pubs, base_positions, 3);
    ros::Duration(2.0).sleep();

    // 3. Chód dwunożny
    ROS_INFO("\n=== Demonstracja chodu dwunożnego ===");
    walkForwardTwoLegs(pubs, base_positions, 3);
    ros::Duration(2.0).sleep();

    // 4. Chód trójnożny
    ROS_INFO("\n=== Demonstracja chodu trójnożnego ===");
    walkForwardThreeLegs(pubs, base_positions, 3);

    ROS_INFO("Demonstracja zakończona.");
    return 0;
}