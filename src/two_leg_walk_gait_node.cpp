#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <map>
#include <string>
#include <vector>

// Definicje stałych dla wymiarów nóg (w centymetrach)
const double L1 = 2.0;  // Długość przesunięcia biodra
const double L2 = 8.0;  // Długość uda
const double L3 = 12.0; // Długość podudzia

// Klasa do publikowania pozycji przegubów
class JointPublisher
{
public:
    JointPublisher() {}
    JointPublisher(ros::NodeHandle &nh, const std::string &topic)
        : pub_(nh.advertise<std_msgs::Float64>(topic, 1)) {}

    void publish(double position)
    {
        std_msgs::Float64 msg;
        msg.data = position;
        pub_.publish(msg);
    }

private:
    ros::Publisher pub_;
};

// Struktura opisująca pozycję początkową nogi względem centrum robota
struct LegOrigin
{
    double x;         // Pozycja X względem środka robota
    double y;         // Pozycja Y względem środka robota
    bool invert_hip;  // Czy biodro ma odwrócony kierunek (dla prawej strony)
    bool invert_knee; // Czy kolano ma odwrócony kierunek (dla prawej strony)
};

// Mapa pozycji początkowych dla każdej nogi
const std::map<int, LegOrigin> leg_origins = {
    {1, {10.0, -10.0, false, false}}, // Lewa przednia
    {2, {-10.0, -10.0, true, true}},  // Prawa przednia
    {3, {12.0, 0.0, false, false}},   // Lewa środkowa
    {4, {-12.0, 0.0, true, true}},    // Prawa środkowa
    {5, {10.0, 10.0, false, false}},  // Lewa tylna
    {6, {-10.0, 10.0, true, true}}    // Prawa tylna
};

// Parametry chodu dwunożnego
struct TwoLegWalkParameters
{
    double step_length;     // Długość kroku
    double step_height;     // Wysokość podnoszenia nogi
    double cycle_time;      // Czas pełnego cyklu ruchu
    double body_shift;      // Przesunięcie ciała podczas przenoszenia nogi
    double standing_height; // Wysokość podczas stania
    double support_width;   // Szerokość rozstawu nóg podporowych
    double movement_delay;  // Opóźnienie między ruchami nóg w parze
};
// Lista nazw wszystkich przegubów
static const std::vector<std::string> joint_names = {
    "hip_joint_1", "hip_joint_2", "hip_joint_3",
    "hip_joint_4", "hip_joint_5", "hip_joint_6",
    "knee_joint_1", "knee_joint_2", "knee_joint_3",
    "knee_joint_4", "knee_joint_5", "knee_joint_6",
    "ankle_joint_1", "ankle_joint_2", "ankle_joint_3",
    "ankle_joint_4", "ankle_joint_5", "ankle_joint_6"};

// Inicjalizacja publisherów dla wszystkich przegubów
std::map<std::string, JointPublisher> init_publishers(ros::NodeHandle &nh)
{
    std::map<std::string, JointPublisher> pubs;
    for (const auto &j : joint_names)
    {
        std::string topic = "/" + j + "_position_controller/command";
        pubs.emplace(j, JointPublisher(nh, topic));
    }
    ROS_INFO("Połączono z kontrolerami - startujemy.");
    return pubs;
}

// Funkcja obliczająca kinematykę odwrotną dla pojedynczej nogi
bool computeLegIK(int leg_number, double x, double y, double z,
                  double &q1, double &q2, double &q3)
{
    const auto &leg = leg_origins.at(leg_number);

    // 1. Przekształcenie do lokalnego układu współrzędnych nogi
    double local_x = x - leg.x;
    double local_y = y - leg.y;

    // 2. Obliczenie kąta biodra (obrót wokół osi Z)
    q1 = std::atan2(local_y, local_x);
    if (leg.invert_hip)
    {
        q1 = q1 > 0 ? q1 - M_PI : q1 + M_PI;
    }

    // 3. Obliczenie odległości radialnej od osi biodra
    double r = std::sqrt(local_x * local_x + local_y * local_y) - L1;
    double h = -z; // Zmiana znaku, bo oś Z jest skierowana w dół

    // 4. Sprawdzenie czy punkt jest w zasięgu nogi
    double D2 = r * r + h * h;
    double D = std::sqrt(D2);
    if (D > (L2 + L3) || D < std::fabs(L2 - L3))
        return false;

    // 5. Obliczenie kątów kolana i kostki
    double cos_gamma = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    cos_gamma = std::max(-1.0, std::min(1.0, cos_gamma));
    double gamma = std::acos(cos_gamma);

    // 6. Obliczenie kąta kolana
    double alpha = std::atan2(h, r);
    double beta = std::acos((D2 + L2 * L2 - L3 * L3) / (2 * L2 * D));
    q2 = -(alpha - beta);

    // 7. Obliczenie kąta kostki
    if (leg.invert_knee)
    {
        q3 = gamma - M_PI; // Dla prawej strony
    }
    else
    {
        q3 = -(M_PI - gamma); // Dla lewej strony
    }

    return true;
}

// Struktura do przechowywania pozycji nogi
struct LegPosition
{
    double x;
    double y;
    double z;
};

// Funkcja do kontroli ruchu pojedynczej nogi
void moveSingleLeg(std::map<std::string, JointPublisher> &pubs,
                   int leg_number,
                   const LegPosition &start_pos,
                   const TwoLegWalkParameters &params)
{
    const int STEPS = 60;
    const double dt = 0.01; // 100Hz

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
            // Ruch do przodu
            y = start_pos.y - params.step_length *
                                  (1.0 - std::cos(M_PI * swing_phase));
            // Ruch w górę i w dół
            z = start_pos.z - params.step_height *
                                  std::sin(M_PI * swing_phase);
        }
        else
        {
            // Faza podporowa
            double support_phase = (phase - 0.5) * 2.0;
            y = (start_pos.y - params.step_length) +
                params.step_length * support_phase;
        }
        double q1, q2, q3;
        if (computeLegIK(leg_number, x, y, z, q1, q2, q3))
        {
            // Publikowanie pozycji przegubów
            std::string hip_joint = "hip_joint_" + std::to_string(leg_number);
            std::string knee_joint = "knee_joint_" + std::to_string(leg_number);
            std::string ankle_joint = "ankle_joint_" + std::to_string(leg_number);

            pubs[hip_joint].publish(q1);
            pubs[knee_joint].publish(q2);
            pubs[ankle_joint].publish(q3);
        }

        ros::Duration(dt).sleep();
    }
}

// Główna funkcja implementująca chód dwunożny
void walkTwoLegs(std::map<std::string, JointPublisher> &pubs,
                 const std::map<int, std::vector<double>> &base_positions,
                 int num_steps)
{
    TwoLegWalkParameters params{
        .step_length = 4.0,
        .step_height = 3.0,
        .cycle_time = 2.0,
        .body_shift = 1.5,
        .standing_height = -24.0,
        .support_width = 20.0,
        .movement_delay = 0.3};

    // Pary nóg do ruchu
    const std::vector<std::pair<int, int>> leg_pairs = {
        {1, 4}, // Lewa przednia i prawa środkowa
        {2, 5}, // Prawa przednia i lewa tylna
        {3, 6}  // Lewa środkowa i prawa tylna
    };

    for (int step = 0; step < num_steps && ros::ok(); ++step)
    {
        ROS_INFO("Krok %d z %d", step + 1, num_steps);

        // Wykonanie ruchu dla każdej pary nóg
        for (const auto &pair : leg_pairs)
        {
            const auto &base1 = base_positions.at(pair.first);
            const auto &base2 = base_positions.at(pair.second);

            LegPosition start_pos1 = {base1[0], base1[1], base1[2]};
            LegPosition start_pos2 = {base2[0], base2[1], base2[2]};

            // Ruch pierwszej nogi z pary
            moveSingleLeg(pubs, pair.first, start_pos1, params);
            ros::Duration(params.movement_delay).sleep();

            // Ruch drugiej nogi z pary
            moveSingleLeg(pubs, pair.second, start_pos2, params);
            ros::Duration(params.movement_delay).sleep();
        }

        ros::Duration(0.5).sleep(); // Pauza między cyklami
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "two_leg_walk_node");
    ros::NodeHandle nh;

    auto pubs = init_publishers(nh);

    // Pozycje bazowe dla nóg
    const std::map<int, std::vector<double>> base_positions = {
        {1, {18.0, -15.0, -24.0}},  // lewa przednia
        {2, {-18.0, -15.0, -24.0}}, // prawa przednia
        {3, {22.0, 0.0, -24.0}},    // lewa środkowa
        {4, {-22.0, 0.0, -24.0}},   // prawa środkowa
        {5, {18.0, 15.0, -24.0}},   // lewa tylna
        {6, {-18.0, 15.0, -24.0}}   // prawa tylna
    };

    ROS_INFO("Rozpoczynam chód dwunożny");
    walkTwoLegs(pubs, base_positions, 5); // 5 kroków

    return 0;
}