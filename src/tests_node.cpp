#include "hex_final_urdf_description/base_gait.h"
#include "hex_final_urdf_description/one_leg_gait.h"
#include "hex_final_urdf_description/two_leg_gait.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <memory>
#include <iostream>

void printMenu()
{
    std::cout << "\n=== HEXAPOD GAIT SELECTION MENU ===" << std::endl;
    std::cout << "1. OneLegGait  (Najstabilniejszy - jedna noga na raz)" << std::endl;
    std::cout << "2. TwoLegGait  (Średnia stabilność - pary nóg)" << std::endl;
    std::cout << "3. TripodalGait (Najszybszy - 3 nogi jednocześnie) [TODO]" << std::endl;
    std::cout << "4. Stand Up Only (Tylko wstawanie)" << std::endl;
    std::cout << "5. DEBUG - Test pozycji bazowych i IK" << std::endl;
    std::cout << "6. DEBUG - Test pojedynczej nogi" << std::endl;
    std::cout << "0. Exit" << std::endl;
    std::cout << "Wybierz opcję (0-6): ";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexapod_test_node");
    ros::NodeHandle nh;

    ROS_INFO("=== HEXAPOD GAIT TESTING NODE ===");
    ROS_INFO("Oczekiwanie na uruchomienie kontrolerów...");

    // Daj czas na uruchomienie kontrolerów
    ros::Duration(3.0).sleep();

    int choice = -1;

    while (ros::ok() && choice != 0)
    {
        printMenu();
        std::cin >> choice;

        switch (choice)
        {
        case 1:
        {
            ROS_INFO("=== TESTOWANIE ONE LEG GAIT ===");
            ROS_INFO("Charakterystyka: Maksymalna stabilność, wolny chód");

            hexapod::OneLegGait gait(nh);
            gait.initialize();

            if (gait.execute())
            {
                ROS_INFO("OneLegGait wykonany pomyślnie");
            }
            else
            {
                ROS_ERROR("OneLegGait zakończony błędem");
            }

            gait.stop();
            break;
        }

        case 2:
        {
            ROS_INFO("=== TESTOWANIE TWO LEG GAIT ===");
            ROS_INFO("Charakterystyka: Średnia stabilność i prędkość");

            hexapod::TwoLegGait gait(nh);
            gait.initialize();

            if (gait.execute())
            {
                ROS_INFO("TwoLegGait wykonany pomyślnie");
            }
            else
            {
                ROS_ERROR("TwoLegGait zakończony błędem");
            }

            gait.stop();
            break;
        }

        case 3:
        {
            ROS_INFO("=== TRIPODAL GAIT - NOT IMPLEMENTED YET ===");
            ROS_WARN("Aby włączyć TripodalGait:");
            ROS_WARN("1. Stwórz plik include/hex_final_urdf_description/tripod_gait.h");
            ROS_WARN("2. Stwórz plik src/tripod_gait.cpp");
            ROS_WARN("3. Odkomentuj #include w tym pliku");
            ROS_WARN("4. Dodaj do CMakeLists.txt");
            break;
        }

        case 4:
        {
            ROS_INFO("=== TESTOWANIE STAND UP ONLY ===");

            hexapod::OneLegGait gait(nh);
            gait.initialize();

            if (gait.standUp())
            {
                ROS_INFO("Robot wstał pomyślnie");
                ROS_INFO("Robot pozostaje w pozycji stojącej przez 10 sekund...");
                ros::Duration(10.0).sleep();
            }
            else
            {
                ROS_ERROR("Nie udało się wstać");
            }
            break;
        }

        case 5:
        {
            ROS_INFO("=== DEBUG - TESTOWANIE POZYCJI BAZOWYCH ===");

            hexapod::OneLegGait gait(nh);
            gait.initialize();

            // Test wszystkich pozycji bazowych
            gait.testAllBasePositions();

            ROS_INFO("Test zakończony - sprawdź logi powyżej");
            break;
        }

        case 6:
        {
            ROS_INFO("=== DEBUG - TEST POJEDYNCZEJ NOGI ===");

            hexapod::OneLegGait gait(nh);
            gait.initialize();

            int leg_number;
            double x, y, z;

            std::cout << "Podaj numer nogi (1-6): ";
            std::cin >> leg_number;

            if (leg_number < 1 || leg_number > 6)
            {
                ROS_ERROR("Nieprawidłowy numer nogi!");
                break;
            }

            std::cout << "Podaj pozycję x [cm]: ";
            std::cin >> x;
            std::cout << "Podaj pozycję y [cm]: ";
            std::cin >> y;
            std::cout << "Podaj pozycję z [cm]: ";
            std::cin >> z;

            ROS_INFO("Testowanie nogi %d w pozycji (%.1f, %.1f, %.1f)", leg_number, x, y, z);

            bool result = gait.debugLegIK(leg_number, x, y, z);

            if (result)
            {
                ROS_INFO("Test zakończony SUKCESEM - pozycja osiągalna!");
                ROS_INFO("Użyj opcji 1 lub 2 aby zobaczyć rzeczywisty ruch");
            }
            else
            {
                ROS_ERROR("Test zakończony NIEPOWODZENIEM - pozycja poza zasięgiem");
            }

            break;
        }

        case 0:
        {
            ROS_INFO("Zamykanie programu testowego...");
            break;
        }

        default:
        {
            ROS_WARN("Nieprawidłowy wybór. Spróbuj ponownie.");
            break;
        }
        }

        if (choice != 0)
        {
            ROS_INFO("Naciśnij Enter aby kontynuować...");
            std::cin.ignore();
            std::cin.get();
        }
    }

    ROS_INFO("Program testowy zakończony");
    return 0;
}